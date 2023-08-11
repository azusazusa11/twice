#include "platform.h"

#include <cerrno>
#include <chrono>
#include <cstring>
#include <format>
#include <iostream>

#include "screenshot.h"

namespace twice {

sdl_platform::sdl_platform()
{
	using namespace twice;

	if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_GAMECONTROLLER)) {
		throw sdl_error("init failed");
	}

	Uint32 window_flags = SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE;
	if (sdl_config.fullscreen) {
		window_flags |= SDL_WINDOW_FULLSCREEN_DESKTOP;
	}
	window_w = NDS_FB_W * sdl_config.window_scale;
	window_h = NDS_FB_H * sdl_config.window_scale;
	window = SDL_CreateWindow("Twice", SDL_WINDOWPOS_UNDEFINED,
			SDL_WINDOWPOS_UNDEFINED, window_w, window_h,
			window_flags);
	if (!window) {
		throw sdl_error("create window failed");
	}
	SDL_SetWindowResizable(window, SDL_TRUE);

	renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
	if (!renderer) {
		throw sdl_error("create renderer failed");
	}
	if (SDL_RenderSetLogicalSize(renderer, NDS_FB_W, NDS_FB_H)) {
		throw sdl_error("renderer set logical size failed");
	}

	texture = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_BGR888,
			SDL_TEXTUREACCESS_STREAMING, NDS_FB_W, NDS_FB_H);
	if (!texture) {
		throw sdl_error("create texture failed");
	}
	if (SDL_SetTextureScaleMode(texture, sdl_config.scale_mode)) {
		throw sdl_error("set texture scale mode failed");
	}

	int num_joysticks = SDL_NumJoysticks();
	for (int i = 0; i < num_joysticks; i++) {
		if (SDL_IsGameController(i)) {
			add_controller(i);
		}
	}

	setup_default_binds();
}

sdl_platform::~sdl_platform()
{
	while (!controllers.empty()) {
		SDL_JoystickID id = *controllers.begin();
		remove_controller(id);
	}

	SDL_DestroyTexture(texture);
	SDL_DestroyRenderer(renderer);
	SDL_DestroyWindow(window);
	SDL_Quit();
}

std::string
get_data_dir()
{
	std::string data_dir;

	char *path = SDL_GetPrefPath("", "twice");
	if (path) {
		data_dir = path;
		SDL_free(path);
	}

	return data_dir;
}

void
sdl_platform::render(void *fb)
{
	SDL_SetRenderDrawColor(renderer, 0x00, 0x00, 0x00, 0xFF);
	SDL_RenderClear(renderer);

	int pitch;
	void *p;
	SDL_LockTexture(texture, NULL, &p, &pitch);
	std::memcpy(p, fb, twice::NDS_FB_SZ_BYTES);
	SDL_UnlockTexture(texture);

	SDL_RenderCopy(renderer, texture, NULL, NULL);
	SDL_RenderPresent(renderer);
}

void
sdl_platform::arm_set_title_fps(
		std::uint64_t ticks_per_frame, std::uint64_t freq)
{
	double fps = (double)freq / ticks_per_frame;
	double frametime = 1000.0 * ticks_per_frame / freq;
	std::string title = std::format(
			"Twice [{:.2f} fps | {:.2f} ms]", fps, frametime);
	SDL_SetWindowTitle(window, title.c_str());
}

void
sdl_platform::loop(twice::nds_machine *nds)
{
	std::uint64_t freq = SDL_GetPerformanceFrequency();
	std::uint64_t tframe = freq / 59.8261;
	std::uint64_t start = SDL_GetPerformanceCounter();
	std::uint64_t ticks_elapsed = 0;

	running = true;
	throttle = true;

	while (running) {
		handle_events(nds);
		nds->run_frame();
		render(nds->get_framebuffer());

		if (throttle) {
			std::uint64_t elapsed =
					SDL_GetPerformanceCounter() - start;
			if (elapsed < tframe) {
				std::uint64_t remaining = tframe - elapsed;
				SDL_Delay(remaining * 1000.0 * 0.98 / freq);
			}
			while (SDL_GetPerformanceCounter() - start < tframe)
				;
		}

		std::uint64_t elapsed = SDL_GetPerformanceCounter() - start;
		start = SDL_GetPerformanceCounter();

		fps_counter.insert(elapsed);

		ticks_elapsed += elapsed;
		if (ticks_elapsed >= freq) {
			ticks_elapsed -= freq;
			arm_set_title_fps(fps_counter.get_average(), freq);
			update_rtc(nds);
		}
	}
}

void
sdl_platform::setup_default_binds()
{
	using enum nds_button;

	key_map = {
		{ SDLK_x, A },
		{ SDLK_z, B },
		{ SDLK_s, X },
		{ SDLK_a, Y },
		{ SDLK_w, R },
		{ SDLK_q, L },
		{ SDLK_1, START },
		{ SDLK_2, SELECT },
		{ SDLK_LEFT, LEFT },
		{ SDLK_RIGHT, RIGHT },
		{ SDLK_UP, UP },
		{ SDLK_DOWN, DOWN },
	};

	button_map = {
		{ SDL_CONTROLLER_BUTTON_B, A },
		{ SDL_CONTROLLER_BUTTON_A, B },
		{ SDL_CONTROLLER_BUTTON_Y, X },
		{ SDL_CONTROLLER_BUTTON_X, Y },
		{ SDL_CONTROLLER_BUTTON_RIGHTSHOULDER, R },
		{ SDL_CONTROLLER_BUTTON_LEFTSHOULDER, L },
		{ SDL_CONTROLLER_BUTTON_START, START },
		{ SDL_CONTROLLER_BUTTON_BACK, SELECT },
		{ SDL_CONTROLLER_BUTTON_DPAD_LEFT, LEFT },
		{ SDL_CONTROLLER_BUTTON_DPAD_RIGHT, RIGHT },
		{ SDL_CONTROLLER_BUTTON_DPAD_UP, UP },
		{ SDL_CONTROLLER_BUTTON_DPAD_DOWN, DOWN },
	};
}

void
sdl_platform::handle_events(twice::nds_machine *nds)
{
	SDL_Event e;

	while (SDL_PollEvent(&e)) {
		switch (e.type) {
		case SDL_QUIT:
			running = false;
			break;
		case SDL_KEYDOWN:
			handle_key_event(nds, e.key.keysym.sym, true);
			break;
		case SDL_KEYUP:
			handle_key_event(nds, e.key.keysym.sym, false);
			break;
		case SDL_CONTROLLERDEVICEADDED:
			add_controller(e.cdevice.which);
			break;
		case SDL_CONTROLLERDEVICEREMOVED:
			remove_controller(e.cdevice.which);
			break;
		case SDL_CONTROLLERBUTTONDOWN:
		{
			handle_controller_button_event(
					nds, e.cbutton.button, true);
			break;
		}
		case SDL_CONTROLLERBUTTONUP:
		{
			handle_controller_button_event(
					nds, e.cbutton.button, false);
			break;
		}
		case SDL_WINDOWEVENT:
			switch (e.window.event) {
			case SDL_WINDOWEVENT_SIZE_CHANGED:
				window_w = e.window.data1;
				window_h = e.window.data2;
				break;
			}
			break;
		}
	}

	update_touchscreen_state(nds);
}

void
sdl_platform::handle_key_event(nds_machine *nds, SDL_Keycode key, bool down)
{
	auto it = key_map.find(key);
	if (it != key_map.end()) {
		nds->button_event(it->second, down);
		return;
	}

	switch (key) {
	case SDLK_0:
		throttle = !throttle;
		break;
	case SDLK_o:
		take_screenshot(nds->get_framebuffer());
		break;
	}
}

void
sdl_platform::handle_controller_button_event(
		nds_machine *nds, int button, bool down)
{
	auto it = button_map.find(button);
	if (it != button_map.end()) {
		nds->button_event(it->second, down);
	}
}

void
sdl_platform::add_controller(int joystick_index)
{
	SDL_GameController *gc = SDL_GameControllerOpen(joystick_index);
	if (gc) {
		SDL_Joystick *joystick = SDL_GameControllerGetJoystick(gc);
		SDL_JoystickID id = SDL_JoystickInstanceID(joystick);
		controllers.insert(id);
	}
}

void
sdl_platform::remove_controller(SDL_JoystickID id)
{
	controllers.erase(id);

	SDL_GameController *gc = SDL_GameControllerFromInstanceID(id);
	if (gc) {
		SDL_GameControllerClose(gc);
	}
}

void
sdl_platform::update_touchscreen_state(twice::nds_machine *nds)
{
	int touch_x, touch_y;
	u32 mouse_buttons = SDL_GetMouseState(&touch_x, &touch_y);

	int ratio_cmp = window_w * 384 - window_h * 256;
	if (ratio_cmp == 0) {
		touch_x = touch_x * 256 / window_w;
		touch_y = (touch_y - window_h / 2) * 192 / (window_h / 2);
	} else if (ratio_cmp < 0) {
		touch_x = touch_x * 256 / window_w;
		int h = window_w * 384 / 256;
		touch_y = (touch_y - window_h / 2) * 192 / (h / 2);
	} else {
		int w = window_h * 256 / 384;
		touch_x = (touch_x - (window_w - w) / 2) * 256 / w;
		touch_y = (touch_y - window_h / 2) * 192 / (window_h / 2);
	}

	nds->update_touchscreen_state(
			touch_x, touch_y, mouse_buttons & SDL_BUTTON_LEFT);
}

void
sdl_platform::update_rtc(twice::nds_machine *nds)
{
	using namespace std::chrono;

	auto tp = zoned_time{ current_zone(), system_clock::now() }
	                          .get_local_time();
	auto dp = floor<days>(tp);
	year_month_day date{ dp };
	hh_mm_ss time{ tp - dp };
	weekday wday{ dp };

	nds->update_real_time_clock((int)date.year(), (unsigned)date.month(),
			(unsigned)date.day(), wday.iso_encoding(),
			time.hours().count(), time.minutes().count(),
			time.seconds().count());
}

void
sdl_platform::take_screenshot(void *fb)
{
	using namespace std::chrono;

	auto const tp = zoned_time{ current_zone(), system_clock::now() }
	                                .get_local_time();

	std::string filename = std::format("twice_screenshot_{:%Y%m%d-%H%M%S}",
			floor<seconds>(tp));
	std::string suffix = ".png";

	int err = write_nds_bitmap_to_png(fb, filename + suffix);
	if (!err) {
		return;
	}
	if (err != EEXIST) {
		goto error;
	}

	for (int i = 1; i < 10; i++) {
		suffix = std::format("_({}).png", i);
		err = write_nds_bitmap_to_png(fb, filename + suffix);
		if (!err) {
			return;
		}
		if (err != EEXIST) {
			goto error;
		}
	}

error:
	std::cout << "screenshot failed\n";
}

} // namespace twice
