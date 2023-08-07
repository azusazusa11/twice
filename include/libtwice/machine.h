#ifndef LIBTWICE_MACHINE_H
#define LIBTWICE_MACHINE_H

#include <cstddef>
#include <memory>
#include <string>

#include "libtwice/config.h"
#include "libtwice/filemap.h"

namespace twice {

struct nds_ctx;

struct nds_machine {
	enum class nds_button {
		A,
		B,
		SELECT,
		START,
		RIGHT,
		LEFT,
		UP,
		DOWN,
		R,
		L,
		X,
		Y,
		NONE,
	};

	nds_machine(const nds_config& config);
	~nds_machine();

	void load_cartridge(const std::string& pathname);
	void boot(bool direct_boot);
	void run_frame();
	void *get_framebuffer();
	void button_event(nds_button button, bool down);
	void update_touchscreen_state(int x, int y, bool down);

      private:
	nds_config config;
	file_map arm7_bios;
	file_map arm9_bios;
	file_map firmware;
	file_map cartridge;
	std::unique_ptr<nds_ctx> nds;
};

} // namespace twice

#endif
