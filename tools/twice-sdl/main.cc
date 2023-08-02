#include <chrono>
#include <cstring>
#include <exception>
#include <format>
#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>

#include "libtwice/exception.h"
#include "libtwice/machine.h"
#include "libtwice/nds_defs.h"

#include "args.h"
#include "platform.h"

twice::arg_parser twice::parser;
twice::sdl_platform_config twice::sdl_config;

static std::string cartridge_pathname;
static std::string data_dir;

const std::vector<twice::option> twice::arg_parser::options = {
	{ "help", 'h', 0 },
	{ "verbose", 'v', 0 },
	{ "filter", '\0', 1 },
};

static void
print_usage()
{
	const std::string usage = "usage: twice [OPTION]... FILE\n";

	std::cerr << usage;
}

int
main(int argc, char **argv)
try {
	using twice::parser;
	using twice::sdl_config;

	if (parser.parse_args(argc, argv)) {
		print_usage();
		return 1;
	}

	if (parser.get_option("help")) {
		print_usage();
		return 0;
	}

	if (parser.num_args() == 0) {
		print_usage();
		return 1;
	}

	cartridge_pathname = parser.get_arg(0);
	if (auto opt = parser.get_option("verbose")) {
		twice::set_logger_verbose_level(opt->count);
	}

	if (auto opt = parser.get_option("filter")) {
		if (opt->arg == "nearest") {
			sdl_config.render_scale_quality = "0";
		} else if (opt->arg == "linear") {
			sdl_config.render_scale_quality = "1";
		} else {
			std::cerr << "argument to --filter must be one of:\n"
				  << "\tnearest\n"
				  << "\tlinear\n";
			return 1;
		}
	}

	if (data_dir.empty()) {
		data_dir = twice::get_data_dir();
		std::cerr << "data dir: " << data_dir << '\n';
	}

	twice::nds_config config{ data_dir };
	twice::nds_machine nds(config);
	nds.load_cartridge(cartridge_pathname);
	nds.direct_boot();

	twice::sdl_platform platform;
	platform.loop(&nds);

	return EXIT_SUCCESS;
} catch (const twice::twice_exception& e) {
	std::cerr << "fatal error: " << e.what() << '\n';
	return EXIT_FAILURE;
}
