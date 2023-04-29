#include "libtwice/machine.h"
#include "libtwice/exception.h"
#include "libtwice/nds.h"

using namespace twice;

Machine::Machine(Config& config)
	: config(config),
	  arm7_bios(config.data_dir + "bios7.bin", NDS_BIOS7_SIZE,
			  FileMap::MAP_EXACT_SIZE),
	  arm9_bios(config.data_dir + "bios9.bin", NDS_BIOS9_SIZE,
			  FileMap::MAP_EXACT_SIZE),
	  firmware(config.data_dir + "firmware.bin", NDS_FIRMWARE_SIZE,
			  FileMap::MAP_EXACT_SIZE)
{
}

Machine::~Machine() = default;

void
Machine::load_cartridge(const std::string& pathname)
{
	cartridge = { pathname, NDS_MAX_CART_SIZE, FileMap::MAP_MAX_SIZE };
}

void
Machine::direct_boot()
{
	if (!cartridge) {
		throw TwiceError("cartridge not loaded");
	}

	nds = std::make_unique<NDS>(arm7_bios.get_data(), arm9_bios.get_data(),
			firmware.get_data(), cartridge.get_data(),
			cartridge.get_size());

	nds->direct_boot();
}

void
Machine::run_frame()
{
	std::cerr << "run frame\n";
}
