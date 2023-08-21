#ifndef TWICE_FIRMWARE_H
#define TWICE_FIRMWARE_H

#include "common/types.h"

namespace twice {

struct nds_ctx;

struct firmware_flash {
	firmware_flash(u8 *data);

	u8 *data{};
	u8 *user_settings{};

	u8 command{};
	u32 count{};
	u32 addr{};
	u8 status{};
	bool cs_active{};
};

void firmware_spi_transfer_byte(nds_ctx *nds, u8 value, bool keep_active);
void firmware_spi_reset(nds_ctx *nds);

} // namespace twice

#endif
