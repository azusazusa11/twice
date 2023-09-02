#ifndef TWICE_GPU3D_H
#define TWICE_GPU3D_H

#include "common/types.h"

#include "nds/gpu/3d/matrix.h"
#include "nds/gpu/color.h"

namespace twice {

struct nds_ctx;

struct gpu_3d_engine {
	gpu_3d_engine(nds_ctx *nds);

	struct vertex {
		s32 x;
		s32 y;
		s32 z;
		s32 w;
		color6 color;
	};

	struct polygon {
		u32 num_vertices;
		vertex *vertices[10];
	};

	struct vertex_ram {
		vertex vertices[6144];
		u32 count{};
	} vtx_ram[2];

	struct polygon_ram {
		polygon polygons[2048];
		u32 count{};
	} poly_ram[2];

	struct geometry_engine {
		u32 polygon_attr_shadow{};
		u32 polygon_attr{};
		u32 vtx_count{};
		u32 teximage_param{};
		u8 primitive_type{};
		color6 vertex_color;
		s32 vx;
		s32 vy;
		s32 vz;
	} ge;

	struct rendering_engine {
		struct registers {
			u16 disp3dcnt{};
		};

		registers shadow;
		registers r;
	} re;

	u32 ge_buf = 0;
	u32 re_buf = 1;

	struct gxfifo {
		static constexpr size_t MAX_BUFFER_SIZE = 260;

		struct fifo_entry {
			u8 command{};
			u32 param{};
		};

		std::queue<fifo_entry> buffer;
	} fifo;

	u32 gxstat{};
	u32 viewport{};
	bool halted{};

	struct {
		u32 count{};
		u32 packed{};
		u32 num_params[4]{};
		u32 total_params{};
		std::vector<u32> params;
		u8 commands[4]{};
	} cmd_fifo;

	u32 cmd_params[32]{};

	ge_matrix projection_mtx;
	ge_matrix position_mtx;
	ge_matrix vector_mtx;
	ge_matrix texture_mtx;
	ge_matrix clip_mtx;
	u32 mtx_mode{};
	ge_matrix projection_stack[1]{};
	ge_matrix position_stack[32]{};
	ge_matrix vector_stack[32]{};
	ge_matrix texture_stack[1]{};
	u32 projection_sp{};
	u32 position_sp{};
	u32 texture_sp{};

	nds_ctx *nds{};
};

void gpu3d_on_vblank(gpu_3d_engine *gpu);
void gxfifo_check_irq(gpu_3d_engine *gpu);

u32 gpu_3d_read32(gpu_3d_engine *gpu, u16 offset);
u16 gpu_3d_read16(gpu_3d_engine *gpu, u16 offset);
u8 gpu_3d_read8(gpu_3d_engine *gpu, u16 offset);
void gpu_3d_write32(gpu_3d_engine *gpu, u16 offset, u32 value);
void gpu_3d_write16(gpu_3d_engine *gpu, u16 offset, u16 value);
void gpu_3d_write8(gpu_3d_engine *gpu, u16 offset, u8 value);

void ge_execute_command(gpu_3d_engine *gpu, u8 cmd);

} // namespace twice

#endif
