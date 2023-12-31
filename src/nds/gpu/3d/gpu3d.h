#ifndef TWICE_GPU3D_H
#define TWICE_GPU3D_H

#include "common/ringbuf.h"
#include "common/types.h"

#include "nds/gpu/3d/matrix.h"
#include "nds/gpu/color.h"

namespace twice {

struct nds_ctx;

struct gpu_3d_engine {
	struct vertex {
		s32 pos[4];

		/* r, g, b, s, t */
		s32 attr[5];

		s32 sx;
		s32 sy;
		u32 reused : 2 {};
		u32 strip_vtx : 2 {};
	};

	struct polygon {
		u32 num_vertices;
		vertex *vertices[10];
		s32 normalized_w[10];
		s32 z[10];
		u32 attr;
		u32 tx_param;
		u16 pltt_base;
		bool backface;
		int wshift;
		bool wbuffering;
		bool translucent;
		u32 start;
		u32 end;
		std::pair<int, s32> sortkey;
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
		u32 poly_attr{};
		u32 poly_attr_s{};
		u32 teximage_param{};
		u16 pltt_base{};
		u8 primitive_type{};
		s32 vx{};
		s32 vy{};
		s32 vz{};
		s32 texcoord[2]{};
		s32 vtx_texcoord[2]{};
		s32 light_vec[4][3]{};
		s32 half_vec[4][3]{};
		s32 normal_vec[3]{};
		u8 vtx_color[3]{};
		u8 light_color[4][3]{};
		u8 diffuse_color[3]{};
		u8 ambient_color[3]{};
		u8 specular_color[3]{};
		u8 emission_color[3]{};
		bool enable_shiny_table{};
		u8 shiny_table[128]{};
		u32 swap_bits{};
		u32 swap_bits_s{};
		s32 disp_1dot_depth{};

		vertex *last_strip_vtx[2]{};
		vertex vtx_buf[4];
		u32 vtx_count{};
	} ge;

	struct rendering_engine {
		struct registers {
			u16 disp3dcnt{};
			u32 clear_color{};
			u16 clear_depth{};
			u16 clrimage_offset{};
			u8 toon_table[64]{};
			u8 edge_color[16]{};
			u8 fog_table[32]{};
			u32 fog_color{};
			u16 fog_offset{};
			u8 alpha_test_ref{};

			bool operator==(const registers&) const = default;
		};

		registers shadow;
		registers r;

		bool manual_sort;
		bool last_poly_is_shadow_mask;
		u32 outside_opaque_id;
		s32 outside_depth;

		polygon *polygons[2048]{};
		u32 num_polygons{};

		struct interpolator {
			s32 x0;
			s32 x1;
			s32 x;
			s32 w0;
			s32 w1;
			s32 w0_n;
			s32 w0_d;
			s32 w1_d;
			s64 denom;
			u32 pfactor;
			u32 pfactor_r;
			int precision;
		};

		struct slope {
			s32 x0;
			s32 y0;
			s32 m;
			s32 dx;
			s32 dy;
			bool negative;
			bool xmajor;
			bool wide;
			bool vertical;
			bool left;
			bool line;
			vertex *v0{};
			vertex *v1{};

			interpolator interp;
		};

		struct polygon_info {
			u32 left;
			u32 right;
			u32 prev_left;
			u32 prev_right;
			slope left_slope;
			slope right_slope;
			s32 y_start;
			s32 y_end;
			bool shadow;
			u32 poly_id;
			s32 top_edge;
			s32 bottom_edge;
			bool horizontal_line;
		} poly_info[2048];
	} re;

	u32 ge_buf = 0;
	u32 re_buf = 1;

	struct gxfifo {
		static constexpr size_t MAX_BUFFER_SIZE = 260;

		struct fifo_entry {
			u8 command{};
			u32 param{};
		};

		ringbuf<fifo_entry, 512> buffer;
	} fifo;

	u32 gxstat{};
	u8 viewport_x[2]{};
	u8 viewport_y[2]{};
	u16 viewport_w{};
	u16 viewport_h{};
	bool halted{};
	bool render_frame = true;

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

	color4 color_buf[2][192][256]{};
	s32 depth_buf[2][192][256]{};
	bool stencil_buf[2][256]{};

	struct {
		u32 coverage : 5;
		u32 edge : 1;
		u32 fog : 1;
		u32 translucent_id : 6;
		u32 translucent : 1;
		u32 backface : 1;
		u32 opaque_id : 6;
	} attr_buf[2][192][256]{};

	nds_ctx *nds{};
};

void gpu3d_on_vblank(gpu_3d_engine *gpu);
void gpu3d_on_scanline_start(nds_ctx *nds);
void gpu3d_render_frame(gpu_3d_engine *gpu);
void gxfifo_check_irq(gpu_3d_engine *gpu);

u8 gpu_3d_read8(gpu_3d_engine *gpu, u16 offset);
u16 gpu_3d_read16(gpu_3d_engine *gpu, u16 offset);
u32 gpu_3d_read32(gpu_3d_engine *gpu, u16 offset);
void gpu_3d_write8(gpu_3d_engine *gpu, u16 offset, u8 value);
void gpu_3d_write16(gpu_3d_engine *gpu, u16 offset, u16 value);
void gpu_3d_write32(gpu_3d_engine *gpu, u16 offset, u32 value);

void ge_execute_command(gpu_3d_engine *gpu, u8 cmd);

} // namespace twice

#endif
