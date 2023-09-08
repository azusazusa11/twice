#include "nds/nds.h"

#include "common/logger.h"

namespace twice {

using vertex = gpu_3d_engine::vertex;
using polygon = gpu_3d_engine::polygon;

static void
update_clip_matrix(gpu_3d_engine *gpu)
{
	mtx_mult_mtx(&gpu->clip_mtx, &gpu->projection_mtx, &gpu->position_mtx);
}

static void
cmd_mtx_mode(gpu_3d_engine *gpu)
{
	gpu->mtx_mode = gpu->cmd_params[0] & 3;
}

static void
update_projection_sp(gpu_3d_engine *gpu)
{
	gpu->projection_sp ^= 1;
	gpu->gxstat = (gpu->gxstat & ~BIT(13)) | (gpu->projection_sp << 13);
}

static void
update_position_sp(gpu_3d_engine *gpu, s32 offset)
{
	gpu->position_sp = (gpu->position_sp + offset) & 0x3F;
	gpu->gxstat = (gpu->gxstat & ~0x1F00) | (gpu->position_sp & 0x1F) << 8;
}

static void
cmd_mtx_push(gpu_3d_engine *gpu)
{
	switch (gpu->mtx_mode) {
	case 0:
		if (gpu->projection_sp == 1) {
			gpu->gxstat |= BIT(15);
		}
		gpu->projection_stack[0] = gpu->projection_mtx;
		update_projection_sp(gpu);
		break;
	case 1:
	case 2:
		if (gpu->position_sp >= 31) {
			gpu->gxstat |= BIT(15);
		}
		gpu->position_stack[gpu->position_sp & 31] = gpu->position_mtx;
		gpu->vector_stack[gpu->position_sp & 31] = gpu->vector_mtx;
		update_position_sp(gpu, 1);
		break;
	case 3:
		if (gpu->texture_sp == 1) {
			gpu->gxstat |= BIT(15);
		}
		gpu->texture_stack[0] = gpu->texture_mtx;
		gpu->texture_sp ^= 1;
	}
}

static void
cmd_mtx_pop(gpu_3d_engine *gpu)
{
	switch (gpu->mtx_mode) {
	case 0:
		update_projection_sp(gpu);
		if (gpu->projection_sp == 1) {
			gpu->gxstat |= BIT(15);
		}
		gpu->projection_mtx = gpu->projection_stack[0];
		break;
	case 1:
	case 2:
	{
		s32 offset = SEXT<6>(gpu->cmd_params[0]);
		update_position_sp(gpu, -offset);
		if (gpu->position_sp >= 31) {
			gpu->gxstat |= BIT(15);
		}
		gpu->position_mtx = gpu->position_stack[gpu->position_sp & 31];
		gpu->vector_mtx = gpu->vector_stack[gpu->position_sp & 31];
		break;
	}
	case 3:
		gpu->texture_sp ^= 1;
		if (gpu->texture_sp == 1) {
			gpu->gxstat |= BIT(15);
		}
		gpu->texture_mtx = gpu->texture_stack[0];
		break;
	}

	if (gpu->mtx_mode < 3) {
		update_clip_matrix(gpu);
	}
}

static void
cmd_mtx_store(gpu_3d_engine *gpu)
{
	switch (gpu->mtx_mode) {
	case 0:
		gpu->projection_stack[0] = gpu->projection_mtx;
		break;
	case 1:
	case 2:
	{
		u32 offset = gpu->cmd_params[0] & 0x1F;
		if (offset == 31) {
			gpu->gxstat |= BIT(15);
		}
		gpu->position_stack[offset] = gpu->position_mtx;
		gpu->vector_stack[offset] = gpu->vector_mtx;
		break;
	}
	case 3:
		gpu->texture_stack[0] = gpu->texture_mtx;
		break;
	}
}

static void
cmd_mtx_restore(gpu_3d_engine *gpu)
{
	switch (gpu->mtx_mode) {
	case 0:
		gpu->projection_mtx = gpu->projection_stack[0];
		break;
	case 1:
	case 2:
	{
		u32 offset = gpu->cmd_params[0] & 0x1F;
		if (offset == 31) {
			gpu->gxstat |= BIT(15);
		}
		gpu->position_mtx = gpu->position_stack[offset];
		gpu->vector_mtx = gpu->vector_stack[offset];
		break;
	}
	case 3:
		gpu->texture_mtx = gpu->texture_stack[0];
		break;
	}

	if (gpu->mtx_mode < 3) {
		update_clip_matrix(gpu);
	}
}

static void
cmd_mtx_identity(gpu_3d_engine *gpu)
{
	switch (gpu->mtx_mode) {
	case 0:
		mtx_set_identity(&gpu->projection_mtx);
		gpu->clip_mtx = gpu->position_mtx;
		break;
	case 1:
		mtx_set_identity(&gpu->position_mtx);
		gpu->clip_mtx = gpu->projection_mtx;
		break;
	case 2:
		mtx_set_identity(&gpu->position_mtx);
		mtx_set_identity(&gpu->vector_mtx);
		gpu->clip_mtx = gpu->projection_mtx;
		break;
	case 3:
		mtx_set_identity(&gpu->texture_mtx);
	}
}

static void
cmd_mtx_load_4x4(gpu_3d_engine *gpu)
{
	switch (gpu->mtx_mode) {
	case 0:
		mtx_load_4x4(&gpu->projection_mtx, gpu->cmd_params);
		break;
	case 1:
		mtx_load_4x4(&gpu->position_mtx, gpu->cmd_params);
		break;
	case 2:
		mtx_load_4x4(&gpu->position_mtx, gpu->cmd_params);
		mtx_load_4x4(&gpu->vector_mtx, gpu->cmd_params);
		break;
	case 3:
		mtx_load_4x4(&gpu->texture_mtx, gpu->cmd_params);
	}

	if (gpu->mtx_mode < 3) {
		update_clip_matrix(gpu);
	}
}

static void
cmd_mtx_load_4x3(gpu_3d_engine *gpu)
{
	switch (gpu->mtx_mode) {
	case 0:
		mtx_load_4x3(&gpu->projection_mtx, gpu->cmd_params);
		break;
	case 1:
		mtx_load_4x3(&gpu->position_mtx, gpu->cmd_params);
		break;
	case 2:
		mtx_load_4x3(&gpu->position_mtx, gpu->cmd_params);
		mtx_load_4x3(&gpu->vector_mtx, gpu->cmd_params);
		break;
	case 3:
		mtx_load_4x3(&gpu->texture_mtx, gpu->cmd_params);
	}

	if (gpu->mtx_mode < 3) {
		update_clip_matrix(gpu);
	}
}

static void
cmd_mtx_mult_4x4(gpu_3d_engine *gpu)
{
	switch (gpu->mtx_mode) {
	case 0:
		mtx_mult_4x4(&gpu->projection_mtx, gpu->cmd_params);
		break;
	case 1:
		mtx_mult_4x4(&gpu->position_mtx, gpu->cmd_params);
		break;
	case 2:
		mtx_mult_4x4(&gpu->position_mtx, gpu->cmd_params);
		mtx_mult_4x4(&gpu->vector_mtx, gpu->cmd_params);
		break;
	case 3:
		mtx_mult_4x4(&gpu->texture_mtx, gpu->cmd_params);
	}

	if (gpu->mtx_mode < 3) {
		update_clip_matrix(gpu);
	}
}

static void
cmd_mtx_mult_4x3(gpu_3d_engine *gpu)
{
	switch (gpu->mtx_mode) {
	case 0:
		mtx_mult_4x3(&gpu->projection_mtx, gpu->cmd_params);
		break;
	case 1:
		mtx_mult_4x3(&gpu->position_mtx, gpu->cmd_params);
		break;
	case 2:
		mtx_mult_4x3(&gpu->position_mtx, gpu->cmd_params);
		mtx_mult_4x3(&gpu->vector_mtx, gpu->cmd_params);
		break;
	case 3:
		mtx_mult_4x3(&gpu->texture_mtx, gpu->cmd_params);
	}

	if (gpu->mtx_mode < 3) {
		update_clip_matrix(gpu);
	}
}

static void
cmd_mtx_mult_3x3(gpu_3d_engine *gpu)
{
	switch (gpu->mtx_mode) {
	case 0:
		mtx_mult_3x3(&gpu->projection_mtx, gpu->cmd_params);
		break;
	case 1:
		mtx_mult_3x3(&gpu->position_mtx, gpu->cmd_params);
		break;
	case 2:
		mtx_mult_3x3(&gpu->position_mtx, gpu->cmd_params);
		mtx_mult_3x3(&gpu->vector_mtx, gpu->cmd_params);
		break;
	case 3:
		mtx_mult_3x3(&gpu->texture_mtx, gpu->cmd_params);
	}

	if (gpu->mtx_mode < 3) {
		update_clip_matrix(gpu);
	}
}

static void
cmd_mtx_scale(gpu_3d_engine *gpu)
{
	switch (gpu->mtx_mode) {
	case 0:
		mtx_scale(&gpu->projection_mtx, gpu->cmd_params);
		break;
	case 1:
	case 2:
		mtx_scale(&gpu->position_mtx, gpu->cmd_params);
		break;
	case 3:
		mtx_scale(&gpu->texture_mtx, gpu->cmd_params);
	}

	if (gpu->mtx_mode < 3) {
		update_clip_matrix(gpu);
	}
}

static void
cmd_mtx_trans(gpu_3d_engine *gpu)
{
	switch (gpu->mtx_mode) {
	case 0:
		mtx_trans(&gpu->projection_mtx, gpu->cmd_params);
		break;
	case 1:
		mtx_trans(&gpu->position_mtx, gpu->cmd_params);
		break;
	case 2:
		mtx_trans(&gpu->position_mtx, gpu->cmd_params);
		mtx_trans(&gpu->vector_mtx, gpu->cmd_params);
		break;
	case 3:
		mtx_trans(&gpu->texture_mtx, gpu->cmd_params);
	}

	if (gpu->mtx_mode < 3) {
		update_clip_matrix(gpu);
	}
}

static void
cmd_color(gpu_3d_engine *gpu)
{
	unpack_bgr555_3d(gpu->cmd_params[0], gpu->ge.vtx_color);
}

static void
texcoord_transform_1(gpu_3d_engine *gpu)
{
	auto& ge = gpu->ge;
	auto& m = gpu->texture_mtx;

	s64 s = ge.texcoord[0];
	s64 t = ge.texcoord[1];
	s64 r0 = (s * m.v[0][0] + t * m.v[1][0] + m.v[2][0] + m.v[3][0]) >> 12;
	s64 r1 = (s * m.v[0][1] + t * m.v[1][1] + m.v[2][1] + m.v[3][1]) >> 12;

	ge.vtx_texcoord[0] = r0;
	ge.vtx_texcoord[1] = r1;
}

static void
texcoord_transform_2(gpu_3d_engine *gpu, s64 nx, s64 ny, s64 nz)
{
	auto& ge = gpu->ge;
	auto& m = gpu->texture_mtx;

	s64 r0 = (nx * m.v[0][0] + ny * m.v[1][0] + nz * m.v[2][0]) >> 21;
	s64 r1 = (nx * m.v[0][1] + ny * m.v[1][1] + nz * m.v[2][1]) >> 21;

	ge.vtx_texcoord[0] = ge.texcoord[0] + r0;
	ge.vtx_texcoord[1] = ge.texcoord[1] + r1;
}

static void
cmd_normal(gpu_3d_engine *gpu)
{
	auto& ge = gpu->ge;

	u32 param = gpu->cmd_params[0];
	s16 nx = (s16)(param << 6) >> 6;
	s16 ny = (s16)(param >> 4) >> 6;
	s16 nz = (s16)(param >> 14) >> 6;

	if (gpu->ge.teximage_param >> 30 == 2) {
		texcoord_transform_2(gpu, nx, ny, nz);
	}

	s32 normal[3];
	mtx_mult_vec3(normal, &gpu->vector_mtx, nx, ny, nz);

	ge.vtx_color[0] = ge.emission_color[0];
	ge.vtx_color[1] = ge.emission_color[1];
	ge.vtx_color[2] = ge.emission_color[2];

	for (u32 i = 0; i < 4; i++) {
		if (!(ge.poly_attr & BIT(i)))
			continue;

		s64 diffuse_dot = (s64)-ge.light_vec[i][0] * normal[0] +
		                  (s64)-ge.light_vec[i][1] * normal[1] +
		                  (s64)-ge.light_vec[i][2] * normal[2];
		s64 diffuse_level =
				std::clamp(diffuse_dot, (s64)0, (s64)1 << 18);
		diffuse_level >>= 9;

		s64 shiny_dot = (s64)-ge.half_vec[i][0] * normal[0] +
		                (s64)-ge.half_vec[i][1] * normal[1] +
		                (s64)-ge.half_vec[i][2] * normal[2];
		s64 shiny_level = std::clamp(shiny_dot, (s64)0, (s64)1 << 18);
		shiny_level = (shiny_level >> 9) * (shiny_level >> 9) >> 9;

		for (u32 j = 0; j < 3; j++) {
			s64 light_color = ge.light_color[i][j];
			s64 specular = ge.specular_color[j] * light_color *
			               shiny_level;
			s64 diffuse = ge.diffuse_color[j] * light_color *
			              diffuse_level;
			s64 ambient = ge.ambient_color[j] * light_color;
			s64 color = ge.vtx_color[j] + (specular >> 15) +
			            (diffuse >> 15) + (ambient >> 6);
			color = std::clamp(color, (s64)0, (s64)63);
			ge.vtx_color[j] = color;
		}
	}
}

static void
cmd_texcoord(gpu_3d_engine *gpu)
{
	u32 param = gpu->cmd_params[0];
	gpu->ge.texcoord[0] = (s32)(param << 16) >> 16;
	gpu->ge.texcoord[1] = (s32)(param) >> 16;

	switch (gpu->ge.teximage_param >> 30) {
	case 0:
		gpu->ge.vtx_texcoord[0] = gpu->ge.texcoord[0];
		gpu->ge.vtx_texcoord[1] = gpu->ge.texcoord[1];
		break;
	case 1:
		texcoord_transform_1(gpu);
	}
}

static s64
get_face_direction(vertex **vertices)
{
	vertex *v0 = vertices[0];
	vertex *v1 = vertices[1];
	vertex *v2 = vertices[2];

	s32 ax = v1->x - v0->x;
	s32 ay = v1->y - v0->y;
	s32 az = v1->w - v0->w;
	s32 bx = v2->x - v0->x;
	s32 by = v2->y - v0->y;
	s32 bz = v2->w - v0->w;

	s64 cx = (s64)ay * bz - (s64)az * by;
	s64 cy = (s64)az * bx - (s64)ax * bz;
	s64 cz = (s64)ax * by - (s64)ay * bx;

	return cx * v0->x + cy * v0->y + cz * v0->w;
}

static void
add_polygon(gpu_3d_engine *gpu)
{
	auto& ge = gpu->ge;
	auto& pr = gpu->poly_ram[gpu->ge_buf];
	auto& vr = gpu->vtx_ram[gpu->ge_buf];

	if (pr.count >= 2048) {
		printf("polygon ram full\n");
		return;
	}

	u32 num_vertices = 0;
	vertex *vertices[4]{};
	vertex *last_strip_vtx_s[2];
	last_strip_vtx_s[0] = ge.last_strip_vtx[0];
	last_strip_vtx_s[1] = ge.last_strip_vtx[1];

	switch (ge.primitive_type) {
	case 0:
		num_vertices = 3;
		vertices[0] = &ge.vtx_buf[ge.vtx_count - 3 & 3];
		vertices[1] = &ge.vtx_buf[ge.vtx_count - 2 & 3];
		vertices[2] = &ge.vtx_buf[ge.vtx_count - 1 & 3];
		break;
	case 1:
		num_vertices = 4;
		vertices[0] = &ge.vtx_buf[ge.vtx_count - 4 & 3];
		vertices[1] = &ge.vtx_buf[ge.vtx_count - 3 & 3];
		vertices[2] = &ge.vtx_buf[ge.vtx_count - 2 & 3];
		vertices[3] = &ge.vtx_buf[ge.vtx_count - 1 & 3];
		break;
	case 2:
		num_vertices = 3;
		if (ge.last_strip_vtx[0]) {
			vertices[0] = ge.last_strip_vtx[0];
			vertices[0]->reused = 2;
		} else {
			vertices[0] = &ge.vtx_buf[ge.vtx_count - 3 & 3];
			vertices[0]->reused = 0;
		}
		vertices[0]->strip_vtx = 0;

		if (ge.last_strip_vtx[1]) {
			vertices[1] = ge.last_strip_vtx[1];
			vertices[1]->reused = 1;
		} else {
			vertices[1] = &ge.vtx_buf[ge.vtx_count - 2 & 3];
			vertices[1]->reused = 0;
		}
		vertices[1]->strip_vtx = 2;

		vertices[2] = &ge.vtx_buf[ge.vtx_count - 1 & 3];
		vertices[2]->reused = 0;
		vertices[2]->strip_vtx = 1;

		if (ge.vtx_count % 2 == 0) {
			std::swap(vertices[1], vertices[2]);
		}
		break;
	case 3:
		num_vertices = 4;
		if (ge.last_strip_vtx[0]) {
			vertices[0] = ge.last_strip_vtx[0];
			vertices[0]->reused = 2;
		} else {
			vertices[0] = &ge.vtx_buf[ge.vtx_count - 4 & 3];
			vertices[0]->reused = 0;
		}
		vertices[0]->strip_vtx = 0;

		if (ge.last_strip_vtx[1]) {
			vertices[1] = ge.last_strip_vtx[0];
			vertices[1]->reused = 1;
		} else {
			vertices[1] = &ge.vtx_buf[ge.vtx_count - 3 & 3];
			vertices[1]->reused = 0;
		}
		vertices[1]->strip_vtx = 0;

		vertices[2] = &ge.vtx_buf[ge.vtx_count - 1 & 3];
		vertices[2]->reused = 0;
		vertices[2]->strip_vtx = 1;
		vertices[3] = &ge.vtx_buf[ge.vtx_count - 2 & 3];
		vertices[3]->reused = 0;
		vertices[3]->strip_vtx = 2;
	}

	ge.last_strip_vtx[0] = nullptr;
	ge.last_strip_vtx[1] = nullptr;

	s64 face_dir = get_face_direction(vertices);
	if ((!(ge.poly_attr & BIT(6)) && face_dir < 0) ||
			(!(ge.poly_attr & BIT(7)) && face_dir > 0)) {
		return;
	}

	/* TODO: clipping */

	if (vr.count + num_vertices > 6144) {
		printf("vertex ram full\n");
		return;
	}

	polygon *poly = &pr.polygons[pr.count++];
	for (u32 i = 0; i < num_vertices; i++) {
		u32 reused = vertices[i]->reused;
		if (reused) {
			poly->vertices[i] = last_strip_vtx_s[reused & 1];
		} else {
			u32 idx = vr.count++;
			vr.vertices[idx] = *vertices[i];
			poly->vertices[i] = &vr.vertices[idx];
		}
		u32 strip_vtx = vertices[i]->strip_vtx;
		if (strip_vtx) {
			ge.last_strip_vtx[strip_vtx & 1] = poly->vertices[i];
		}
	}
	poly->num_vertices = num_vertices;

	int max_bits = 0;
	for (u32 i = 0; i < poly->num_vertices; i++) {
		vertex *v = poly->vertices[i];
		if (v->w >= 0) {
			max_bits = std::max(max_bits,
					1 + std::bit_width((u32)v->w));
		} else {
			max_bits = std::max(max_bits,
					1 + std::bit_width(~(u32)v->w));
		}

		if (v->w == 0) {
			v->sx = 0;
			v->sy = 0;
		} else {
			v->sx = (v->x + v->w) * gpu->viewport_w / (2 * v->w) +
			        gpu->viewport_x[0];
			v->sy = (-v->y + v->w) * gpu->viewport_h / (2 * v->w) +
			        gpu->viewport_y[0];
		}
	}

	poly->wshift = (max_bits - 16 + 4 - 1) & ~3;
	for (u32 i = 0; i < poly->num_vertices; i++) {
		vertex *v = poly->vertices[i];
		if (poly->wshift >= 0) {
			poly->normalized_w[i] = v->w >> poly->wshift;
		} else {
			poly->normalized_w[i] = v->w << -poly->wshift;
		}
	}

	poly->attr = ge.poly_attr;
	poly->backface = face_dir < 0;
}

static void
add_vertex(gpu_3d_engine *gpu)
{
	auto& ge = gpu->ge;
	auto& vr = gpu->vtx_ram[gpu->ge_buf];

	if (vr.count >= 6144) {
		printf("vertex ram full\n");
		return;
	}

	ge_vector vtx_point = { ge.vx, ge.vy, ge.vz, 1 << 12 };
	ge_vector result;
	mtx_mult_vec(&result, &gpu->clip_mtx, &vtx_point);

	vertex *v = &ge.vtx_buf[ge.vtx_count & 3];
	v->x = result.v[0];
	v->y = result.v[1];
	v->z = result.v[2];
	v->w = result.v[3];
	v->color = { ge.vtx_color[0], ge.vtx_color[1], ge.vtx_color[2] };
	ge.vtx_count++;

	switch (ge.primitive_type) {
	case 0:
		if (ge.vtx_count % 3 == 0) {
			add_polygon(gpu);
		}
		break;
	case 1:
		if (ge.vtx_count % 4 == 0) {
			add_polygon(gpu);
		}
		break;
	case 2:
		if (ge.vtx_count >= 3) {
			add_polygon(gpu);
		}
		break;
	case 3:
		if (ge.vtx_count >= 4 && ge.vtx_count % 2 == 0) {
			add_polygon(gpu);
		}
	}
}

static void
cmd_vtx_16(gpu_3d_engine *gpu)
{
	gpu->ge.vx = (s32)(gpu->cmd_params[0] << 16) >> 16;
	gpu->ge.vy = (s32)(gpu->cmd_params[0]) >> 16;
	gpu->ge.vz = (s32)(gpu->cmd_params[1] << 16) >> 16;
	add_vertex(gpu);
}

static void
cmd_vtx_10(gpu_3d_engine *gpu)
{
	gpu->ge.vx = (s32)(gpu->cmd_params[0] << 22) >> 22 << 6;
	gpu->ge.vy = (s32)(gpu->cmd_params[0] << 12) >> 22 << 6;
	gpu->ge.vz = (s32)(gpu->cmd_params[0] << 2) >> 22 << 6;
	add_vertex(gpu);
}

static void
cmd_vtx_xy(gpu_3d_engine *gpu)
{
	gpu->ge.vx = (s32)(gpu->cmd_params[0] << 16) >> 16;
	gpu->ge.vy = (s32)(gpu->cmd_params[0]) >> 16;
	add_vertex(gpu);
}

static void
cmd_vtx_xz(gpu_3d_engine *gpu)
{
	gpu->ge.vx = (s32)(gpu->cmd_params[0] << 16) >> 16;
	gpu->ge.vz = (s32)(gpu->cmd_params[0]) >> 16;
	add_vertex(gpu);
}

static void
cmd_vtx_yz(gpu_3d_engine *gpu)
{
	gpu->ge.vy = (s32)(gpu->cmd_params[0] << 16) >> 16;
	gpu->ge.vz = (s32)(gpu->cmd_params[0]) >> 16;
	add_vertex(gpu);
}

static void
cmd_vtx_diff(gpu_3d_engine *gpu)
{
	gpu->ge.vx += (s16)(gpu->cmd_params[0] << 6) >> 6;
	gpu->ge.vy += (s16)(gpu->cmd_params[0] >> 4) >> 6;
	gpu->ge.vz += (s16)(gpu->cmd_params[0] >> 14) >> 6;
	add_vertex(gpu);
}

static void
cmd_polygon_attr(gpu_3d_engine *gpu)
{
	gpu->ge.poly_attr_s = gpu->cmd_params[0];
}

static void
cmd_teximage_param(gpu_3d_engine *gpu)
{
	gpu->ge.teximage_param = gpu->cmd_params[0];
}

static void
cmd_pltt_base(gpu_3d_engine *gpu)
{
	gpu->ge.pltt_base = gpu->cmd_params[0] & 0x1FFF;
}

static void
cmd_dif_amb(gpu_3d_engine *gpu)
{
	u32 param = gpu->cmd_params[0];
	unpack_bgr555_3d(param, gpu->ge.diffuse_color);
	if (param & BIT(15)) {
		gpu->ge.vtx_color[0] = gpu->ge.diffuse_color[0];
		gpu->ge.vtx_color[1] = gpu->ge.diffuse_color[1];
		gpu->ge.vtx_color[2] = gpu->ge.diffuse_color[2];
	}
	unpack_bgr555_3d(param >> 16, gpu->ge.ambient_color);
}

static void
cmd_spe_emi(gpu_3d_engine *gpu)
{
	u32 param = gpu->cmd_params[0];
	unpack_bgr555_3d(param, gpu->ge.specular_color);
	unpack_bgr555_3d(param >> 16, gpu->ge.emission_color);
	gpu->ge.enable_shiny_table = param & BIT(15);
}

static void
cmd_light_vector(gpu_3d_engine *gpu)
{
	u32 param = gpu->cmd_params[0];
	s16 lx = (s16)(param << 6) >> 6;
	s16 ly = (s16)(param >> 4) >> 6;
	s16 lz = (s16)(param >> 14) >> 6;
	u32 l = param >> 30;

	mtx_mult_vec3(gpu->ge.light_vec[l], &gpu->vector_mtx, lx, ly, lz);
	gpu->ge.half_vec[l][0] = gpu->ge.light_vec[l][0] >> 1;
	gpu->ge.half_vec[l][1] = gpu->ge.light_vec[l][1] >> 1;
	gpu->ge.half_vec[l][2] = (gpu->ge.light_vec[l][2] - (1 << 9)) >> 1;
}

static void
cmd_light_color(gpu_3d_engine *gpu)
{
	u32 l = gpu->cmd_params[0] >> 30;
	unpack_bgr555_3d(gpu->cmd_params[0], gpu->ge.light_color[l]);
}

static void
cmd_shininess(gpu_3d_engine *gpu)
{
	for (u32 i = 0; i < 32; i++) {
		u32 param = gpu->cmd_params[i];
		for (u32 j = 0; j < 4; j++) {
			gpu->ge.shiny_table[4 * i + j] = param >> 8 * j & 0xFF;
		}
	}
}

static void
cmd_begin_vtxs(gpu_3d_engine *gpu)
{
	gpu->ge.poly_attr = gpu->ge.poly_attr_s;
	gpu->ge.primitive_type = gpu->cmd_params[0] & 3;
	gpu->ge.vtx_count = 0;
	gpu->ge.last_strip_vtx[0] = nullptr;
	gpu->ge.last_strip_vtx[1] = nullptr;
}

static void
cmd_swap_buffers(gpu_3d_engine *gpu)
{
	gpu->halted = true;
	gpu->ge.swap_bits_s = gpu->cmd_params[0] & 3;
}

static void
cmd_viewport(gpu_3d_engine *gpu)
{
	gpu->viewport_x[0] = gpu->cmd_params[0];
	gpu->viewport_y[0] = gpu->cmd_params[0] >> 8;
	gpu->viewport_x[1] = gpu->cmd_params[0] >> 16;
	gpu->viewport_y[1] = gpu->cmd_params[0] >> 24;
	gpu->viewport_w = gpu->viewport_x[1] - gpu->viewport_x[0] + 1;
	gpu->viewport_h = gpu->viewport_y[1] - gpu->viewport_y[0] + 1;
}

void
ge_execute_command(gpu_3d_engine *gpu, u8 command)
{
	switch (command) {
	case 0x00:
		break;
	case 0x10:
		cmd_mtx_mode(gpu);
		break;
	case 0x11:
		cmd_mtx_push(gpu);
		break;
	case 0x12:
		cmd_mtx_pop(gpu);
		break;
	case 0x13:
		cmd_mtx_store(gpu);
		break;
	case 0x14:
		cmd_mtx_restore(gpu);
		break;
	case 0x15:
		cmd_mtx_identity(gpu);
		break;
	case 0x16:
		cmd_mtx_load_4x4(gpu);
		break;
	case 0x17:
		cmd_mtx_load_4x3(gpu);
		break;
	case 0x18:
		cmd_mtx_mult_4x4(gpu);
		break;
	case 0x19:
		cmd_mtx_mult_4x3(gpu);
		break;
	case 0x1A:
		cmd_mtx_mult_3x3(gpu);
		break;
	case 0x1B:
		cmd_mtx_scale(gpu);
		break;
	case 0x1C:
		cmd_mtx_trans(gpu);
		break;
	case 0x20:
		cmd_color(gpu);
		break;
	case 0x21:
		cmd_normal(gpu);
		break;
	case 0x22:
		cmd_texcoord(gpu);
		break;
	case 0x23:
		cmd_vtx_16(gpu);
		break;
	case 0x24:
		cmd_vtx_10(gpu);
		break;
	case 0x25:
		cmd_vtx_xy(gpu);
		break;
	case 0x26:
		cmd_vtx_xz(gpu);
		break;
	case 0x27:
		cmd_vtx_yz(gpu);
		break;
	case 0x28:
		cmd_vtx_diff(gpu);
		break;
	case 0x29:
		cmd_polygon_attr(gpu);
		break;
	case 0x2A:
		cmd_teximage_param(gpu);
		break;
	case 0x2B:
		cmd_pltt_base(gpu);
		break;
	case 0x30:
		cmd_dif_amb(gpu);
		break;
	case 0x31:
		cmd_spe_emi(gpu);
		break;
	case 0x32:
		cmd_light_vector(gpu);
		break;
	case 0x33:
		cmd_light_color(gpu);
		break;
	case 0x34:
		cmd_shininess(gpu);
		break;
	case 0x41:
		break;
	case 0x40:
		cmd_begin_vtxs(gpu);
		break;
	case 0x50:
		cmd_swap_buffers(gpu);
		break;
	case 0x60:
		cmd_viewport(gpu);
		break;
	default:
		LOG("unhandled ge command %02X\n", command);
	}
}

} // namespace twice
