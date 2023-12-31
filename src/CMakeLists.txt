add_executable(generate-lut
	nds/arm/interpreter/lut_generator.cc)

add_custom_command(
	OUTPUT arm_lut.cc thumb_lut.cc
	COMMAND generate-lut "${CMAKE_CURRENT_BINARY_DIR}/arm_lut.cc" "${CMAKE_CURRENT_BINARY_DIR}/thumb_lut.cc"
	DEPENDS generate-lut)

CHECK_INCLUDE_FILE_CXX(sys/mman.h HAVE_POSIX_MMAP)
if(HAVE_POSIX_MMAP)
	set(FILEMAP_SOURCE libtwice/filemap.cc)
else()
	set(FILEMAP_SOURCE libtwice/filemap_fstream.cc)
endif()

add_library(twice STATIC
	common/date.cc
	common/logger.cc
	common/profiler.cc
	libtwice/config.cc
	${FILEMAP_SOURCE}
	libtwice/game_db.cc
	libtwice/machine.cc
	nds/arm/arm.cc
	nds/arm/arm7.cc
	nds/arm/arm9.cc
	nds/arm/interpreter/lut.cc
	nds/cart/backup.cc
	nds/cart/cart.cc
	nds/cart/key.cc
	nds/dma.cc
	nds/firmware.cc
	nds/gpu/2d/gpu2d.cc
	nds/gpu/2d/render.cc
	nds/gpu/2d/render_bg.cc
	nds/gpu/2d/render_gfx.cc
	nds/gpu/2d/render_obj.cc
	nds/gpu/3d/ge.cc
	nds/gpu/3d/gpu3d.cc
	nds/gpu/3d/matrix.cc
	nds/gpu/3d/re.cc
	nds/gpu/vram.cc
	nds/ipc.cc
	nds/math.cc
	nds/mem/io.cc
	nds/mem/io7.cc
	nds/mem/io9.cc
	nds/nds.cc
	nds/powerman.cc
	nds/rtc.cc
	nds/scheduler.cc
	nds/sound.cc
	nds/spi.cc
	nds/timer.cc
	nds/touchscreen.cc
	nds/wifi.cc
	"${CMAKE_CURRENT_BINARY_DIR}/arm_lut.cc"
	"${CMAKE_CURRENT_BINARY_DIR}/thumb_lut.cc"
)

target_include_directories(twice
	PUBLIC ${PROJECT_SOURCE_DIR}/include)

target_include_directories(twice
	PRIVATE ${PROJECT_SOURCE_DIR}/src)
