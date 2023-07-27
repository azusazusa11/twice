if (TOOLCHAIN_FILE_INCLUDED)
	return()
endif()
set(TOOLCHAIN_FILE_INCLUDED true)

set(CMAKE_CXX_FLAGS_INIT " \
-Wall -Wextra -pedantic \
-Wcast-align \
-Wsuggest-final-types \
-Wsuggest-final-methods \
-Wimplicit-fallthrough=5")

set(CMAKE_CXX_COMPILER g++)
set(CMAKE_EXE_LINKER_FLAGS_INIT "-fuse-ld=gold")
set(CMAKE_MODULE_LINKER_FLAGS_INIT "-fuse-ld=gold")
set(CMAKE_SHARED_LINKER_FLAGS_INIT "-fuse-ld=gold")
