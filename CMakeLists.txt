file(GLOB VLIB_SOURCES
	CONFIGURE_DEPENDS
	algorithm/*.c
	algorithm/*.cpp
	utils/*.c
	utils/*.cpp
)

add_library(vlib STATIC ${VLIB_SOURCES})

target_include_directories(vlib
	PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/..
)

target_compile_features(vlib PUBLIC cxx_std_17)
