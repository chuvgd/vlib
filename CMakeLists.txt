# 支持库：平台无关，以头文件为主，只有少数 .c/.cpp 要编译
file(GLOB VLIB_SOURCES algorithm/*.c algorithm/*.cpp utils/*.c utils/*.cpp)

add_library(vlib STATIC ${VLIB_SOURCES})

target_include_directories(vlib PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}      # 库内： #include "algorithm/arm-matrix.hpp"
  ${CMAKE_CURRENT_SOURCE_DIR}/..)  # 上层： #include "vlib/algorithm/pid.hpp"

target_compile_features(vlib PUBLIC cxx_std_17)
