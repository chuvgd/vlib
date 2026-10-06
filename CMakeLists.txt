cmake_minimum_required(VERSION 3.16)
project(vlib VERSION 0.1.0 LANGUAGES CXX)

# C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 创建库（STATIC 静态库 / SHARED 动态库，不写则由 BUILD_SHARED_LIBS 控制）
add_library(vlib STATIC
)

# 给目标起一个别名，方便本仓库内部其他 target 使用
# add_library(vlib ALIAS vlib)

target_include_directories(vlib
	PUBLIC
		${CMAKE_CURRENT_SOURCE_DIR}/algorithm
		${CMAKE_CURRENT_SOURCE_DIR}/device
		${CMAKE_CURRENT_SOURCE_DIR}/module
		${CMAKE_CURRENT_SOURCE_DIR}/utils
)

# 头文件目录
target_include_directories(vlib
	PUBLIC
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		$<INSTALL_INTERFACE:include>
)

# 只对库自身生效的编译选项
target_compile_features(vlib PUBLIC cxx_std_17)

# 设置版本号（动态库会生成 soname 符号链接）
set_target_properties(vlib
	PROPERTIES
		VERSION ${PROJECT_VERSION}
		SOVERSION ${PROJECT_VERSION_MAJOR}
)

# ----------------------------- 安装规则 -------------------------------

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

install(
	TARGETS vlib
	EXPORT vlibTargets
	RUNTIME
		DESTINATION
		${CMAKE_INSTALL_BINDIR} # .dll / .exe
	LIBRARY
		DESTINATION
		${CMAKE_INSTALL_LIBDIR} # .so / .a
	ARCHIVE
		DESTINATION
		${CMAKE_INSTALL_LIBDIR} # .lib / .a
	INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# 导出 targets
install(EXPORT vlibTargets
	FILE vlibTargets.cmake
	NAMESPACE vlib::
	DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/vlib
)

# 生成并安装 Config 文件
configure_package_config_file(
	${CMAKE_CURRENT_SOURCE_DIR}/cmake/vlibConfig.cmake.in
	${CMAKE_CURRENT_BINARY_DIR}/vlibConfig.cmake
	INSTALL_DESTINATION
	${CMAKE_INSTALL_LIBDIR}/cmake/vlib
)
write_basic_package_version_file(
	${CMAKE_CURRENT_BINARY_DIR}/vlibConfigVersion.cmake
	VERSION
	${PROJECT_VERSION}
	COMPATIBILITY
	SameMajorVersion
)
install(
	FILES
		${CMAKE_CURRENT_BINARY_DIR}/vlibConfig.cmake
		${CMAKE_CURRENT_BINARY_DIR}/vlibConfigVersion.cmake
	DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/vlib
)
