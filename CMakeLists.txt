#
# nRF54L15 Tag Demo
#
cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(nRF54L15TagDemo)

# main.c lives at the top level; each functional area is a self-contained
# module under src/ with its own CMakeLists.txt and Kconfig.
target_sources(app PRIVATE src/main.c)

add_subdirectory(src/sensors)
add_subdirectory(src/ble)
add_subdirectory(src/cs)
add_subdirectory(src/ess)
add_subdirectory(src/motion)
add_subdirectory(src/ux)
add_subdirectory(src/control)
