# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)
set(EXTRA_COMPONENT_DIRS /home/szm/esp-rainmaker/components/esp-insights/components /home/szm/esp-rainmaker/components /home/szm/esp-rainmaker/examples/common)
#set(COMPONENTS main) # "Trim" the build. Include the minimal set of components; main and anything it depends on. 
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
add_compile_options("-Wno-attributes") # For LVGL code
project(ip_internal_network)
