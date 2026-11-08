# Host test: setting values between NMEA ($PLARS, $g) and the CAN bus,
# checked against the interface definitions in doc_larus (#278).
#
#   cmake -S sw_stm32/tests/nmea_settings -B build/nmea_settings_test
#   cmake --build build/nmea_settings_test
#   ctest --test-dir build/nmea_settings_test --output-on-failure
cmake_minimum_required(VERSION 3.16)
project(nmea_settings_test CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_EXTENSIONS ON) # gnu++14, as in the firmware build

get_filename_component(SW_STM32 "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)
set(LIB "${SW_STM32}/lib")

add_executable(nmea_settings_test
  nmea_settings_test.cpp
  "${SW_STM32}/Communication/NMEA_listener.cpp"
  "${LIB}/Output_Formatter/NMEA_format.cpp"
  "${LIB}/Output_Formatter/ascii_support.cpp")

target_include_directories(nmea_settings_test PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/stubs" # before the library's stubs
  "${LIB}/tests/stubs"
  "${LIB}/Generic_Algorithms"
  "${LIB}/NAV_Algorithms"
  "${LIB}/Output_Formatter"
  "${LIB}/Persistent_Data"
  "${LIB}/ARM_math")
target_compile_options(nmea_settings_test PRIVATE -Wall)
target_link_libraries(nmea_settings_test PRIVATE m)

enable_testing()
add_test(NAME nmea_to_can COMMAND nmea_settings_test nmea_to_can)
add_test(NAME can_to_nmea COMMAND nmea_settings_test can_to_nmea)
