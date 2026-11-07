# Host test: runs the real GNSS_driver.cpp and GNSS.cpp against recorded
# receiver output, with HAL, DMA and FreeRTOS simulated (see sim.cpp).
#   cmake -S sw_stm32/tests/gnss_driver -B build/gnss_driver_test
#   cmake --build build/gnss_driver_test && ctest --test-dir build/gnss_driver_test --output-on-failure
cmake_minimum_required(VERSION 3.16)
project(gnss_driver_test CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(SW_STM32 ${CMAKE_CURRENT_SOURCE_DIR}/../..)
# another driver version can be tested by pointing this to its source
set(GNSS_DRIVER_SOURCE ${SW_STM32}/Communication/GNSS_driver.cpp CACHE FILEPATH "GNSS driver under test")

add_executable(gnss_driver_test
  gnss_driver_test.cpp
  sim.cpp
  ${GNSS_DRIVER_SOURCE}
  ${SW_STM32}/Drivers/Custom/GNSS.cpp)
# message parser, if the driver uses one
if(EXISTS ${SW_STM32}/Drivers/Custom/UBX_receiver.cpp)
  target_sources(gnss_driver_test PRIVATE ${SW_STM32}/Drivers/Custom/UBX_receiver.cpp)
endif()

# the stubs come first, so they replace the firmware's own headers
target_include_directories(gnss_driver_test PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/stubs
  ${SW_STM32}/Communication
  ${SW_STM32}/Drivers/Custom
  ${SW_STM32}/lib/NAV_Algorithms
  ${SW_STM32}/lib/Generic_Algorithms
  ${SW_STM32}/lib/Output_Formatter
  ${SW_STM32}/lib/Persistent_Data)

target_compile_options(gnss_driver_test PRIVATE -Wall -Wno-unused-but-set-variable)

enable_testing()
add_test(NAME gnss_driver_x20d
  COMMAND gnss_driver_test ${CMAKE_CURRENT_SOURCE_DIR}/data/x20d_hdg200_pvt-daheading_60s.bin)
# the same recording with every NAV-DAHEADING 25 ms after its NAV-PVT
add_test(NAME gnss_driver_x20d_late_heading
  COMMAND gnss_driver_test ${CMAKE_CURRENT_SOURCE_DIR}/data/x20d_hdg200_pvt-daheading_60s.bin --heading-delay-ms 25)
