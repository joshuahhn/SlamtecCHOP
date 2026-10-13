cmake_minimum_required(VERSION 3.22)
if(CMAKE_HOST_APPLE AND NOT DEFINED CMAKE_OSX_ARCHITECTURES)
  set(CMAKE_OSX_ARCHITECTURES arm64 CACHE STRING "Target TouchDesigner architecture")
endif()
project(SlamtecCHOP VERSION 0.5.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
if(WIN32 AND NOT CMAKE_SIZEOF_VOID_P EQUAL 8)
  message(FATAL_ERROR "TouchDesigner 2025.33230 requires a 64-bit build; configure with -A x64")
endif()

include(CTest)
find_package(Threads REQUIRED)
find_package(Git REQUIRED)
set(SDK_COMMIT 99478e5fb90de3b4a6db0080acacd373f8b36869)
set(SDK_SOURCE "${CMAKE_CURRENT_SOURCE_DIR}/third_party/rplidar_sdk")
execute_process(COMMAND "${GIT_EXECUTABLE}" -C "${SDK_SOURCE}" rev-parse HEAD
  OUTPUT_VARIABLE SDK_ACTUAL OUTPUT_STRIP_TRAILING_WHITESPACE RESULT_VARIABLE SDK_RESULT)
if(NOT SDK_RESULT EQUAL 0 OR NOT SDK_ACTUAL STREQUAL SDK_COMMIT)
  message(FATAL_ERROR "SDK source must be ${SDK_COMMIT}; run git submodule update --init")
endif()
execute_process(COMMAND "${GIT_EXECUTABLE}" -C "${SDK_SOURCE}" diff --quiet HEAD
  RESULT_VARIABLE SDK_DIRTY)
if(NOT SDK_DIRTY EQUAL 0)
  message(FATAL_ERROR "SDK source has local changes; keep the pinned submodule clean")
endif()

# Apply tracked safety changes to a build copy; never mutate the pinned submodule.
file(MAKE_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/sdk-source")
file(REAL_PATH "${CMAKE_CURRENT_BINARY_DIR}/sdk-source" SDK_COPY)
file(COPY "${SDK_SOURCE}/sdk" DESTINATION "${SDK_COPY}")
# Zero-context diff avoids embedding upstream whitespace violations. The exact
# clean SDK commit is verified above, so line-based application is reproducible.
execute_process(COMMAND "${GIT_EXECUTABLE}" apply --unidiff-zero --unsafe-paths "--directory=${SDK_COPY}"
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/sdk-safety.patch"
  WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" RESULT_VARIABLE PATCH_RESULT ERROR_VARIABLE PATCH_ERROR)
if(NOT PATCH_RESULT EQUAL 0)
  message(FATAL_ERROR "SDK safety patch did not apply: ${PATCH_ERROR}")
endif()
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS cmake/sdk-safety.patch)
set(SDK "${SDK_COPY}/sdk")
if(WIN32)
  set(SDK_ARCH win32)
elseif(APPLE)
  set(SDK_ARCH macOS)
else()
  set(SDK_ARCH linux)
endif()
add_library(rplidar_sdk STATIC
  "${SDK}/src/sl_lidar_driver.cpp" "${SDK}/src/hal/thread.cpp" "${SDK}/src/sl_crc.cpp"
  "${SDK}/src/sl_serial_channel.cpp" "${SDK}/src/sl_lidarprotocol_codec.cpp"
  "${SDK}/src/sl_async_transceiver.cpp" "${SDK}/src/sl_tcp_channel.cpp" "${SDK}/src/sl_udp_channel.cpp"
  "${SDK}/src/dataunpacker/dataunpacker.cpp"
  "${SDK}/src/dataunpacker/unpacker/handler_hqnode.cpp"
  "${SDK}/src/dataunpacker/unpacker/handler_capsules.cpp"
  "${SDK}/src/dataunpacker/unpacker/handler_normalnode.cpp"
  "${SDK}/src/arch/${SDK_ARCH}/net_serial.cpp"
  "${SDK}/src/arch/${SDK_ARCH}/net_socket.cpp" "${SDK}/src/arch/${SDK_ARCH}/timer.cpp")
target_include_directories(rplidar_sdk PUBLIC "${SDK}/include" PRIVATE "${SDK}/src")
target_link_libraries(rplidar_sdk PUBLIC Threads::Threads)
if(APPLE)
  target_compile_definitions(rplidar_sdk PRIVATE _MACOS)
endif()
if(WIN32)
  target_compile_definitions(rplidar_sdk PUBLIC WIN32 WIN64 NOMINMAX= _WIN32_WINNT=0x0A00 WINVER=0x0A00)
  target_link_libraries(rplidar_sdk PUBLIC ws2_32 winmm)
endif()

add_library(slamtec_receiver STATIC drvlogic/Receiver.cpp)
target_include_directories(slamtec_receiver PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(slamtec_receiver PUBLIC Threads::Threads)
add_library(slamtec_device STATIC drvlogic/RPLidarDevice.cpp drvlogic/NetworkChannel.cpp)
target_link_libraries(slamtec_device PUBLIC slamtec_receiver rplidar_sdk)

add_library(SlamtecCHOP MODULE SlamtecCHOP.cpp Parameters.cpp)
target_link_libraries(SlamtecCHOP PRIVATE slamtec_device)
set_target_properties(SlamtecCHOP PROPERTIES PREFIX "" OUTPUT_NAME SlamtecCHOP)
if(APPLE)
  set_target_properties(SlamtecCHOP PROPERTIES BUNDLE TRUE BUNDLE_EXTENSION plugin
    MACOSX_BUNDLE_INFO_PLIST "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Info.plist.in")
endif()

foreach(target slamtec_receiver slamtec_device SlamtecCHOP)
  if(MSVC)
    target_compile_options(${target} PRIVATE /W4 /WX /permissive-)
  else()
    target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic -Werror)
  endif()
endforeach()
set(SLAMTEC_SANITIZER "" CACHE STRING "Optional sanitizer: address or thread")
if(SLAMTEC_SANITIZER AND NOT MSVC)
  if(NOT SLAMTEC_SANITIZER MATCHES "^(address|thread)$")
    message(FATAL_ERROR "SLAMTEC_SANITIZER must be address or thread")
  endif()
  foreach(target rplidar_sdk slamtec_receiver slamtec_device SlamtecCHOP)
    target_compile_options(${target} PUBLIC "-fsanitize=${SLAMTEC_SANITIZER}" -fno-omit-frame-pointer)
    target_link_options(${target} PUBLIC "-fsanitize=${SLAMTEC_SANITIZER}")
  endforeach()
endif()
if(BUILD_TESTING)
  add_executable(receiver_tests tests/receiver_tests.cpp)
  target_link_libraries(receiver_tests PRIVATE slamtec_device)
  if(MSVC)
    target_compile_options(receiver_tests PRIVATE /W4 /WX /permissive-)
  else()
    target_compile_options(receiver_tests PRIVATE -Wall -Wextra -Wpedantic -Werror)
  endif()
  add_test(NAME receiver_tests COMMAND receiver_tests)
  set_tests_properties(receiver_tests PROPERTIES TIMEOUT 60)
endif()
install(TARGETS SlamtecCHOP LIBRARY DESTINATION . RUNTIME DESTINATION .)
install(FILES "${SDK_SOURCE}/LICENSE" DESTINATION licenses/rplidar_sdk)
install(FILES README.md DESTINATION .)
