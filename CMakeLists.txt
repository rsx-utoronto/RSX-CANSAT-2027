cmake_minimum_required(VERSION 3.20)
project(rsx_cansat_native LANGUAGES CXX)

# Host-only build. No board, GPIO, radio channel or flight configuration is implied.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(cansat_core INTERFACE)
target_include_directories(cansat_core INTERFACE software/lib/core/include)
add_library(cansat_drivers software/lib/drivers/src/ina236.cpp)
target_include_directories(cansat_drivers PUBLIC software/lib/drivers/include)
target_link_libraries(cansat_drivers PUBLIC cansat_core)
add_library(cansat_communications software/lib/communications/src/serial_frame.cpp
  software/lib/communications/src/radio_protocol.cpp
  software/lib/communications/src/node_com.cpp)
target_sources(cansat_communications PRIVATE software/lib/communications/src/mission_payloads.cpp)
target_include_directories(cansat_communications PUBLIC software/lib/communications/include)
add_library(cansat_mission software/lib/mission/src/mission.cpp
  software/lib/mission/src/flight_detection.cpp)
target_include_directories(cansat_mission PUBLIC software/lib/mission/include)

foreach(target cansat_drivers cansat_communications cansat_mission)
  if(MSVC)
    target_compile_options(${target} PRIVATE /W4 /WX)
  else()
    target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic -Werror)
  endif()
endforeach()

add_executable(sensor_diagnostics tools/sensor_diagnostics.cpp)
target_link_libraries(sensor_diagnostics PRIVATE cansat_drivers)
add_executable(flight_replay_backend tools/flight_replay_backend.cpp)
target_link_libraries(flight_replay_backend PRIVATE cansat_mission)
if(NOT MSVC)
  target_compile_options(flight_replay_backend PRIVATE -Wall -Wextra -Wpedantic -Werror)
endif()

include(CTest)
if(BUILD_TESTING)
  find_package(Python3 3.10 REQUIRED COMPONENTS Interpreter)
  add_executable(core_tests software/tests/native/test_core.cpp)
  target_link_libraries(core_tests PRIVATE cansat_drivers cansat_communications)
  add_test(NAME native_core COMMAND core_tests)
  add_executable(node_com_tests software/tests/native/test_node_com.cpp)
  target_link_libraries(node_com_tests PRIVATE cansat_communications)
  if(NOT MSVC)
    target_compile_options(node_com_tests PRIVATE -Wall -Wextra -Wpedantic -Werror)
  endif()
  add_test(NAME node_communications COMMAND node_com_tests)
  add_executable(mission_payload_tests software/tests/native/test_mission_payloads.cpp)
  target_link_libraries(mission_payload_tests PRIVATE cansat_communications)
  if(NOT MSVC)
    target_compile_options(mission_payload_tests PRIVATE -Wall -Wextra -Wpedantic -Werror)
  endif()
  add_test(NAME mission_payload_schemas COMMAND mission_payload_tests)
  add_test(NAME mission_payload_golden
    COMMAND ${Python3_EXECUTABLE} -B ${CMAKE_CURRENT_SOURCE_DIR}/software/tests/native/test_payload_golden.py
      ${CMAKE_CURRENT_SOURCE_DIR} $<TARGET_FILE:mission_payload_tests>)
  add_test(NAME radio_wire_golden
    COMMAND ${Python3_EXECUTABLE} -B ${CMAKE_CURRENT_SOURCE_DIR}/software/tests/native/test_radio_golden.py
      ${CMAKE_CURRENT_SOURCE_DIR}/software/lib/communications/protocol.json $<TARGET_FILE:node_com_tests>)
  add_executable(esp_now_transport_tests software/tests/native/test_esp_now_transport.cpp
    software/lib/platform/esp_idf/src/esp_now_transport.cpp)
  target_include_directories(esp_now_transport_tests PRIVATE
    software/tests/native/radio_stubs software/lib/platform/esp_idf/include)
  target_link_libraries(esp_now_transport_tests PRIVATE cansat_communications)
  if(NOT MSVC)
    target_compile_options(esp_now_transport_tests PRIVATE -Wall -Wextra -Wpedantic -Werror)
  endif()
  add_test(NAME esp_now_adapter_contract COMMAND esp_now_transport_tests)
  add_executable(frame_cli software/tests/native/frame_cli.cpp)
  target_link_libraries(frame_cli PRIVATE cansat_communications)
  add_test(NAME python_cpp_protocol
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/ground_station/gui-new/tests/protocol/test_serial_frame.py
      $<TARGET_FILE:frame_cli>)
  set_tests_properties(python_cpp_protocol PROPERTIES ENVIRONMENT PYTHONDONTWRITEBYTECODE=1)
  add_test(NAME fixture_diagnostic COMMAND sensor_diagnostics --fixture)
  set_tests_properties(fixture_diagnostic PROPERTIES PASS_REGULAR_EXPRESSION "source=fixture voltage_v=12.0000 current_a=0.100000")
  add_test(NAME diagnostic_requires_explicit_fixture COMMAND sensor_diagnostics)
  set_tests_properties(diagnostic_requires_explicit_fixture PROPERTIES WILL_FAIL TRUE)

  # Compile production adapter against a private SDK test double, never expose
  # this include directory to production components or the portable libraries.
  add_executable(esp_idf_bus_tests software/tests/native/test_esp_idf_bus.cpp
    software/lib/platform/esp_idf/src/register_bus.cpp)
  target_include_directories(esp_idf_bus_tests PRIVATE
    software/tests/native/esp_idf_stubs software/lib/platform/esp_idf/include)
  target_link_libraries(esp_idf_bus_tests PRIVATE cansat_drivers)
  if(NOT MSVC)
    target_compile_options(esp_idf_bus_tests PRIVATE -Wall -Wextra -Wpedantic -Werror)
  endif()
  add_test(NAME esp_idf_bus_contract COMMAND esp_idf_bus_tests)
  add_executable(mission_tests software/tests/native/test_mission.cpp)
  target_link_libraries(mission_tests PRIVATE cansat_mission)
  add_test(NAME deterministic_mission COMMAND mission_tests)
  add_executable(flight_detection_tests software/tests/native/test_flight_detection.cpp)
  target_link_libraries(flight_detection_tests PRIVATE cansat_mission)
  if(NOT MSVC)
    target_compile_options(flight_detection_tests PRIVATE -Wall -Wextra -Wpedantic -Werror)
  endif()
  add_test(NAME altitude_flight_detection COMMAND flight_detection_tests)
  add_test(NAME csv_flight_replay
    COMMAND ${Python3_EXECUTABLE} -B ${CMAKE_CURRENT_SOURCE_DIR}/software/tests/replay/test_flight_replay.py
      $<TARGET_FILE:flight_replay_backend>)
  set_tests_properties(csv_flight_replay PROPERTIES ENVIRONMENT PYTHONDONTWRITEBYTECODE=1)
  add_test(NAME preferred_module_policy
    COMMAND ${Python3_EXECUTABLE} -B ${CMAKE_CURRENT_SOURCE_DIR}/software/tests/native/test_board_policy.py
      ${CMAKE_CURRENT_SOURCE_DIR})

  # Execute each real firmware entry point on the host with logging substituted.
  # No SDK, RTOS, peripherals, or toolchain compatibility is simulated here.
  foreach(role container pocketqube ground_radio)
    add_executable(${role}_app_smoke
      software/tests/native/test_app_boot.cpp
      software/esp_idf/apps/${role}/main/app_main.cpp
      software/esp_idf/components/cansat_app_boot/app_boot.cpp)
    target_include_directories(${role}_app_smoke PRIVATE
      software/tests/native/app_stubs
      software/esp_idf/components/cansat_app_boot/include)
    target_compile_definitions(${role}_app_smoke PRIVATE CANSAT_EXPECTED_ROLE="${role}")
    if(NOT MSVC)
      target_compile_options(${role}_app_smoke PRIVATE -Wall -Wextra -Wpedantic -Werror)
    endif()
    add_test(NAME ${role}_inactive_startup COMMAND ${role}_app_smoke)
  endforeach()
  add_test(NAME esp_idf_app_project_contract
    COMMAND ${Python3_EXECUTABLE} -B ${CMAKE_CURRENT_SOURCE_DIR}/software/tests/native/test_app_projects.py
      ${CMAKE_CURRENT_SOURCE_DIR})
  add_test(NAME esp_idf_build_only_runner
    COMMAND ${Python3_EXECUTABLE} -B ${CMAKE_CURRENT_SOURCE_DIR}/software/tests/native/test_idf_build_runner.py)
endif()
