# Shared build profile, not a detected board or a flight-ready pin/memory map.
if(DEFINED IDF_TARGET AND NOT "${IDF_TARGET}" STREQUAL "esp32")
  message(FATAL_ERROR "DevKitC V4 skeletons require IDF_TARGET=esp32; alternate SoCs need a reviewed profile.")
endif()
if(DEFINED ENV{IDF_TARGET} AND NOT "$ENV{IDF_TARGET}" STREQUAL "" AND
   NOT "$ENV{IDF_TARGET}" STREQUAL "esp32")
  message(FATAL_ERROR "DevKitC V4 skeletons require IDF_TARGET=esp32; unset the conflicting environment target.")
endif()
set(IDF_TARGET "esp32" CACHE STRING "DevKitC V4 skeleton SoC")

if(NOT DEFINED ENV{IDF_PATH} OR "$ENV{IDF_PATH}" STREQUAL "")
  message(FATAL_ERROR "ESP-IDF is not activated. Source the chosen SDK's export.sh, then run idf.py build. The repository root is the separate host-test build.")
endif()
if(NOT EXISTS "$ENV{IDF_PATH}/tools/cmake/project.cmake")
  message(FATAL_ERROR "IDF_PATH does not contain tools/cmake/project.cmake; activate a complete ESP-IDF SDK.")
endif()

# Direct SDK invocation checks the release version. The build-only Python runner
# additionally checks the exact commit, clean submodules and tool manifest hash.
file(READ "${CMAKE_CURRENT_LIST_DIR}/../sdk.lock.json" CANSAT_SDK_LOCK)
string(JSON CANSAT_IDF_VERSION GET "${CANSAT_SDK_LOCK}" version)
include("$ENV{IDF_PATH}/tools/cmake/version.cmake")
set(CANSAT_ACTUAL_IDF_VERSION "${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}.${IDF_VERSION_PATCH}")
if(NOT CANSAT_ACTUAL_IDF_VERSION STREQUAL CANSAT_IDF_VERSION)
  message(FATAL_ERROR "Pinned ESP-IDF version ${CANSAT_IDF_VERSION} required; found ${CANSAT_ACTUAL_IDF_VERSION}.")
endif()

get_filename_component(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../components" ABSOLUTE)
# Compile main and its declared dependency closure, not unrelated components.
set(COMPONENTS main)
