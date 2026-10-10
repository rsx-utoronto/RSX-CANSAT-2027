idf_component_register(
  SRCS "../../../lib/mission/src/mission.cpp" "../../../lib/mission/src/flight_detection.cpp"
  INCLUDE_DIRS "../../../lib/mission/include")
target_compile_features(${COMPONENT_LIB} PUBLIC cxx_std_17)
