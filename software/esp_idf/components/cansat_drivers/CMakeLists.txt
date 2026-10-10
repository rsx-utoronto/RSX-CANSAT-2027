idf_component_register(
  SRCS "../../../lib/drivers/src/ina236.cpp"
  INCLUDE_DIRS "../../../lib/drivers/include"
  REQUIRES cansat_core)
target_compile_features(${COMPONENT_LIB} PUBLIC cxx_std_17)
