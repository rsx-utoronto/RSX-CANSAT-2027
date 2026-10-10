idf_component_register(
  SRCS "../../../lib/platform/esp_idf/src/register_bus.cpp"
  INCLUDE_DIRS "../../../lib/platform/esp_idf/include"
  REQUIRES cansat_core esp_driver_i2c)
target_compile_features(${COMPONENT_LIB} PUBLIC cxx_std_17)
