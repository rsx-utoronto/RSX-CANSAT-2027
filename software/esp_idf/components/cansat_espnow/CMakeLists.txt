idf_component_register(
  SRCS "../../../lib/platform/esp_idf/src/esp_now_transport.cpp"
  INCLUDE_DIRS "../../../lib/platform/esp_idf/include"
  REQUIRES cansat_communications esp_wifi freertos
  REQUIRED_IDF_TARGETS esp32)
target_compile_features(${COMPONENT_LIB} PUBLIC cxx_std_17)
