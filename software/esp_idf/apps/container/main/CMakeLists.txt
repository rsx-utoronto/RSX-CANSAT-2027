idf_component_register(
  SRCS "app_main.cpp"
  PRIV_REQUIRES cansat_app_boot cansat_mission cansat_drivers cansat_esp_idf cansat_communications cansat_espnow
  REQUIRED_IDF_TARGETS esp32)
target_compile_features(${COMPONENT_LIB} PRIVATE cxx_std_17)
