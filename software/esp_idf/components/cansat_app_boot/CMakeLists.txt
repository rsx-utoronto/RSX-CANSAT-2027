idf_component_register(
  SRCS "app_boot.cpp"
  INCLUDE_DIRS "include"
  PRIV_REQUIRES log
  REQUIRED_IDF_TARGETS esp32)
target_compile_features(${COMPONENT_LIB} PUBLIC cxx_std_17)
