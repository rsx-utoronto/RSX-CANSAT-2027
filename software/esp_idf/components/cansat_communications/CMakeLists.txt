idf_component_register(
  SRCS "../../../lib/communications/src/serial_frame.cpp"
       "../../../lib/communications/src/radio_protocol.cpp"
       "../../../lib/communications/src/node_com.cpp"
       "../../../lib/communications/src/mission_payloads.cpp"
  INCLUDE_DIRS "../../../lib/communications/include")
target_compile_features(${COMPONENT_LIB} PUBLIC cxx_std_17)
