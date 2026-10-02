target_include_directories(app PRIVATE ${CMAKE_CURRENT_LIST_DIR})

if(CONFIG_BOARD_NATIVE_SIM)
  target_sources(native_simulator INTERFACE ${CMAKE_CURRENT_LIST_DIR}/benchmark_clock.c)
endif()
