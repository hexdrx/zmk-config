if(CONFIG_PER_KEY_REACTIVE_RGB)
  zephyr_library()
  zephyr_library_sources(src/per_key_rgb.c)
endif()
