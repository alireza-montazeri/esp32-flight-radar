idf_component_register(
  SRCS "lcd_touch_bsp.c"

  PRIV_REQUIRES
  esp_driver_gpio
  i2c_bsp
  main

  INCLUDE_DIRS "./")

##REQUIRES
