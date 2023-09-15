idf_component_register(SRCS "BMP2-Sensor-API/bmp2.c"
                    INCLUDE_DIRS "./" "BMP2-Sensor-API"
                    PRIV_REQUIRES freertos driver main
                    )