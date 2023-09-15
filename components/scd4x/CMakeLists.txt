idf_component_register(SRCS 
                        "embedded-i2c-scd4x/scd4x_i2c.c"
                        "embedded-i2c-scd4x/sensirion_common.c"
                        "embedded-i2c-scd4x/sensirion_i2c.c"
                    INCLUDE_DIRS "" "embedded-i2c-scd4x"
                    PRIV_REQUIRES freertos driver main
                    )
