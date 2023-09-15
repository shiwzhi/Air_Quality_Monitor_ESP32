idf_component_register(SRCS "embedded-i2c-sgp40/sgp40_i2c.c"
                            "gas-index-algorithm/sensirion_gas_index_algorithm/sensirion_gas_index_algorithm.c"
                    INCLUDE_DIRS  "gas-index-algorithm/sensirion_gas_index_algorithm/"
                                  "embedded-i2c-sgp40"
                    PRIV_REQUIRES freertos driver main
                    )
