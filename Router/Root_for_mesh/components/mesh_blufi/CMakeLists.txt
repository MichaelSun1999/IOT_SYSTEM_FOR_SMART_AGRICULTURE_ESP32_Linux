idf_component_register(SRCS "blufi_security.c"
                            "blufi_init.c"
                    INCLUDE_DIRS "."
                    REQUIRES bt mbedtls nvs_flash lwip)
