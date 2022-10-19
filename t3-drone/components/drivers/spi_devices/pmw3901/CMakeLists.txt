idf_component_register(SRCS "flowdeck_v1v2.c" "pmw3901.c"
                       INCLUDE_DIRS "include"
                       REQUIRES spi_bus core
                        PRIV_REQUIRES config utils)