idf_component_register(SRCS "mcc5983ma.c"
                       INCLUDE_DIRS "." "include"
                       REQUIRES i2c_bus 
                       PRIV_REQUIRES core utils)