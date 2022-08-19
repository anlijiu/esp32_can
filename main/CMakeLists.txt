set(srcs "main.c"
    "blespp.c"
    "can_proto.c"
    "mcp2515.c"
    "spi_routine.c"
    "libwecan.c"
    )

idf_component_register(SRCS ${srcs}
                    INCLUDE_DIRS ".")
