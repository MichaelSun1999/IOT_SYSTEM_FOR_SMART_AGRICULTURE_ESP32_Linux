idf_component_register(SRCS "mesh_netif.c"
                       INCLUDE_DIRS "." "include"
                       REQUIRES esp_wifi esp-tls mqtt lwip)
