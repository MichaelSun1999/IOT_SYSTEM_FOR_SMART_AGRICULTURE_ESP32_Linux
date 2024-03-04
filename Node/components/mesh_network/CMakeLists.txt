idf_component_register(SRCS "mesh_netif.c"
			    "mqtt_app.c"
                       INCLUDE_DIRS "." "include"
                       PRIV_REQUIRES esp_wifi esp-tls mqtt)
