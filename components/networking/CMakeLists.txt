idf_component_register(SRCS "networking.c" "cube_wifi.c" "cube_ethernet.c"
                  INCLUDE_DIRS "./include"
                  REQUIRES gamecube_controller
                  PRIV_REQUIRES nvs_flash)
