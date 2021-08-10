idf_component_register(SRCS "networking.c"
                  INCLUDE_DIRS "./include"
                  REQUIRES gamecube_controller
                  PRIV_REQUIRES nvs_flash)
