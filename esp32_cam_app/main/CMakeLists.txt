set(srcs 
  app_camera.c
  app_wifi.c
  app_httpd.c
  app_main.c
)

idf_component_register(SRCS ${srcs}
                    INCLUDE_DIRS "include"
                    EMBED_FILES "www/index.html.gz")

target_compile_options(${COMPONENT_LIB} PRIVATE
  -Wall
  -Wextra
  -Werror=double-promotion
)
