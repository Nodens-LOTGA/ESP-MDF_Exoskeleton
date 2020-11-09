
# idf_component_register(SRCS "cam.c" "router_example.c" "./router_example.c"
#                     INCLUDE_DIRS ""
#                     REQUIRES mcommon mconfig mwifi mlink
# )


FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/main/*.*)

idf_component_register(SRCS ${app_sources})