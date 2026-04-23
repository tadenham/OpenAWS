file(GLOB_RECURSE srcs "main.c" "src/*.c")

idf_component_register(SRCS "${srcs}"
                    INCLUDE_DIRS "./include")

set(ulp_app_name ulp_${COMPONENT_NAME})
set(ulp_s_sources "ulp/pulse_count.S")
set(ulp_exp_dep_srcs "station.c")
ulp_embed_binary(${ulp_app_name} "${ulp_s_sources}" "${ulp_exp_dep_srcs}")
set(EXTRA_COMPONENT_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/ulp)
