cmake_minimum_required(VERSION 3.2)
project(LightingNode)

include(../cmake-xc16/MPLABX.cmake)

set(SOURCES
    src/main.c
    gen/lighting.c
)

set(HEADERS
    gen/lighting.h
)

set(NODE_FILES
    lighting.ncf
)

add_custom_command(OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/gen/lighting.h ${CMAKE_CURRENT_SOURCE_DIR}/gen/lighting.c
                   COMMAND LIN -t PIC${CMAKE_SYSTEM_PROCESSOR} -i uart1 ${NODE_FILES}
                   DEPENDS ${NODE_FILES}
                   WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

add_executable(LightingNode ${SOURCES} ${HEADERS} ${NODE_FILES})
target_compile_definitions(LightingNode PUBLIC -DFCY=2000000)
target_compile_options(LightingNode PUBLIC -std=gnu99)
target_include_directories(LightingNode PUBLIC include gen)
generate_mplabx_project(LightingNode)
