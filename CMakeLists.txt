cmake_minimum_required(VERSION 3.6)
project(pc-arWet1)

set(CMAKE_C_STANDARD 99)

#set(GCC_COVERAGE_COMPILE_FLAGS "-Wall -pedantic-errors -Werror")
set( CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}" )


# ---- HDU TEST ----
set(SOURCE_FILES HDU_test.c sim_mem.c sim_core.c sim_api.h )


#set(SOURCE_FILES sim_api.h sim_core.c sim_mem.c sim_main.c )

#set(SOURCE_FILES tests/list_mtm/list_mtm1.h tests/list_mtm/list_example_test.c
#        tests/list_mtm/test_utilities.h tests/list_mtm/list_mtm1.c)

#link_directories(libmtm/win64)

add_executable(pc-arWet1 ${SOURCE_FILES})

