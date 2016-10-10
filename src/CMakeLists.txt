include_directories(${RTPiDrone_SOURCE_DIR}/include)
SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -pedantic -O3 -std=gnu11")
add_executable(RTPiDrone RTPiDrone.c main.c)
#target_link_libraries(RTPiDrone -lbcm2835 -lgsl -lgslcblas -lrt -lm -lpthread)
