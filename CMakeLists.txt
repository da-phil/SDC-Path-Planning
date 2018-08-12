project(Path_Planning)
cmake_minimum_required (VERSION 3.5)

find_package(Eigen3)

include_directories(
        ${EIGEN3_INCLUDE_DIR}
	src/
)

add_definitions(-std=c++11)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CXX_FLAGS "-Wall")

set(sources
	src/main.cpp
	src/tools.cpp
	src/trackmap.cpp
	src/trajectoryGenerator.cpp
	src/polynomials.cpp
	src/vehicle.cpp
)


if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 
  include_directories(/usr/local/include)
  include_directories(/usr/local/opt/openssl/include)
  link_directories(/usr/local/lib)
  link_directories(/usr/local/opt/openssl/lib)
  link_directories(/usr/local/Cellar/libuv/1.11.0/lib)
endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 


add_executable(path_planning ${sources})

target_link_libraries(path_planning z ssl uv uWS)
