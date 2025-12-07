cmake_minimum_required(VERSION 3.18)
project(excite_c_modules C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Find Basilisk
find_package(Basilisk REQUIRED)

add_subdirectory(excite_c_modules)
