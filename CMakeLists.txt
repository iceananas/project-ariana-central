cmake_minimum_required(VERSION 3.13.1)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(project-ariana-central)

FILE(GLOB app_sources src/*.c)
include_directories(include)
target_sources(app PRIVATE ${app_sources})

zephyr_library_include_directories(${ZEPHYR_BASE}/samples/bluetooth)
