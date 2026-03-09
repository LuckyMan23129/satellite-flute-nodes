cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(flute_iridium_lidar)

include_directories("include/")

target_include_directories(app PRIVATE src)

target_sources(app PRIVATE 
                        src/main.c src/iridium_uart.c src/astar_diurnal.c src/switch_opencircuit.c 
                        src/read_Vpv.c src/read_Vcap.c src/read_Vsupply.c 
                        src/switch_Vpv_divider.c src/switch_peripheral_power.c
                        src/i2c_sensor4.c src/lowpower.c 
                        )
