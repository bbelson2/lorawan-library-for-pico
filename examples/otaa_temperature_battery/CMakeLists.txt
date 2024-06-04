cmake_minimum_required(VERSION 3.12)

set(projname "pico_lorawan_otaa_temperature_battery")

# rest of your project
add_executable(${projname}
    main.c
)

target_link_libraries(${projname} pico_lorawan hardware_adc)

# enable usb output, disable uart output
pico_enable_stdio_usb(${projname} 1)
pico_enable_stdio_uart(${projname} 0)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(${projname})
