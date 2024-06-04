cmake_minimum_required(VERSION 3.12)

set(projname "pico_lorawan_otaa_temperature_battery_count")

# rest of your project
add_executable(${projname} main.c)
target_compile_definitions(${projname} PUBLIC LORAWAN_DEVICE_EUI=\"70B3D57ED006081A\" LORAWAN_APP_KEY=\"3AC7A1DDF11C11BCCDEFB05D42693AB1\")

target_link_libraries(${projname} pico_lorawan hardware_adc)

# enable usb output, disable uart output
pico_enable_stdio_usb(${projname} 1)
pico_enable_stdio_uart(${projname} 0)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(${projname})

add_executable(${projname}_1 main.c)
target_compile_definitions(${projname}_1 PUBLIC REPORT_TO_STDIO=1 MUST_INIT_USB=0 LORAWAN_DEVICE_EUI=\"70B3D57ED006081A\" LORAWAN_APP_KEY=\"3AC7A1DDF11C11BCCDEFB05D42693AB1\")

target_link_libraries(${projname}_1 pico_lorawan hardware_adc)

# enable usb output, disable uart output
pico_enable_stdio_usb(${projname}_1 1)
pico_enable_stdio_uart(${projname}_1 0)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(${projname}_1)

add_executable(${projname}_2 main.c)
target_compile_definitions(${projname}_2 PUBLIC REPORT_TO_STDIO=1 MUST_INIT_USB=0 LORAWAN_DEVICE_EUI=\"70B3D57ED0060826\" LORAWAN_APP_KEY=\"509CBA67E82DA2374136092489A68CC7\")

target_link_libraries(${projname}_2 pico_lorawan hardware_adc)

# enable usb output, disable uart output
pico_enable_stdio_usb(${projname}_2 1)
pico_enable_stdio_uart(${projname}_2 0)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(${projname}_2)
