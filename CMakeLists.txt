# Link101 board support. Add this to any Pico project to get the board's
# peripherals; include only the headers for the ones you use.
add_library(hardware_link101 STATIC
    ${CMAKE_CURRENT_LIST_DIR}/src/pio_serial.c
    ${CMAKE_CURRENT_LIST_DIR}/src/uart_serial.c
    ${CMAKE_CURRENT_LIST_DIR}/src/neopixel.c
    ${CMAKE_CURRENT_LIST_DIR}/src/can.c
)

target_include_directories(hardware_link101 PUBLIC ${CMAKE_CURRENT_LIST_DIR}/include)

target_link_libraries(hardware_link101 PUBLIC
    pico_serial
    pico_stdlib
    hardware_pio
    hardware_clocks
    hardware_uart
    hardware_irq
    hardware_spi
)

# Generated .pio headers land on this target's include path, so they must be
# generated here rather than by whoever links us.
pico_generate_pio_header(hardware_link101 ${CMAKE_CURRENT_LIST_DIR}/src/pio/uart_tx.pio)
pico_generate_pio_header(hardware_link101 ${CMAKE_CURRENT_LIST_DIR}/src/pio/uart_rx.pio)
pico_generate_pio_header(hardware_link101 ${CMAKE_CURRENT_LIST_DIR}/src/pio/ws2812.pio)

target_compile_options(hardware_link101 PRIVATE -Wall -Wextra)
