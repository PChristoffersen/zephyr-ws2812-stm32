# Zephyr WS2812 driver for stm32 

A led_strip driver for Zephyr that uses STM32 timers.

The driver uses STM32 timers and DMA to output WS2812 data
on a PWM pin.

# Example

See the [examples](examples/led_strip/) for an example of how
to use the driver on an STM32 BluePill board.

# Notes

The driver have only been tested on STM32F1x, but may work on other
chip versions.
