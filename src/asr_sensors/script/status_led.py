import board
import neopixel_spi as neopixel
import time

NUM_PIXELS = 12   # change to your strip length

spi = board.SPI()

pixels = neopixel.NeoPixel_SPI(
    spi,
    NUM_PIXELS,
    brightness=0.3,
    auto_write=False
)

print("SPI NeoPixel test starting")

pixels.fill((255, 0, 0))
pixels.show()


print("Done")
