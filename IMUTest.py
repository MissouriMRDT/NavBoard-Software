import board
from digitalio import DigitalInOut, Direction

spi = board.SPI()
csag = DigitalInOut(board.D5)
csag.direction = Direction.OUTPUT
csag.value = True
csm = DigitalInOut(board.D6)
csm.direction = Direction.OUTPUT
csm.value = True
sensor = adafruit_lsm9ds1.LSM9DS1_SPI(spi, csag, csm)