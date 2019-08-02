import board,time
import busio
import digitalio

import sx1280

CS = digitalio.DigitalInOut(board.D35)
RESET = digitalio.DigitalInOut(board.D36)
BUSY = digitalio.DigitalInOut(board.D37) # lambda DIO0
DIO1 = digitalio.DigitalInOut(board.D41)
DIO2 = digitalio.DigitalInOut(board.D42)
DIO3 = digitalio.DigitalInOut(board.D38)

DIO1.direction = digitalio.Direction.INPUT
DIO2.direction = digitalio.Direction.INPUT
DIO3.direction = digitalio.Direction.INPUT

spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)

radio = sx1280.SX1280(spi, CS, RESET, BUSY, debug=False)

radio.set_Ranging_Params(master=True)

while True:
    radio.range()
    time.sleep(4)
    status=radio.get_Irq_Status()
    if status[2] > 0 or status[3] > 0:
        data1 = radio.read_range(raw=False)
        print('Filtered:',data1)
        data2 = radio.read_range(raw=True)
        print('Raw:\t',data2)