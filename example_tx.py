'''
Working TX example for SX1280 breakout using SAM32
'''

import board,time
import busio
import digitalio

import sx1280

CS = digitalio.DigitalInOut(board.D35)
RESET = digitalio.DigitalInOut(board.D36)
BUSY = digitalio.DigitalInOut(board.D37) # lambda DIO0
DIO1 = digitalio.DigitalInOut(board.D41)
DIO2 = digitalio.DigitalInOut(board.D42)
DIO3 = digitalio.DigitalInOut(board.D31)

DIO1.direction = digitalio.Direction.INPUT
DIO2.direction = digitalio.Direction.INPUT
DIO3.direction = digitalio.Direction.INPUT

spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)

radio = sx1280.SX1280(spi, CS, RESET, BUSY, debug=True)

# Prepare radio for Tx
radio.set_Dio_IRQ_Params(irqMask=[0x40,0x23],dio1Mask=[0x00,0x01],dio2Mask=[0x00,0x02],dio3Mask=[0x40,0x20]) # DEFAULT:TxDone IRQ on DIO1, RxDone IRQ on DIO2, HeaderError and RxTxTimeout IRQ on DIO3

cnt=0
while True:
    cnt+=1
    radio.write_Buffer('ping'+str(cnt))
    txstatus = radio.set_Tx()
    time.sleep(1)
    buf = radio.get_Irq_Status()
    [print(hex(i)) for i in buf]