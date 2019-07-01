import board,time
import busio
import digitalio

import sx1280

CS = digitalio.DigitalInOut(board.D35)
RESET = digitalio.DigitalInOut(board.D36)
BUSY = digitalio.DigitalInOut(board.D43) # not on lambda breakout
DIO1 = digitalio.DigitalInOut(board.D41)
DIO2 = digitalio.DigitalInOut(board.D42)
DIO3 = digitalio.DigitalInOut(board.D31)

DIO1.direction = digitalio.Direction.INPUT
DIO2.direction = digitalio.Direction.INPUT
DIO3.direction = digitalio.Direction.INPUT

spi = busio.SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)

radio = sx1280.SX1280(spi, CS, RESET, BUSY)

# Configure radio for Lora
radio.set_Standby('STDBY_RC')
radio.set_Packet_Type() # DEFAULT: _PACKET_TYPE_LORA
radio.set_RF_Freq([0xB8,0x9D,0x89])
radio.set_Buffer_Base_Address(txBaseAddress=0x80,rxBaseAddress=0x00)
radio.set_Modulation_Params(modParam1=0x70,modParam2=0x0A,modParam3=0x01)
radio.set_Packet_Params() # DEFAULT:16 preamble symbols,explicit header,128-byte payload,CRC enabled,standard IQ 

print(radio.status)

# Prepare radio for Tx
radio.set_Tx_Param() # DEFAULT:power=13dBm,rampTime=20us
radio.write_Buffer('hello world')
radio.set_Dio_IRQ_Params() # DEFAULT:TxDone IRQ on DIO1, RxDone IRQ on DIO2, HeaderError and RxTxTimeout IRQ on DIO3

# Transmit
radio.set_Tx()
print('Message Sent')
cnt=0
while True:
    print('TxDone:{0}\tRxDone:{1}\tHeaderError/RxTxTimeout:{2}'.format(DIO1.value,DIO2.value,DIO3.value))
    cnt+=1
    time.sleep(1)
    if cnt >= 5:
        radio.write_Buffer('hello world')
        radio.set_Tx()
        cnt=0
        radio.clear_Irq_Status()
