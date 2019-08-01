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

# Configure radio for Lora
radio.set_Regulator_Mode()
radio.set_Packet_Type() # default LoRa
radio.set_Standby('STDBY_RC')
radio.set_Modulation_Params(modParam1=0x70,modParam2=0x26,modParam3=0x01)
radio.set_Packet_Params(pktParam1=0x08,pktParam2=0x00,pktParam3=0x0F,pktParam4=0x20) # DEFAULT:16 preamble symbols,explicit header,128-byte payload,CRC enabled,standard IQ 
radio.set_RF_Freq([0xB8,0x9D,0x80])
radio.set_Buffer_Base_Address(txBaseAddress=0x00,rxBaseAddress=0x00)

# Prepare radio for Tx
radio.set_Tx_Param() # DEFAULT:power=13dBm,rampTime=20us
radio.set_Dio_IRQ_Params(irqMask=[0x40,0x23],dio1Mask=[0x00,0x01],dio2Mask=[0x00,0x02],dio3Mask=[0x40,0x20]) # DEFAULT:TxDone IRQ on DIO1, RxDone IRQ on DIO2, HeaderError and RxTxTimeout IRQ on DIO3
radio.set_Dio_IRQ_Params() # DEFAULT:TxDone IRQ on DIO1, RxDone IRQ on DIO2, HeaderError and RxTxTimeout IRQ on DIO3
radio.write_Buffer('ping')

# Transmit
while True:
    radio.set_Tx(pBase=0x02,pBaseCount=[0x00,0x00])
    time.sleep(2)
