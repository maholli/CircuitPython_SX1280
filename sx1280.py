import time
import digitalio
from micropython import const

try:
    from warnings import warn
except ImportError:
    def warn(msg, **kwargs):
        "Issue a warning to stdout."
        print("%s: %s" % ("Warning" if kwargs.get("cat") is None else kwargs["cat"].__name__, msg))

import adafruit_bus_device.spi_device as spidev

# Radio Commands
_RADIO_GET_STATUS                 = const(0xC0)
_RADIO_WRITE_REGISTER             = const(0x18)
_RADIO_READ_REGISTER              = const(0x19)
_RADIO_WRITE_BUFFER               = const(0x1A)
_RADIO_READ_BUFFER                = const(0x1B)
_RADIO_SET_SLEEP                  = const(0x84)
_RADIO_SET_STANDBY                = const(0x80)
_RADIO_SET_FS                     = const(0xC1)
_RADIO_SET_TX                     = const(0x83)
_RADIO_SET_RX                     = const(0x82)
_RADIO_SET_RXDUTYCYCLE            = const(0x94)
_RADIO_SET_CAD                    = const(0xC5)
_RADIO_SET_TXCONTINUOUSWAVE       = const(0xD1)
_RADIO_SET_TXCONTINUOUSPREAMBLE   = const(0xD2)
_RADIO_SET_PACKETTYPE             = const(0x8A)
_RADIO_GET_PACKETTYPE             = const(0x03)
_RADIO_SET_RFFREQUENCY            = const(0x86)
_RADIO_SET_TXPARAMS               = const(0x8E)
_RADIO_SET_CADPARAMS              = const(0x88)
_RADIO_SET_BUFFERBASEADDRESS      = const(0x8F)
_RADIO_SET_MODULATIONPARAMS       = const(0x8B)
_RADIO_SET_PACKETPARAMS           = const(0x8C)
_RADIO_GET_RXBUFFERSTATUS         = const(0x17)
_RADIO_GET_PACKETSTATUS           = const(0x1D)
_RADIO_GET_RSSIINST               = const(0x1F)
_RADIO_SET_DIOIRQPARAMS           = const(0x8D)
_RADIO_GET_IRQSTATUS              = const(0x15)
_RADIO_CLR_IRQSTATUS              = const(0x97)
_RADIO_CALIBRATE                  = const(0x89)
_RADIO_SET_REGULATORMODE          = const(0x96)
_RADIO_SET_SAVECONTEXT            = const(0xD5)
_RADIO_SET_AUTOTX                 = const(0x98)
_RADIO_SET_AUTOFS                 = const(0x9E)
_RADIO_SET_LONGPREAMBLE           = const(0x9B)
_RADIO_SET_UARTSPEED              = const(0x9D)
_RADIO_SET_RANGING_ROLE           = const(0xA3)
_PACKET_TYPE_GFSK                 = const(0x00)
_PACKET_TYPE_LORA                 = const(0x01)
_PACKET_TYPE_RANGING              = const(0x02)
_PACKET_TYPE_FLRC                 = const(0x03)
_PACKET_TYPE_BLE                  = const(0x04)




class SX1280:
    _status=bytearray(1)
    _status_mode={0:'N/A',
                  1:'N/A',
                  2:'STDBY_RC',
                  3:'STDBY_XOSC',
                  4:'FS',
                  5:'Rx',
                  6:'Tx'}
    _status_cmd={0:'N/A',
                 1:'Successful',
                 2:'Data Available',
                 3:'Timed-out',
                 4:'Error',
                 5:'Failure to Execute',
                 6:'Tx Done'}
    
    _BUFFER = bytearray(10)
    def __init__(self, spi, cs, reset, busy, *, preamble_length=8, baudrate=1500000, debug=False):
        self._device = spidev.SPIDevice(spi, cs, baudrate=baudrate, polarity=0, phase=0)
        self._reset = reset
        self._reset.switch_to_input(pull=digitalio.Pull.UP)
        self._busy = busy
        self._busy.switch_to_input()
        self.packet_type = _PACKET_TYPE_GFSK # default
        self._debug = debug

        self.reset()
        self.clear_Irq_Status()
        print(self.status)

    def _send_command(self,command):
        _size=len(command)
        with self._device as device:
            # device.write(command, end=_size)
            # device.readinto(self._BUFFER, end=_size)
            device.write_readinto(command,self._BUFFER,out_end=_size,in_end=_size)
        if self._debug:
            try:
                print('\tStatus:',hex(int(bin(self._BUFFER[0])[:4])),hex(int(bin(self._BUFFER[0])[4:7])))
                # [print(hex(i),' ',end='') for i in self._BUFFER[1:_size]]
                # print('')
            except Exception as e:
                print(e)

        return self._BUFFER[:_size]

    def _writeRegister(self,address1,address2,data):
        if self._debug:
            print('Writing to _writeRegister:',address1,address2)
        self._send_command(bytes([_RADIO_WRITE_REGISTER,address1,address2,data]))

    def _busywait(self):
        print('waiting for busy pin.',end='')
        while self._busy.value:
            print('.',end='')
            pass
        print('done!')

    def reset(self):
        self._reset.switch_to_output(value=False)
        time.sleep(0.05)  # 100 us
        self._reset.switch_to_input(pull=digitalio.Pull.UP)
        time.sleep(0.03)   # 5 ms

    def set_Standby(self,state='STDBY_RC'):
        if self._debug:
            print('Setting Standby')
        if state == 'STDBY_RC':
            self._send_command(bytes([_RADIO_SET_STANDBY, 0x00]))
        elif state == 'STDBY_XOSC':
            self._send_command(bytes([_RADIO_SET_STANDBY, 0x01]))

    def set_Packet_Type(self,packetType=_PACKET_TYPE_LORA):
        if self._debug:
            print('Setting Packet Type')
        self._send_command(bytes([_RADIO_SET_PACKETTYPE, packetType]))
        self.packet_type = packetType

    def set_RF_Freq(self,freq=[0xB8,0x9D,0x89]):
        # 0xB89D89 = 12098953 PLL steps (2.4 GHz)
        if self._debug:
            print('Setting RF Freq')
        self._send_command(bytes([_RADIO_SET_RFFREQUENCY]+freq))

    def set_Buffer_Base_Address(self,txBaseAddress=0x80,rxBaseAddress=0x00):
        if self._debug:
            print('Setting Buffer Base Address')
        self._txBaseAddress = txBaseAddress
        self._send_command(bytes([_RADIO_SET_BUFFERBASEADDRESS, txBaseAddress, rxBaseAddress]))

    def set_Modulation_Params(self,modParam1=0x70,modParam2=0x0A,modParam3=0x01):
        # LoRa: modParam1=SpreadingFactor, modParam2=Bandwidth, modParam3=CodingRate
        # LoRa with SF7, BW 1600, CR 4/5 
        # Must set PacketType first! - See Table 13-48,49,50
        if self._debug:
            print('Setting Modulation parameters')
        self._send_command(bytes([_RADIO_SET_MODULATIONPARAMS,modParam1,modParam2,modParam3]))
        if self.packet_type == _PACKET_TYPE_LORA:
            # If the Spreading Factor selected is SF5 or SF6
            if modParam1 in (0x50,0x60):
                self._writeRegister(0x09,0x25,0x1E)
            # If the Spreading Factor is SF7 or SF-8
            elif modParam1 in (0x70,0x80):
                self._writeRegister(0x09,0x25,0x37)
            # If the Spreading Factor is SF9, SF10, SF11 or SF12 
            elif modParam1 in (0x90,0xA0,0xB0,0xC0):
                self._writeRegister(0x09,0x25,0x32)
            else:
                print('Invalid Spreading Factor')

    def set_Packet_Params(self,pktParam1=0x0C,pktParam2=0x00,pktParam3=0x80,pktParam4=0x20,pktParam5=0x40,pktParam6=0x00,pktParam7=0x00):
        # 16 preamble symbols (0x0C), explicit header (0x00), 128-byte payload (0x80), CRC enabled (0x20), standard IQ (0x40)
        if self._debug:
            print('Setting Packet Parameters')
        self._send_command(bytes([_RADIO_SET_PACKETPARAMS,pktParam1,pktParam2,pktParam3,pktParam4,pktParam5,pktParam6,pktParam6]))

    def set_Tx_Param(self,power=0x1F,rampTime=0xE0):
        # power=13 dBm (0x1F), rampTime=20us (0xE0). See Table 11-47
        if self._debug:
            print('Setting Tx Parameters')
        self._send_command(bytes([_RADIO_SET_TXPARAMS,power,rampTime]))

    def write_Buffer(self,data):
        #Offset will correspond to txBaseAddress in normal operation.
        if self._debug:
            print('Writing Buffer')
        _offset = self._txBaseAddress
        _len = len(data)
        assert 0 < _len <= 252
        with self._device as device:
            device.write(bytes([_RADIO_WRITE_BUFFER,_offset])+data,end=_len)
            # device.write(data,end=_len)

    def set_Dio_IRQ_Params(self,irqMask=[0x40,0x23],dio1Mask=[0x00,0x01],dio2Mask=[0x00,0x02],dio3Mask=[0x40,0x20]):
        # TxDone IRQ on DIO1, RxDone IRQ on DIO2, HeaderError and RxTxTimeout IRQ on DIO3
        if self._debug:
            print('Setting DIO IRQ Parameters')
        self._send_command(bytes([_RADIO_SET_DIOIRQPARAMS]+irqMask+dio1Mask+dio2Mask+dio3Mask))
    def get_Irq_Status(self):
        if self._debug:
            print('Getting IRQ Status')
        self._send_command(bytes([_RADIO_GET_IRQSTATUS,0x00,0x00,0x00]))
    def clear_Irq_Status(self):
        if self._debug:
            print('Clearing IRQ Status')
        self._send_command(bytes([_RADIO_CLR_IRQSTATUS, 0xFF, 0xFF]))
    
    def set_Tx(self,pBase=0x00,pBaseCount=[0x00,0x00]):
        #Activate transmit mode with no timeout. Tx mode will stop after first packet sent.
        if self._debug:
            print('Setting Tx')
        self._send_command(bytes([_RADIO_SET_TX, pBase]+pBaseCount))
        self.status

    def set_Rx(self,pBase=0x03,pBaseCount=[0x00,0x00]):
        # pBaseCount = 16 bit parameter of how many steps to time-out
        # see Table 11-22 for pBase values
        # Time-out duration = pBase * periodBaseCount
        if self._debug:
            print('Setting Rx')
        self._send_command(bytes([_RADIO_SET_RX, pBase]+pBaseCount))

    def get_Packet_Status(self):
        #Table 11-63
        with self._device as device:
            device.write(bytes([_RADIO_GET_PACKETSTATUS]), end=1)
            device.readinto(self._BUFFER, end=6)
        self.rssiSync = (int(self._BUFFER[4])/2)
        self.snr = (int(self._BUFFER[5])/4)
        return (self.rssiSync,self.snr)

    def get_Rx_Buffer_Status(self):
        with self._device as device:
            device.write(bytes([_RADIO_GET_RXBUFFERSTATUS]), end=1)
            device.readinto(self._BUFFER, end=3)
        return self._BUFFER[1:3]
        
    def read_Buffer(self,offset,payloadLen):
        _payload = bytearray(payloadLen)
        with self._device as device:
            device.write(bytes([_RADIO_READ_BUFFER,offset]), end=2)
            device.readinto(_payload, end=payloadLen)
        return _payload


    @property
    def status(self):
        _status = bin(self._send_command(bytes([_RADIO_GET_STATUS]))[0])
        # print('{0:08b} Mode:{1}, Cmd Status:{2}'.format(int(_status),_status_mode[int(_status[:4])],_status_cmd[int(_status[4:6])]))
        # self._busywait()
        try:
            return (_status,self._status_mode[int(_status[:4])],self._status_cmd[int(_status[4:7])])
        except KeyError:
            pass
