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
    def __init__(self, spi, cs, reset, busy, *, preamble_length=8, baudrate=500000):
        self._device = spidev.SPIDevice(spi, cs, baudrate=baudrate, polarity=0, phase=0)
        self._reset = reset
        self._reset.switch_to_input(pull=digitalio.Pull.UP)
        self._busy = busy
        self._busy.switch_to_input()

        self.reset()
        print(self.status)

        # self.enable_XOSC()
        self.clear_Irq_Status()
        # self.set_Rx()
        # self.status()

    def _readregister(self, address, buf, length=None):
        # Read a number of bytes from the specified address into the provided
        # buffer.  If length is not specified (the default) the entire buffer
        # will be filled.
        if length is None:
            length = len(buf)
        with self._device as device:
            self._BUFFER[0] = address & 0x7F  # Strip out top bit to set 0
                                              # value (read).
            device.write(self._BUFFER, end=1)
            device.readinto(buf, end=length)

    def _read_u8(self, address):
        # Read a single byte from the provided address and return it.
        self._read_into(address, self._BUFFER, length=1)
        return self._BUFFER[0]

    def _write_from(self, address, buf, length=None):
        # Write a number of bytes to the provided address and taken from the
        # provided buffer.  If no length is specified (the default) the entire
        # buffer is written.
        if length is None:
            length = len(buf)
        with self._device as device:
            self._BUFFER[0] = (address | 0x80) & 0xFF  # Set top bit to 1 to
                                                       # indicate a write.
            device.write(self._BUFFER, end=1)
            device.write(buf, end=length)

    def _write_u8(self, address, val):
        # Write a byte register to the chip.  Specify the 7-bit address and the
        # 8-bit value to write to that address.
        with self._device as device:
            self._BUFFER[0] = (address | 0x80) & 0xFF  # Set top bit to 1 to
                                                       # indicate a write.
            self._BUFFER[1] = val & 0xFF
            device.write(self._BUFFER, end=2)
    
    def _send_command(self,command):
        _size=len(command)
        with self._device as device:
            device.write(command, end=_size)
            device.readinto(self._BUFFER, end=_size)
        return self._BUFFER[:_size]

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

    def enable_XOSC(self):
        self._send_command(bytes([_RADIO_SET_STANDBY, 0x01]))

    def clear_Irq_Status(self):
        self._send_command(bytes([_RADIO_CLR_IRQSTATUS, 0xFF, 0xFF]))

    def set_Rx(self,pBase=0x03,pBaseCountLow=0x00,pBaseCountHigh=0x00):
        # pBaseCount = 16 bit parameter of how many steps to time-out
        # see Table 11-22 for pBase values
        # Time-out duration = pBase * periodBaseCount
        self._send_command(bytes([_RADIO_SET_RX, pBase, pBaseCountLow, pBaseCountHigh]))

    def set_Packet_Type(self,packetType=_PACKET_TYPE_LORA):
        self._send_command(bytes([_RADIO_SET_PACKETTYPE, packetType]))

    def set_RF_Freq(self,freq1=0xB8,freq2=0x9D,freq3=0x89):
        self._send_command(bytes([_RADIO_SET_RFFREQUENCY, freq1, freq2, freq3]))

    @property
    def status(self):
        _status = bin(self._send_command(bytes([_RADIO_GET_STATUS]))[0])
        # print('{0:08b} Mode:{1}, Cmd Status:{2}'.format(int(_status),_status_mode[int(_status[:4])],_status_cmd[int(_status[4:6])]))
        self._busywait()
        return (_status,self._status_mode[int(_status[:4])],self._status_cmd[int(_status[4:6])])