import ctypes
import os
import time

STRING_BUFFER_SIZE = 128

#
# CP2110 signal pin assignments
#
CP2110_GIO0_CLK = 0
CP2110_GIO1_RTS = 1
CP2110_GIO2_CTS = 2
CP2110_GIO3_RS485 = 3
CP2110_GIO4_TX_TOGGLE = 4
CP2110_GIO5_RX_TOGGLE = 5
CP2110_GIO6 = 6
CP2110_GIO7 = 7
CP2110_GIO8_RESET = 8
CP2110_GIO9_BACKDOOR_EN = 9
CP2110_TX = 10
CP2110_SUSPEND = 11
CP2110_SUSPEND_BAR = 12
CP2110_VID = 0x10C4
CP2110_PID = 0xEA80
CP2110_READ_TIMEOUT = 20
CP2110_WRITE_TIMEOUT = 50

#
# Determines if deviceString contains a vendor ID string, product ID string, serial
# string, device path string, manufacturer string, or product string.
#
HID_UART_GET_VID_STR = 0x01
HID_UART_GET_PID_STR = 0x02
HID_UART_GET_PATH_STR = 0x03
HID_UART_GET_SERIAL_STR = 0x04
HID_UART_GET_MANUFACTURER_STR = 0x05
HID_UART_GET_PRODUCT_STR = 0x06

STRING_LENGTH = {
    HID_UART_GET_VID_STR: 5,
    HID_UART_GET_PID_STR: 5,
    HID_UART_GET_PATH_STR: 260,
    HID_UART_GET_SERIAL_STR: 256,
    HID_UART_GET_MANUFACTURER_STR: 256,
    HID_UART_GET_PRODUCT_STR: 256
}

DECODE_LENGTH = {
    HID_UART_GET_VID_STR: 4,
    HID_UART_GET_PID_STR: 4,
    HID_UART_GET_PATH_STR: 82,
    HID_UART_GET_SERIAL_STR: 6,
    HID_UART_GET_MANUFACTURER_STR: 20,
    HID_UART_GET_PRODUCT_STR: 29
}

#
# CP2110 pin mask
#
CP2110_MASK_GPIO_0_CLK = 0x0001
CP2110_MASK_GPIO_1_RTS = 0x0002
CP2110_MASK_GPIO_2_CTS = 0x0004
CP2110_MASK_GPIO_3_RS485 = 0x0008
CP2110_MASK_TX = 0x0010
CP2110_MASK_RX = 0x0020
CP2110_MASK_GPIO_4_TX_TOGGLE = 0x0040
CP2110_MASK_GPIO_5_RX_TOGGLE = 0x0080
CP2110_MASK_SUSPEND_BAR = 0x0100
CP2110_MASK_GPIO_6 = 0x0400
CP2110_MASK_GPIO_7 = 0x0800
CP2110_MASK_GPIO_8 = 0x1000
CP2110_MASK_GPIO_9 = 0x2000
CP2110_MASK_SUSPEND = 0x4000

PIN_MASK_DICT = {
    CP2110_GIO0_CLK: CP2110_MASK_GPIO_0_CLK,
    CP2110_GIO1_RTS: CP2110_MASK_GPIO_1_RTS,
    CP2110_GIO2_CTS: CP2110_MASK_GPIO_2_CTS,
    CP2110_GIO3_RS485: CP2110_MASK_GPIO_3_RS485,
    CP2110_GIO4_TX_TOGGLE: CP2110_MASK_GPIO_4_TX_TOGGLE,
    CP2110_GIO5_RX_TOGGLE: CP2110_MASK_GPIO_5_RX_TOGGLE,
    CP2110_GIO6: CP2110_MASK_GPIO_6,
    CP2110_GIO7: CP2110_MASK_GPIO_7,
    CP2110_GIO8_RESET: CP2110_MASK_GPIO_8,
    CP2110_GIO9_BACKDOOR_EN: CP2110_MASK_GPIO_9,
    CP2110_TX: CP2110_MASK_TX,
    CP2110_SUSPEND: CP2110_MASK_SUSPEND,
    CP2110_SUSPEND_BAR: CP2110_MASK_SUSPEND_BAR
}

#
# CP2110 error code
#

HID_UART_SUCCESS = 0x00
HID_UART_DEVICE_NOT_FOUND = 0x01
HID_UART_INVALID_HANDLE = 0x02
HID_UART_INVALID_DEVICE_OBJECT = 0x03
HID_UART_INVALID_PARAMETER = 0x04
HID_UART_INVALID_REQUEST_LENGTH = 0x05
HID_UART_READ_ERROR = 0x10
HID_UART_WRITE_ERROR = 0x11
HID_UART_READ_TIMED_OUT = 0x12
HID_UART_WRITE_TIMED_OUT = 0x13
HID_UART_DEVICE_IO_FAILED = 0x14
HID_UART_DEVICE_ACCESS_ERROR = 0x15
HID_UART_DEVICE_NOT_SUPPORTED = 0x16
HID_UART_UNKNOWN_ERROR = 0xFF

HID_UART_ERR_MSG = {
    HID_UART_DEVICE_NOT_FOUND: "HID_UART_DEVICE_NOT_FOUND",
    HID_UART_INVALID_HANDLE: "HID_UART_INVALID_HANDLE",
    HID_UART_INVALID_DEVICE_OBJECT: "HID_UART_INVALID_DEVICE_OBJECT",
    HID_UART_INVALID_PARAMETER: "HID_UART_INVALID_PARAMETER",
    HID_UART_INVALID_REQUEST_LENGTH: "HID_UART_INVALID_REQUEST_LENGTH",
    HID_UART_READ_ERROR: "HID_UART_READ_ERROR",
    HID_UART_WRITE_ERROR: "HID_UART_WRITE_ERROR",
    HID_UART_READ_TIMED_OUT: "HID_UART_READ_TIMED_OUT",
    HID_UART_WRITE_TIMED_OUT: "HID_UART_WRITE_TIMED_OUT",
    HID_UART_DEVICE_IO_FAILED: "HID_UART_DEVICE_IO_FAILED",
    HID_UART_DEVICE_ACCESS_ERROR: "HID_UART_DEVICE_ACCESS_ERROR",
    HID_UART_DEVICE_NOT_SUPPORTED: "HID_UART_DEVICE_NOT_SUPPORTED",
    HID_UART_UNKNOWN_ERROR: "HID_UART_UNKNOWN_ERROR"
}

class HidDevInfo(ctypes.Structure):
    _fields_ = [("path", ctypes.c_char*512),
                ("vendor_id", ctypes.c_ushort),
                ("product_id", ctypes.c_ushort),
                ("serial_number", ctypes.c_wchar*512),
                ("release_number", ctypes.c_ushort),
                ("manufacturer_string", ctypes.c_wchar*512),
                ("product_string", ctypes.c_wchar*512),
                ("usage_page", ctypes.c_ushort),
                ("usage", ctypes.c_ushort),
                ("interface_number", ctypes.c_int)]
                
class CP2110_Enum_Ary(ctypes.Structure):
    _fields_ = [("devNums", ctypes.c_uint),
                ("devs", ctypes.POINTER(HidDevInfo))]

class SLABHIDDevice():

    def __init__(self):
        self.hidLib = ctypes.cdll.LoadLibrary("./cp2110-c-for-pi.so")
        self.device = ctypes.c_void_p()
        

    def _convert_str(self, buffer, length):
        return ''.join(map(chr, buffer[:length]))

    def get_num_devices(self):
        ctRetPtr = ctypes.c_void_p()
        ctRetPtr = self.hidLib.CP2110_enumerate_array()
        ctCp2110EnumAry = ctypes.cast(ctRetPtr, ctypes.POINTER(CP2110_Enum_Ary))
        if ctCp2110EnumAry == 0 or ctCp2110EnumAry.contents.devNums == 0:
            return None
        ctHidDevInfos = ctCp2110EnumAry.contents.devs
        devInfos = []
        #ctHidDevInfoTyp = (HidDevInfo * ctCp2110EnumAry.contents.devNums)()
        #ctHidDevInfos = ctypes.cast(ctCp2110EnumAry.contents.devs, ctHidDevInfoTyp)
        for i in range(ctCp2110EnumAry.contents.devNums):
            devInfo = {
                "path" : ctHidDevInfos[i].path,
                "vendor_id" : ctHidDevInfos[i].vendor_id,
                "product_id" : ctHidDevInfos[i].product_id,
                "serial_number" : ctHidDevInfos[i].serial_number,
                "release_number" : ctHidDevInfos[i].release_number,
                "manufacturer_string" : ctHidDevInfos[i].manufacturer_string,
                "product_string" : ctHidDevInfos[i].product_string,
                "usage_page" : ctHidDevInfos[i].usage_page,
                "usage" : ctHidDevInfos[i].usage,
                "interface_number" : ctHidDevInfos[i].interface_number,
            }
            devInfos.append(devInfo)
        #ctRetPtr = self.hidLib.CP2110_enumerate_array()
        return devInfos

    def open(self, device_num=0):
        devInfos = self.get_num_devices()
        
        # make the device number is valid
        if len(devInfos) <= device_num:
            raise SLABHIDError(-1)
        
        serNumStr = devInfos[device_num]['serial_number']
        ctSerNumStr = ctypes.c_wchar_p(serNumStr)

        self.device = self.hidLib.CP2110_init(ctSerNumStr)
        
        if not self.device:
            raise SLABHIDError(-1)


    def close(self):
        self.hidLib.CP2110_release(self.device)

    def flush_buffers(self, clear_tx, clear_rx):
        code = ctypes.c_int()
        if clear_tx and clear_rx:
            code.value = 3
        elif clear_rx:
            code.value = 2
        else:
            code.value = 1
        status = self.hidLib.CP2110_purgeFIFO(self.device, code)
        if status < 0:
            raise SLABHIDError(status)

    def reset(self):
        raise SLABHIDError(HID_UART_DEVICE_NOT_SUPPORTED)
    
    def is_opened(self):
        return NOT (self.device == false)

    def read_latch(self):
        raise SLABHIDError(HID_UART_DEVICE_NOT_SUPPORTED)

    def write_latch(self, value, mask):
        raise SLABHIDError(HID_UART_DEVICE_NOT_SUPPORTED)

    def write(self, buffer):
        status = self.hidLib.CP2110_write(self.device, buffer.encode(), len(buffer))
        if status < 0:
            raise SLABHIDError(HID_UART_DEVICE_NOT_SUPPORTED)
        else:
            return status

    def read(self, length):
        buffer = (ctypes.c_ubyte * length)()
        try:
            status = self.hidLib.CP2110_read(self.device, buffer, length)
            if status >= 0:
                return ctypes.string_at(buffer, status)
            else:
                raise SLABHIDError(status)
        finally:
            del buffer

    def get_product_string(self):
        raise SLABHIDError(HID_UART_DEVICE_NOT_SUPPORTED)

    def get_serial_string(self):
        raise SLABHIDError(HID_UART_DEVICE_NOT_SUPPORTED)

    def get_receive_fifo_size(self):
        raise SLABHIDError(HID_UART_DEVICE_NOT_SUPPORTED)

    def set_timeouts(self, readTimeout, writeTimeout):
        raise SLABHIDError(HID_UART_DEVICE_NOT_SUPPORTED)

    def get_timeouts(self):
        raise SLABHIDError(HID_UART_DEVICE_NOT_SUPPORTED)

    def get_string(self, device_num, vid, pid, options):
        raise SLABHIDError(HID_UART_DEVICE_NOT_SUPPORTED)

    def CP2110_Set_Pin(self, pin, activeLevel):
        if pin >= 0 and pin <= 12:
            self.hidLib.CP2110_setGPIOPin(self.device, pin, activeLevel)
        else:
            raise SLABHIDError(HID_UART_INVALID_PARAMETER)

    def CP2110_TogglePin(self, pin, activeLevel):
        self.CP2110_Set_Pin(pin, not activeLevel)
        time.sleep(0.1)

        self.CP2110_Set_Pin(pin, activeLevel)
        time.sleep(0.25)

        self.CP2110_Set_Pin(pin, not activeLevel)
        time.sleep(0.1)

    def SetUartConfig(self,baudrate,bytesize,parity,stopbits,rtscts):
        _baudrate = ctypes.c_uint(baudrate)
        _bytesize = ctypes.c_ubyte(bytesize)
        _parity = ctypes.c_ubyte(parity)
        _stopbits = ctypes.c_ubyte(stopbits)
        _rtscts = ctypes.c_ubyte(rtscts)
        try:
            status = self.hidLib.CP2110_setUARTConfig(self.device,_baudrate,_bytesize,_parity,_stopbits,_rtscts)
            if status <= 0:
                raise SLABHIDError(status)
        finally:
            del _baudrate 
            del _bytesize
            del _parity
            del _stopbits
            del _rtscts
            
class SLABHIDError():
    def __init__(self, error_code):
        if error_code > HID_UART_DEVICE_NOT_SUPPORTED:
            self.error_code = HID_UART_UNKNOWN_ERROR
        else:
            self.error_code = error_code

    def __str__(self):
        return "<hid device error: {}>".format(HID_UART_ERR_MSG[self.error_code])

#---------------------------------------------------------------------
    
class Serial:
    def __init__(self,port=None,baudrate=115200,bytesize=3,parity=0,stopbits=0,timeout=None,xonxoff=False,rtscts=0,write_timeout=1000,read_timeout=1000,dsrdtr=False):
        self.ser = SLABHIDDevice();   
        self.port = port
        self.baudrate = baudrate
        self.bytesize = bytesize
        self.parity = parity
        self.stopbits = stopbits
        self.timeout = timeout
        self.xonxoff = xonxoff
        self.rtscts = rtscts
        self.write_timeout = write_timeout
        self.read_timeout = read_timeout
        self.dsrdtr = dsrdtr

       
        #ser.WriteLatch(port)
        
    def enumerate(self):
        return self.ser.get_num_devices()
          
    def open(self,device_num = 0):
        self.ser.open(device_num)
        #self.ser.set_timeouts(self.read_timeout,self.write_timeout)
        self.ser.SetUartConfig(self.baudrate,self.bytesize,self.parity,self.stopbits,self.rtscts)

    def close(self):
        self.ser.close()
        
    def read(self,length):
        return self.ser.read(length)
        
    def write(self,buffer):
        return self.ser.write(buffer)
        
    def reset(self):
        self.ser.reset()
        
    def setPin(self, pin, onOff):
        self.ser.CP2110_Set_Pin(pin, onOff)

    def togglePin(self, pin):
        self.ser.CP2110_TogglePin(pin)

def test():
    serial = Serial()
    print(serial.enumerate())
    serial.open()
    serial.close()

if __name__ == '__main__': test()
        

#----------------------------------------------------------------------

