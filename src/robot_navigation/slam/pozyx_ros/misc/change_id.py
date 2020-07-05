#!/usr/bin/env python

from pypozyx import *
from pypozyx.definitions import *
from pypozyx.tools.discovery import *
from pypozyx.tools.device_list import *

def printErrorCode():
    error_code = SingleRegister()
    status = pozyx.getErrorCode(error_code)
    if status == POZYX_SUCCESS:
        print pozyx.getErrorMessage(error_code)

port = get_first_pozyx_serial_port()
pozyx = PozyxSerial(port)

pozyx.printDeviceInfo()

des = input("sure to change id? y or other: ")
print(des)

if des == 'y':
    pozyx.setNetworkId(0x6a60)
    pozyx.printDeviceInfo()
    print POZYX_FLASH_REGS
    print PozyxRegisters.NETWORK_ID
    status = pozyx.saveConfiguration(POZYX_FLASH_REGS, [PozyxRegisters.NETWORK_ID])
    if status == POZYX_SUCCESS:
        print "success"
        pozyx.resetSystem()
    else:
        printErrorCode()
else:
    pass

pozyx.ser.close()
