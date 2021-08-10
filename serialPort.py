import serial


def writeInfo(serial, width, center):
    try:
        error = width - center
        if error >= 0:
            sign = '+'
        else:
            sign = '-'
        values = bytearray([0x02, sign, error, 0x03])
        serial.write(values)
        print('Successfully Transmitted!!!')
    except:
        print('Cannot Transmitted!!!')