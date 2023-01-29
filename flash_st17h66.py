#!/usr/bin/env python3
import sys,time
import base64

irom1 = bytearray()

with open('./bin/Lenze_blinky.hex') as f:
    # hex file order is assumed to be ER_ROM_XIP - JUMP_TABLE - ER_IROM1
    sections = ['ER_IROM1']
    infiles = [irom1]
    infile_current = -1
    inbuf = None
    for line in f:
        if line[7:9] == '04':
            infile_current += 1
            print(f'Start of new ihex section found, assuming {sections[infile_current]}')
            inbuf = infiles[infile_current]
        elif line[7:9] == '00':
            inbuf.extend(bytearray.fromhex(line[9:-3]))


c0 = bytearray() # hexf header

c0.extend(bytearray.fromhex('01000000FFFFFFFF3818FF1FFFFFFFFF')) # blinky has only 1 ihex section
c0.extend(bytearray.fromhex('FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF'))
c0.extend(bytearray.fromhex('FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF'))
c0.extend(bytearray.fromhex('FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF'))
c0.extend(bytearray.fromhex('FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF'))
c0.extend(bytearray.fromhex('FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF'))
c0.extend(bytearray.fromhex('FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF'))
c0.extend(bytearray.fromhex('FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF'))
c0.extend(bytearray.fromhex('FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF'))
c0.extend(bytearray.fromhex('FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF'))
c0.extend(bytearray.fromhex('FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF'))
c0.extend(bytearray.fromhex('FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF'))
c0.extend(bytearray.fromhex('FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF'))
c0.extend(bytearray.fromhex('FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF'))
c0.extend(bytearray.fromhex('FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF'))
c0.extend(bytearray.fromhex('FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF'))
c0.extend(bytearray.fromhex('00900000FFFF00003818FF1FFFFFFFFF'))

c0[-12:-10] = int.to_bytes(len(irom1), 2, 'little') # length ER_IROM1

c1 = irom1

c = [c0,c1]
#c_test = c0
#c_test.extend(c1)
#with open('test.hexf', 'w') as f:
#    j = 512
#    for i in range(0, len(c_test), 16):
#        row = ':1' + int.to_bytes(j, 2, 'big').hex().upper() + '000' + c_test[i:i+16].hex().upper()
#        f.write(row)
#        f.write(hex((sum(bytearray.fromhex(row[1:])) % 256) - (1<<8))[-2:].upper().replace('X','0') + '\n') #UGLY!
#        if j == 528:
#            j = 2304
#        else:
#            j += 1
#    f.close()
#exit(0)


import serial
uart = serial.Serial('/dev/ttyUSB0', 9600, timeout=0.01, inter_byte_timeout=0.01)
res = uart.read(10)
while not res.endswith(b'cmd>>:'):
    uart.write(b'UXTDWU')
    time.sleep(0.05)
    res = uart.read(10)
    if res: print(res)

print('RESET MODE activated. Changing baudrate to 115200')
uart.baudrate = 115200


print('Erase + Write')
cmds = []
cmds.append(b'er512') # erase
cmds.append(b'rdrev+')
cmds.append(b'wrreg4000c890 ab000001 ')
cmds.append(b'wrreg4000c838 ff010005 ')
cmds.append(b'spifs 0 1 3 0 ')
cmds.append(b'sfmod 2 2 ')
cmds.append(b'cpnum ffffffff ')
cmds.append(b'cpbin c0 002000 ' + b'%x' % len(c0) + b' 11002000')
cmds.append(b'cpbin c1 009000 ' + b'%x' % len(c1) + b' 11005000')

for cmd in cmds:
    uart.write(cmd)
    print('sent', cmd)
    while not uart.in_waiting:
        time.sleep(0.3)
    msg = uart.read(uart.in_waiting)
    print('Response is:', msg)

    if cmd[5:9] in [b' c0 ', b' c1 ']:
        cfile = cmd[7] -48
        data = c[cfile]

        ldata = uart.write(data)
        print('sent c%d (len=%d)' % (cfile, ldata))
        while not uart.in_waiting:
            time.sleep(0.3)
        msg = uart.read(uart.in_waiting)
        print('Response is:', msg)

        uart.write(b'%08x' % sum(data))
        print('sent checksum')
        while not uart.in_waiting:
            time.sleep(0.3)
        msg = uart.read(uart.in_waiting)
        print('Response is:', msg)
