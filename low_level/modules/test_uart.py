from periphery import Serial
import time

uart = Serial('/dev/ttyAS3', 9600)
test_msg = b'LOOPBACK_TEST'

uart.write(test_msg)
time.sleep(0.1)

received = uart.read(len(test_msg), timeout=1)
uart.close()

if received == test_msg:
    print('PASS — UART pins are working correctly')
    print(f'Sent    : {test_msg}')
    print(f'Received: {received}')
else:
    print('FAIL — pins may be damaged')
    print(f'Sent    : {test_msg}')
    print(f'Received: {received}')