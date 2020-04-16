# misc
This is a simple unit test for a custom protocol and two serial ports. One side is connected via a NULL-MODEM
cable type-of-connection and the other side too.
The parameters are passed IN THE SAME way one side and another.

For example:

./testunit /dev/ttyS1 /dev/ttyUSB0 1 10 0 0 2 2

usage: ./testunit [SERIAL 1] [SERIAL 2] [SPEED IDX 1] [SPEED IDX 2] [DELAY RTS 1 BEFORE] [DELAY RTS 1 AFTER] [DELAY RTS 2 BEFORE] [DELAY RTS 2 AFTER]

The above example means:
Thread 1 is using serial port named /dev/ttyS1. Its speed is the indexed 1 (1200 baud) speed rate. NO RTS delay before send (RS485
struct) and NO RTS delay after sent (RS485 struct)

Thread 2 is using serial port named /dev/ttyUSB0. Its speed is the indexed 10 (230400 baud) speed rate. 2 millisecs of RTS delay
before send (RS485 struct) and 2 millisecs of RTS delay after sent (RS485 struct)

The protocol is very simple and it is a sort-of ping-pong data transfer. The transfer size is directly proportional to the speed
data rate, so at low speed (1200 baudrate) the packet size is 480 bytes at it will takes about 4 seconds to go out. In the same way
increasing the speed will increase the buffer size too, just to have a lot of data transferring between those two ports at the
same time.

I hope to be clear enough as English is not my native spoken language.

Some comments are left in Italian, so feel free to change to English and request for a push to myself writing me an e-mail:

icjtqrNOSPAM@gmail.com

obviously, removing the NOSPAM from the username.

Regards,
Gianluca
