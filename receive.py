import time
import board
import digitalio as dio
import binascii
from circuitpython_nrf24l01 import RF24

# addresses needs to be in a buffer protocol object (bytearray)
addressRx = b'1Node'
addressTx = b'1Node'
# change these (digital output) pins accordingly
ce = dio.DigitalInOut(board.D5)
csn = dio.DigitalInOut(board.D6)

# using board.SPI() automatically selects the MCU's
# available SPI pins, board.SCK, board.MOSI, board.MISO
spi = board.SPI()  # init spi bus object

# we'll be using the dynamic payload size feature (enabled by default)
# initialize the nRF24L01 on the spi bus object
nrf = RF24(spi, csn, ce)
nrf.dynamic_payloads = False
nrf.payload_length  = 32

# lets create a list of payloads to be streamed to the nRF24L01 running slave()
buffers = []
SIZE = 32  # we'll use SIZE for the number of payloads in the list and the payloads' length
for i in range(SIZE):
    buff = b''
    for j in range(SIZE):
        buff += bytes([(j >= SIZE / 2 + abs(SIZE / 2 - i) or j <
                        SIZE / 2 - abs(SIZE / 2 - i)) + 48])
    buffers.append(buff)
    del buff

def master(count=1):  # count = 5 will transmit the list 5 times
    """Transmits a massive buffer of payloads"""
    # set address of RX node into a TX pipe
    nrf.open_tx_pipe(addressTx)
    # ensures the nRF24L01 is in TX mode
    nrf.listen = False

    success_percentage = 0
    for _ in range(count):
        now = time.monotonic() * 1000  # start timer
        result = nrf.send(buffers)
        print('Transmission took', time.monotonic() * 1000 - now, 'ms')
        for r in result:
            success_percentage += 1 if r else 0
    success_percentage /= SIZE * count
    print('successfully sent', success_percentage * 100, '%')

def binaryToDecimal(binary): 
      
    binary1 = binary 
    decimal, i, n = 0, 0, 0
    while(binary != 0): 
        dec = binary % 10
        decimal = decimal + dec * pow(2, i) 
        binary = binary//10
        i += 1
    print(decimal)

def receiveData():
    nrf.open_rx_pipe(0, addressRx)
    nrf.listen = True

    count = ''
    start = 0
    clock = 0;
    indexer = 0
    accelerationX = 0
    accelerationY = 0
    accelerationZ = 0
    batteryLevel = 0
    humidity = 0
    temperature = 0
    binaryCount = 0
    actualData = 0
    extension = 0
    dataString = ''
    while True:
        if nrf.any():
            binaryString = ''
            message = ''
            timestamp = ''
            count = ''
            rx = nrf.recv()
            if chr(rx[0]) == '$':
                start = 1
                indexer += 1
            elif chr(rx[0]) == '!':
                clock = 1
                #print('Clock received')
            elif chr(rx[0]) == '%':
                actualData = 1
                #print('Actual data received')
            elif chr(rx[0]) == '&':
                extension = 1
                #print('Extension received')

            #print(rx)
            for i in range(1, len(rx)):
                if start:
                   if chr(rx[i]) == 'b':
                       #print(int(count))
                       #print('b')
                       binaryCount = int(count)
                       for m in range(0,binaryCount):
                          binaryString += '0'
                       count = ''
                   elif chr(rx[i]) == 'w':
                       #print(int(count))
                       #print('w')
                       binaryCount = int(count)
                       for m in range(0,binaryCount):
                          binaryString += '1'
                       count = ''
                   elif chr(rx[i]) == ';':
                       count = ''
                       start = 0;
                       break
                   else:
                       count += chr(rx[i])

                elif clock:
                   if chr(rx[i]) == ';':
                       clock = 0
                       break;
                   else:
                       dataString += chr(rx[i])
                elif actualData:
                   if chr(rx[i]) == ';':
                       actualData = 0
                       break;
                   else:
                       dataString += chr(rx[i])
                elif extension:
                   if chr(rx[i]) == ';':
                       break;
                   else:
                       dataString += chr(rx[i])

            #if start:
                #print(binaryString)

            if extension:
                print(dataString) 
                extension = 0
                dataString = ''

            start = 0
            clock = 0
            now = time.monotonic()

        time.sleep(0.01);


print("""\t\tStructural Health Monitor""")
print('\t TIMESTAMP \t ACCx \tACCy \tACCz \t  BAT \t HUM \t TEMP \t MAS \t RLE ')
receiveData()
