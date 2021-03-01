import time, struct
import board
import digitalio as dio
from busio import SPI
from nrf24l01 import NRF24L01

pipes = (b'\xf0\xf0\xf0\xf0\xe1', b'1Node')

cs = dio.DigitalInOut(board.D6)
ce = dio.DigitalInOut(board.D5)

cs.direction = dio.Direction.OUTPUT
ce.direction = dio.Direction.OUTPUT

spi = SPI(board.SCK, MOSI=board.MOSI, MISO=board.MISO)
nrf = NRF24L01(spi, cs, ce, channel=0, payload_size=3)

def radio_tx(buf):
    nrf.open_tx_pipe(pipes[1])
    nrf.open_rx_pipe(1, pipes[0])
    nrf.stop_listening()


    # transmit buffer
    try:
        nrf.send(buf)
    except OSError:
        pass

i = 0
    
while True:
    radio_tx(struct.pack("B", i))
    i += 1
    time.sleep(1)
