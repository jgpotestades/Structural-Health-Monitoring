"""
    nRF24L01+ driver
    This file is part of the MicroPython project, http://micropython.org/
    MIT license; Copyright (c) 2017 Damien P. George
    https://github.com/micropython/micropython/tree/master/drivers/nrf24l01
    Modified by Rhys Thomas. Ported to work on the Raspberry Pi with Adafruit
    Blinka SPI and DigitalInOut objects.
"""

import time

# nRF24L01+ registers
CONFIG       = 0x00
EN_RXADDR    = 0x02
SETUP_AW     = 0x03
SETUP_RETR   = 0x04
RF_CH        = 0x05
RF_SETUP     = 0x06
STATUS       = 0x07
RX_ADDR_P0   = 0x0a
TX_ADDR      = 0x10
RX_PW_P0     = 0x11
FIFO_STATUS  = 0x17
DYNPD	     = 0x1c

# CONFIG register
EN_CRC       = 0x08 # enable CRC
CRCO         = 0x04 # CRC encoding scheme; 0=1 byte, 1=2 bytes
PWR_UP       = 0x02 # 1=power up, 0=power down
PRIM_RX      = 0x01 # RX/TX control; 0=PTX, 1=PRX

# RF_SETUP register
POWER_0      = 0x00 # -18 dBm
POWER_1      = 0x02 # -12 dBm
POWER_2      = 0x04 # -6 dBm
POWER_3      = 0x06 # 0 dBm
SPEED_1M     = 0x00
SPEED_2M     = 0x08
SPEED_250K   = 0x20

# STATUS register
RX_DR        = 0x40 # RX data ready; write 1 to clear
TX_DS        = 0x20 # TX data sent; write 1 to clear
MAX_RT       = 0x10 # max retransmits reached; write 1 to clear

# FIFO_STATUS register
RX_EMPTY     = 0x01 # 1 if RX FIFO is empty

# constants for instructions
R_RX_PL_WID  = 0x60 # read RX payload width
R_RX_PAYLOAD = 0x61 # read RX payload
W_TX_PAYLOAD = 0xa0 # write TX payload
FLUSH_TX     = 0xe1 # flush TX FIFO
FLUSH_RX     = 0xe2 # flush RX FIFO
NOP          = 0xff # use to read STATUS register

class NRF24L01:
    def __init__(self, spi, cs, ce, channel=46, payload_size=16):
        assert payload_size <= 32

        self.buf = bytearray(1)

        # store the pins
        self.spi = spi
        self.cs = cs
        self.ce = ce

        # init the SPI bus and pins
        self.init_spi(2000000)

        # reset everything
        self.ce.value = 0
        self.ce.value = 1

        self.payload_size = payload_size
        self.pipe0_read_addr = None
        time.sleep(0.005) # 5ms sleep

        # set address width to 5 bytes and check for device present
        self.reg_write(SETUP_AW, 0b11)
        if self.reg_read(SETUP_AW) != 0b11:
            raise OSError("nRF24L01+ Hardware not responding")

        # disable dynamic payloads
        self.reg_write(DYNPD, 0)

        # auto retransmit delay: 1750us
        # auto retransmit count: 8
        self.reg_write(SETUP_RETR, (6 << 4) | 8)

        # set rf power and speed
        self.set_power_speed(POWER_3, SPEED_250K) # Best for point to point links

        # init CRC
        self.set_crc(2)

        # clear status flags
        self.reg_write(STATUS, RX_DR | TX_DS | MAX_RT)

        # set channel
        self.set_channel(channel)

        # flush buffers
        self.flush_rx()
        self.flush_tx()

    def init_spi(self, baudrate):
        # take SPI lock
        while not self.spi.try_lock():
            pass

        # configure peripheral
        self.spi.configure(baudrate=baudrate)

    def reg_read(self, reg):
        self.cs.value = 0
        self.spi.readinto(self.buf, write_value=reg)
        self.spi.readinto(self.buf)
        self.cs.value = 1
        return self.buf[0]

    def reg_write_bytes(self, reg, buf):
        self.cs.value = 0
        self.spi.readinto(self.buf, write_value=(0x20 | reg))
        self.spi.write(buf)
        self.cs.value = 1
        return self.buf[0]

    def reg_write(self, reg, value):
        self.cs.value = 0
        self.spi.readinto(self.buf, write_value=(0x20 | reg))
        ret = self.buf[0]
        self.spi.readinto(self.buf, write_value=value)
        self.cs.value = 1
        return ret

    def flush_rx(self):
        self.cs.value = 0
        self.spi.readinto(self.buf, write_value=FLUSH_RX)
        self.cs.value = 1

    def flush_tx(self):
        self.cs.value = 0
        self.spi.readinto(self.buf, write_value=FLUSH_TX)
        self.cs.value = 1

    # power is one of POWER_x defines; speed is one of SPEED_x defines
    def set_power_speed(self, power, speed):
        setup = self.reg_read(RF_SETUP) & 0b11010001
        self.reg_write(RF_SETUP, setup | power | speed)

    # length in bytes: 0, 1 or 2
    def set_crc(self, length):
        config = self.reg_read(CONFIG) & ~(CRCO | EN_CRC)
        if length == 0:
            pass
        elif length == 1:
            config |= EN_CRC
        else:
            config |= EN_CRC | CRCO
        self.reg_write(CONFIG, config)

    def set_channel(self, channel):
        self.reg_write(RF_CH, min(channel, 125))

    # address should be a bytes object 5 bytes long
    def open_tx_pipe(self, address):
        assert len(address) == 5
        self.reg_write_bytes(RX_ADDR_P0, address)
        self.reg_write_bytes(TX_ADDR, address)
        self.reg_write(RX_PW_P0, self.payload_size)

    # address should be a bytes object 5 bytes long
    # pipe 0 and 1 have 5 byte address
    # pipes 2-5 use same 4 most-significant bytes as pipe 1, plus 1 extra byte
    def open_rx_pipe(self, pipe_id, address):
        assert len(address) == 5
        assert 0 <= pipe_id <= 5
        if pipe_id == 0:
            self.pipe0_read_addr = address
        if pipe_id < 2:
            self.reg_write_bytes(RX_ADDR_P0 + pipe_id, address)
        else:
            self.reg_write(RX_ADDR_P0 + pipe_id, address[0])
        self.reg_write(RX_PW_P0 + pipe_id, self.payload_size)
        self.reg_write(EN_RXADDR, self.reg_read(EN_RXADDR) | (1 << pipe_id))

    def start_listening(self):
        self.reg_write(CONFIG, self.reg_read(CONFIG) | PWR_UP | PRIM_RX)
        self.reg_write(STATUS, RX_DR | TX_DS | MAX_RT)

        if self.pipe0_read_addr is not None:
            self.reg_write_bytes(RX_ADDR_P0, self.pipe0_read_addr)

        self.flush_rx()
        self.flush_tx()
        self.ce.value = 1
        time.sleep(0.00013)

    def stop_listening(self):
        self.ce.value = 0
        self.flush_tx()
        self.flush_rx()

    # returns True if any data available to recv
    def any(self):
        return not bool(self.reg_read(FIFO_STATUS) & RX_EMPTY)

    def recv(self):
        # get the data
        self.cs.value = 0
        self.spi.readinto(self.buf, write_value=R_RX_PAYLOAD)
        buf = self.spi.read(self.payload_size)
        self.cs.value = 1
        # clear RX ready flag
        self.reg_write(STATUS, RX_DR)

        return buf

    # blocking wait for tx complete
    def send(self, buf, timeout=0.500):
        self.send_start(buf)
        start = time.time()
        result = None
        while result is None and (time.time() - start) < timeout:
            result = self.send_done() # 1 == success, 2 == fail
        if result == 2:
            raise OSError("send failed")

    # non-blocking tx
    def send_start(self, buf):
        # power up
        self.reg_write(CONFIG, (self.reg_read(CONFIG) | PWR_UP) & ~PRIM_RX)
        time.sleep(0.00015)
        # send the data
        self.cs.value = 0
        self.spi.readinto(self.buf, write_value=W_TX_PAYLOAD)
        self.spi.write(buf)
        if len(buf) < self.payload_size:
            self.spi.write(b'\x00' * (self.payload_size - len(buf))) # pad out data
        self.cs.value = 1

        # enable the chip so it can send the data
        self.ce.value = 1
        time.sleep(0.000015) # needs to be >10us
        self.ce.value = 0

    # returns None if send still in progress, 1 for success, 2 for fail
    def send_done(self):
        if not (self.reg_read(STATUS) & (TX_DS | MAX_RT)):
            return None # tx not finished

        # either finished or failed: get and clear status flags, power down
        status = self.reg_write(STATUS, RX_DR | TX_DS | MAX_RT)
        self.reg_write(CONFIG, self.reg_read(CONFIG) & ~PWR_UP)
        return 1 if (status & TX_DS) else 2
