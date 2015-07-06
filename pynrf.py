#!/usr/bin/env python3

import struct
import ctypes

import RPi.GPIO as gpio
import spidev

WAIT = 0
SENT = 1
LOST = 2

class Protocol (object):

    def __init__(self, bus=0, device=0):
        self.spi = spidev.SpiDev()
        self.spi.open(bus, device)
        self.spi.max_speed_hz = 10000000

    def command_write(self, command, values=b''):
        data = struct.pack('B', command)
        data += values
        response = bytes(self.spi.xfer2(list(data)))
        return response

    def command_read(self, command, size):
        values = b'\0' * size
        response = self.command_write(command, values)
        return response

    def register_write(self, register, value):
        command = 0x20 | register
        values = struct.pack('B', value)
        response = self.command_write(command, values)
        status = response[0]
        return status

    def register_read(self, register):
        command = register
        response = self.command_read(command, 1)
        return response

    def address_write(self, register, address):
        command = 0x20 | register
        response = self.command_write(command, address)
        status = response[0]
        return status

    def address_read(self, register, size):
        response = self.command_read(register, size)
        return response


class NRF24L01P (object):

    def __init__(self, bus=0, device=0):
        self._protocol = Protocol(bus, device)
        self._reset()

    def _reset(self):
        # disable interrupts, enable CRC, CRC 16 bits, power off
        self._protocol.register_write(0x00, 0b01111100)
        # enable ACK on pipes 0, 1
        self._protocol.register_write(0x01, 0b00000011)
        # enable data pipes 0, 1
        self._protocol.register_write(0x02, 0b00000011)
        # address width 5 bytes
        self._protocol.register_write(0x03, 0b00000011)
        # retransmission delay 4 ms, 15 times
        self._protocol.register_write(0x04, 0b11111111)
        # rf channel 1
        self._protocol.register_write(0x05, 0b00000001)
        # speed 256 kbps, power 0 dBm
        self._protocol.register_write(0x06, 0b00100110)
        # reset status bits
        self._protocol.register_write(0x07, 0b01110000)
        # error counter - read only
        #self._protocol.register_write(0x08, 0b)
        # power detector - read only
        #self._protocol.register_write(0x09, 0b)
        # rx address for pipes 0..5
        self._protocol.address_write(0x0a, b'\xe7\xe7\xe7\xe7\xe7')
        self._protocol.address_write(0x0b, b'\xc2\xc2\xc2\xc2\xc2')
        self._protocol.register_write(0x0c, 0xc3)
        self._protocol.register_write(0x0d, 0xc4)
        self._protocol.register_write(0x0e, 0xc5)
        self._protocol.register_write(0x0f, 0xc6)
        # tx address
        self._protocol.address_write(0x10, b'\xe7\xe7\xe7\xe7\xe7')
        # paylo9ad size 0..5
        self._protocol.register_write(0x11, 0)
        self._protocol.register_write(0x12, 0)
        self._protocol.register_write(0x13, 0)
        self._protocol.register_write(0x14, 0)
        self._protocol.register_write(0x15, 0)
        self._protocol.register_write(0x16, 0)
        # fifo status - read only
        #self._protocol.register_write(0x17, 0)
        # dynamic payload on pipes 0, 1
        self._protocol.register_write(0x1c, 0b00000011)
        # enable dynamic payload
        self._protocol.register_write(0x1d, 0b00000100)
        # flush tx
        self._protocol.command_write(0b11100001)
        # flush rx
        self._protocol.command_write(0b11100010)

    def power_off(self):
        self._protocol.register_write(0x00, 0b01111100)

    def rx_mode(self):
        self._protocol.register_write(0x00, 0b01111111)

    def tx_mode(self):
        self._protocol.register_write(0x00, 0b01111110)

    def channel(self, channel_number):
        self._protocol.register_write(0x05, channel_number)

    def rx_status_reset(self):
        self._protocol.register_write(0x07, 0b01000000)

    def tx_status_reset(self):
        self._protocol.register_write(0x07, 0b00110000)

    def rx_status(self):
        response = self._protocol.command_read(0xff, 0)
        status = response[0]

        rx_fifo_number = (status >> 1) & 0x07
        if rx_fifo_number != 7:
            return True

        return False

    def tx_status(self):
        response = self._protocol.command_read(0xff, 0)
        status = response[0]
        if status & 0b00010000:
            return LOST

        if status & 0b00100000:
            return SENT

        return WAIT

    def tx_address(self, address):
        # pipe 0 for ack packet
        self._protocol.address_write(0x0a, address)
        # tx address
        self._protocol.address_write(0x10, address)

    def rx_address(self, address):
        # pipe 1 for rx address
        self._protocol.address_write(0x0b, address)

    def read(self, size):
        response = self._protocol.command_read(0b01100001, size)
        data = response[1:]
        return data

    def write(self, data):
        self._protocol.command_write(0b10100000, data)

    def tx_flush(self):
        self._protocol.command_write(0b11100001)

    def rx_flush(self):
        self._protocol.command_write(0b11100010)

    def rx_size(self):
        response = self._protocol.command_read(0b01100000, 1)
        size = response[1]
        return size

    def speed_power(speed, power):
        reg_value = 0
        reg_value |= (speed & 0x01) << 3
        reg_value |= (speed & 0x02) << 5

        reg_value |= (power & 0x03) << 1
        self._protocol.register_write(0x06, reg_value)

    def tx_data_lost():
        response = self._protocol.register_read(0x08)
        register_value = response[1]
        lost = register_value >> 4
        retr = register_value & 0x0f;
        return lost, retr

    def rx_signal_strength():
        response = self._protocol.register_read(0x09)
        register_value = response[1]
        return register_value


class CE (object):

    def __init__(self, pin=22):
        self.pin = pin
        gpio.setmode(gpio.BOARD)
        gpio.setup(pin, gpio.OUT)
        gpio.output(pin, gpio.LOW)

    def __del__(self):
        gpio.cleanup(self.pin)

    def activate(self):
        gpio.output(self.pin, gpio.HIGH)

    def deactivate(self):
        gpio.output(self.pin, gpio.LOW)

    def pulse(self):
        gpio.output(self.pin, gpio.HIGH)
        gpio.output(self.pin, gpio.LOW)


class Radio (object):

    def __init__(self, address, channel=1, bus=0, csdevice=0, ce_pin=22):
        self.device = NRF24L01P(bus, csdevice)
        self._ce = CE(ce_pin)
        self.device.rx_address(address)
        self.device.channel(channel)
        self._is_rx_mode = False

    def write(self, address, data):
        self._ce.deactivate()
        self.device.tx_mode()
        self._is_rx_mode = False
        self.device.tx_flush()
        self.device.tx_address(address)
        self.device.write(data)
        self.device.tx_status_reset()
        self._ce.pulse()

    def _rx_mode(self):
        self._ce.deactivate()
        self.device.rx_mode()
        self._is_rx_mode = True
        self.device.rx_flush()
        self.device.rx_status_reset()
        self._ce.activate()
        

    def read(self):
        data_size = self.device.rx_size()
        data = self.device.read(data_size)
        self.device.rx_status_reset()
        return data

    def available(self):
        if not self._is_rx_mode:
            self._rx_mode()

        is_data_available_to_read = self.device.rx_status()
        return is_data_available_to_read

    def flush(self):
        is_data_delivered = self.device.tx_status()
        return is_data_delivered

    def off(self):
        self._ce.deactivate()
        self.device.power_off()
        self._is_rx_mode = False
