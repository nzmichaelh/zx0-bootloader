"""Host side programmer for the ZX0 bootloader."""

import enum
from dataclasses import dataclass
import random
import struct
from typing import List, Optional, Sequence

import click
import smbus
import intelhex
from crccheck.crc import Crc16Xmodem

FILL = 0xFF


class Command(enum.IntEnum):
    BININFO = 0x0001
    INFO = 0x0002
    RESET_INTO_APP = 0x0003
    RESET_INTO_BOOTLOADER = 0x0004
    START_FLASH = 0x0005
    WRITE_FLASH_PAGE = 0x0006
    CHKSUM_PAGES = 0x0007
    READ_WORDS = 0x0008
    WRITE_WORDS = 0x0009
    DMESG = 0x0010


class Flag(enum.IntFlag):
    SERIAL_OUT = 0x80
    SERIAL_ERR = 0xC0
    CMDPKT_LAST = 0x40
    CMDPKT_BODY = 0x00
    FLAG_MASK = 0xC0
    SIZE_MASK = 63


class Status(enum.IntEnum):
    OK = 0
    INVALID_CMD = 1
    ERROR = 2


@dataclass
class BinInfo:
    mode: int
    flash_page_size: int
    flash_num_pages: int
    max_message_size: int
    family_id: int


class HF2:
    MAX_PACKET = 31

    def __init__(self, bus, address, command):
        self._bus = bus
        self._address = address
        self._command = command

        self._tag = random.randint(0, 30000)

    def send(self, command: Command, args: Optional[bytes] = None) -> None:
        self._tag = (self._tag + 1) & 0xFFFF
        msg = struct.pack('<IHBB', command, self._tag, 0, 0)
        if args:
            msg += args

        for i in range(0, len(msg), self.MAX_PACKET):
            remain = len(msg) - i
            take = min(remain, self.MAX_PACKET)
            header = take
            final = (i + self.MAX_PACKET >= len(msg))
            if final:
                header |= Flag.CMDPKT_LAST
            packet = [header] + list(msg[i:i + take])
            self._bus.write_i2c_block_data(self._address, self._command,
                                           packet)

    def exec(self, command: Command, args: Optional[bytes] = None) -> bytes:
        self.send(command, args)
        got = bytes(self._bus.read_block_data(self._address, self._command))
        header, tag, status, status_info = struct.unpack_from('<BHBB', got, 0)

        if (header & Flag.FLAG_MASK) != Flag.CMDPKT_LAST:
            raise Exception(
                'Target returned a header {header:x}, want the last packet')
        if tag != self._tag:
            raise Exception(
                f'Target returned the wrong tag. Want {self._tag}, got {tag}')
        if status != Status.OK:
            raise Exception(f'Target returned status {status}: {status_info}')

        return got[5:]

    def get_bininfo(self) -> BinInfo:
        data = self.exec(Command.BININFO)
        return BinInfo(*struct.unpack_from('<IIIII', data, 0))

    def write_flash_page(self, address: int, data: bytes) -> None:
        args = struct.pack('<I', address) + data
        self.exec(Command.WRITE_FLASH_PAGE, args)

    def chksum_pages(self, address: int, num_pages: int = 1) -> Sequence[int]:
        args = struct.pack('<II', address, num_pages)
        got = self.exec(Command.CHKSUM_PAGES, args)
        return [
            struct.unpack_from('<H', got, x)[0] for x in range(0, len(got), 2)
        ]

    def read_words(self, address: int, num_words: int = 1) -> Sequence[int]:
        args = struct.pack('<II', address, num_words)
        got = self.exec(Command.READ_WORDS, args)
        return [
            struct.unpack_from('<I', got, x)[0] for x in range(0, len(got), 4)
        ]

    def reset_into_app(self):
        self.send(Command.RESET_INTO_APP)


def _show_address(v) -> str:
    if v is None:
        return ''
    return '%x' % v


@click.command()
@click.option('--bus', default=3, help='I2C bus')
@click.option('--address', default=0x22, help='Device address')
@click.option('--command', default=0xFE, help='HF2 command')
@click.option('--program',
              type=click.File(),
              help='Hex file to program to the device')
@click.option('--start',
              is_flag=True,
              help='Start the application running after any other operations')
def main(bus: int, address: int, command: int, program: str, start: bool):
    b = smbus.SMBus(bus)
    h = HF2(b, address, command)

    info = h.get_bininfo()
    click.echo(
        f'Found a device with {info.flash_num_pages} pages of {info.flash_page_size} bytes'
    )

    if program:
        hex = intelhex.IntelHex()
        hex.fromfile(program, format='hex')

        page = info.flash_page_size
        addresses: List[int] = []
        for start, end in hex.segments():
            addresses.extend(range(start, end, page))

        with click.progressbar(addresses,
                               label='Program',
                               item_show_func=_show_address) as bar:
            for address in bar:
                data = [
                    hex[x] for x in range(address, min(address + page, end))
                ]
                data += [FILL] * (page - len(data))
                assert len(data) == page
                h.write_flash_page(address, bytes(data))
                got, want = h.chksum_pages(address)[0], Crc16Xmodem.calc(data)
                if got != want and address >= 16384:
                    raise Exception(
                        f'Program error at address {address:x}. Want CRC {want:x}, got {got:x}'
                    )

    if start:
        click.echo('Starting app')
        h.reset_into_app()


if __name__ == '__main__':
    main()
