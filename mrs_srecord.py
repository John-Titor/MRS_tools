#
# S-record handling for MRS devices.
#
# Notes:
#
# S-record files for S32K (CC16 etc.) modules include S0 records denoting the
# intended target device part / order numbers and hardware
# revisions, e.g.:
#
# MRS-Check>>>400803,400948,401395,401380,400817,400811,400814,400819,401368<<<B,B1 // V2.6.0.0 Lib-S32K-2.6.0
# MRS-ProgData>>>125 kBit;V0.0.0;CC16WP Application
# MRS-ExtraData>>>1.154.300.00,1.154.211.00,1.154.300.0010,1.154.300.0200,1.154.300.10,1.154.302.00,1.154.302.03,1.154.310.00,1.154.320.00,1.154.330.00,1.154.343.03,1.154.;B,B1;
#
# An S5 record may also be present. These records are never sent to
# the module.
#


import struct
from binascii import crc32


class Srecord(object):
    """S-record line parser for 16- and 32-bit address records"""

    def __init__(self, flavor, address, data):
        self._flavor = flavor
        self._address = address
        self._data = data

    @classmethod
    def from_line(cls, line):
        if ((line[0] != 'S') or (line[1] not in '135790')):
            raise RuntimeError(f'malformed or unsupported S-record header {line}')
        count = int(f'0x{line[2:4]}', 16)
        if len(line) != ((count * 2) + 4):
            raise RuntimeError(f'S-record length {count} not {(len(line) - 4) / 2} as expected: \n {line}')
        payload = bytes.fromhex(line[4:-2])

        flavor = line[1]
        check = int(f'0x{line[-2:]}', 16)

        # parse selected S-records
        if flavor == '0':
            address = 0
            data = payload[2:].decode('ascii')
        elif flavor == '1':
            address = struct.unpack_from('>H', payload)[0]
            data = payload[2:]
        elif flavor == '3':
            address = struct.unpack_from('>I', payload)[0]
            data = payload[4:]
        elif flavor == '5':
            address = struct.unpack_from('>H', payload)[0]
            data = None
        elif flavor == '7':
            address = struct.unpack_from('>I', payload)[0]
            data = None
        elif flavor == '9':
            address = struct.unpack_from('>H', payload)[0]
            data = None

        return cls(flavor, address, data)

    @classmethod
    def from_data(cls, flavor, address, data):
        return cls(flavor, address, data)

    @property
    def flavor(self):
        return self._flavor

    @property
    def address(self):
        return self._address

    @property
    def data(self):
        return self._data

    @property
    def check(self):
        """compute the checksum byte"""
        if self._flavor in '0159':
            adata = self._address.to_bytes(2, byteorder='big')
        elif self._flavor in '37':
            adata = self._address.to_bytes(4, byteorder='big')
        accum = len(adata) + 1
        for b in adata:
            accum += b
        if self._data is not None:
            accum += len(self._data)
            for b in self._data:
                accum += b
        return ~accum & 0xff

    def __str__(self):
        if self._flavor == '0':
            address = '0000'
            payload = self._data.encode("ascii")
        if self._flavor == '1':
            address = f'{self._address:04X}'
            payload = self._data.hex().upper()
        if self._flavor == '3':
            address = f'{self._address:08X}'
            payload = self._data.hex().upper()
        if self._flavor == '5':
            address = f'{self._address:04X}'
            payload = ''
        if self._flavor == '7':
            address = f'{self._address:08X}'
            payload = ''
        if self._flavor == '9':
            address = f'{self._address:04X}'
            payload = ''
        length = ((len(address) + len(payload)) >> 1) + 1

        record = f'S{self._flavor}{length:02X}{address}{payload}{self.check:02X}'
        return record


class S32K_Srecords(object):
    '''Load and fix up S-records for S32K-based targets'''

    def __init__(self, path, args, mcu_type):

        # configure properties for the MCU
        if mcu_type == 6:
            # S32K144 with MRS bootloader; bootloader owns the first 64K of the actual flash.
            self._flash_base = 0x10000
            flash_limit = 0x80000
        else:
            raise RuntimeError(f'unsupported MCU type {mcu_type}')

        # read the input file
        try:
            with path.open() as f:
                lines = f.readlines()
        except Exception as e:
            raise RuntimeError(f'could not read S-records from {path}')

        # convert lines to S-records
        self._s0_records = list()
        mem_records = dict()
        self._image_entry = None
        header_base = flash_limit
        image_limit = self._flash_base
        for line in lines:
            srec = Srecord.from_line(line.strip())

            # S0 record?
            if srec.flavor == '0':
                self._s0_records += [srec]

            # flash data
            if srec.flavor == '3':
                limit = srec.address + len(srec.data)
                if (srec.address < self._flash_base) or (limit > flash_limit):
                    raise RuntimeError(f'address {srec.address:#08x} outside flashable area')
                mem_records[srec.address] = srec.data
                if limit > image_limit:
                    image_limit = limit
                if srec.address < header_base:
                    header_base = srec.address

            # entrypoint
            if srec.flavor == '7':
                self._image_entry = srec.address

        # sanity-check the entrypoint
        if self._image_entry is None:
            raise RuntimeError(f'missing entrypoint')
        if (self._image_entry < header_base) or (self._image_entry > image_limit):
            raise RuntimeError(f'entrypoint outside app')

        # image must start at base of flash
        if header_base != self._flash_base:
            raise RuntimeError(f'data does not start at base of flash ({header_base:#x} != {self._flash_base:#x})')

        # round the image limit up to a multiple of 256
        # XXX is this a hard requirement, or only for signing?
        mod = image_limit % 256
        if mod != 0:
            image_limit += 256 - mod

        # build the memory array, default locations are all-zero
        self._mem_buf = bytearray(image_limit - self._flash_base)

        # and populate it with srecord data
        for address, payload in mem_records.items():
            rec_offset = address - self._flash_base
            rec_limit = rec_offset + len(payload)
            self._mem_buf[rec_offset:rec_limit] = payload

        # do header fixups
        self._fix_flash_header()

    def _fix_flash_header(self):
        # typedef struct
        # {
        #     uint32_t header_key;          default 0x12345678
        #     uint32_t header_crc;          crc32 from 0x10008-0x10fff
        #     uint32_t app_header_version;  default 1
        #     uint32_t application_crc;     crc32 from 0x11000 of application_length
        #     uint32_t application_length;  app length (multiple of 256)
        #     uint8_t sw_version[32];       default "NO PROG"
        #     uint8_t reserve[460];         zeros
        #     uint8_t signature_key[512];   zeros
        # } struct_hal_sys_app_header_t;
        #
        header_fmt = '<IIIII20s'

        # parse the current header state
        (header_key,
         header_crc,
         app_header_version,
         application_crc,
         application_length,
         sw_version) = struct.unpack_from(header_fmt, self._mem_buf, 0)

        # compute the application CRC and length
        new_app_crc = crc32(self._mem_buf[0x1000:])
        new_app_length = len(self._mem_buf) - 0x1000

        #  check whether the header has already been populated...
        if application_crc != 0:

            # check the magic number
            if app_header_version != 1:
                raise RuntimeError(f'unsupported flash header version {app_header_version}')
            # verify that the header matches our expectations
            if application_length != new_app_length:
                raise RuntimeError(f'app length mismatch {new_app_length} != {application_length}')
            if application_crc != new_app_crc:
                raise RuntimeError(f'app crc mismatch {new_app_crc:#08x} != {application_crc:#08x}')
            new_hdr_crc = crc32(self._mem_buf[0x8:0x1000])
            if header_crc != new_hdr_crc:
                raise RuntimeError(f'header crc mismatch {new_hdr_crc:#08x} != {header_crc:#08x}')

        else:
            # Fill in the header with the computed app CRC and size
            #
            struct.pack_into(header_fmt, self._mem_buf, 0,
                             0x12345678,        # header_key
                             0,                 # header_crc
                             1,                 # app_header_version
                             new_app_crc,       # application_crc
                             new_app_length,    # application_length
                             'NO_PROG\0\0\0\0\0\0\0\0\0\0\0\0\0'.encode('ascii'))

            # compute the header CRC
            new_hdr_crc = crc32(self._mem_buf[0x8:0x1000])

            # rewrite the header with the computed header CRC
            struct.pack_into(header_fmt, self._mem_buf, 0,
                             0x12345678,        # header_key
                             new_hdr_crc,       # header_crc
                             1,                 # app_header_version
                             new_app_crc,       # application_crc
                             new_app_length,    # application_length
                             'NO_PROG\0\0\0\0\0\0\0\0\0\0\0\0\0'.encode('ascii'))

        # Note also FlashConfiguration section just after vectors:
        #
        # .section .FlashConfig, "a"
        # .long 0xFFFFFFFF     /* 8 bytes backdoor comparison key           */
        # .long 0xFFFFFFFF     /*                                           */
        # .long 0xFFFFFFFF     /* 4 bytes program flash protection bytes    */
        # .long 0xFFFF7FFE     /* FDPROT:FEPROT:FOPT:FSEC(0xFE = unsecured) */
        #
        # MRS-generated image seems to contain default values, but keep this
        # in case we want to try patching it later.
        #

    def text_records(self, upload_only=False):
        """generator yielding text S-records"""

        if not upload_only:
            for srec in self._s0_records:
                yield str(srec)

        for offset in range(0, len(self._mem_buf), 32):
            address = self._flash_base + offset
            payload = self._mem_buf[offset:offset + 32]
            yield str(Srecord('3', address, payload))

        yield str(Srecord('7', self._image_entry, None))

    def upload_records(self):
        """generator yielding S-records in ready-to-send format"""
        for srec in self.text_records(upload_only=True):
            # first two bytes to send are ascii, remainder are literals
            yield bytearray(srec[0:2], 'ascii') + bytes.fromhex(srec[2:])


class HCS08_Srecords(object):
    '''read S-records and fix up for HCS08-based targets'''

    @staticmethod
    def sum(srec):
        count = int(f'0x{srec[2:4]}', 16)
        sum = 0
        for ofs in range(2, 2 + (count * 2), 2):
            sum += int(f'0x{srec[ofs:ofs+2]}', 16)
        return (~sum) & 0xff

    def __init__(self, path, args):
        self._hexbytes = dict()
        self._s0_records = list()

        # read the input file
        try:
            with path.open() as f:
                lines = f.readlines()
        except Exception as e:
            raise RuntimeError(f'could not read S-records from {path}')

        # populate the bytes array with data from S1 records
        for line in lines:
            if line[0] != 'S':
                raise RuntimeError(f'malformed S-record: {line}')
            if line[1] in '2378':
                raise RuntimeError(f'unsupported S{line[1]} record: {line}')
            if line[1] == '0':
                self._s0_records += [line.strip()]
            if line[1] != '1':
                # ignore anything that's not an S1 record; we discard S[056],
                # and fake S9 at the end
                continue

            # get the starting address of the line
            address = int(f'0x{line[4:8]}', 16)

            # and paste bytes into memory
            hexbytes = line[8:-3]
            # log(f"{address:04x}: {hexbytes}")
            self._insert_bytes(address, hexbytes)

        # do we need to patch vectors into the thunk table, or does
        # this image already have them?
        if self._word(0xaffd) is None:

            # locate the reset vector
            self._entry = self._vector(0)
            if self._entry is None:
                raise RuntimeError("no reset vector")

            # extract vectors and patch them into the thunk space
            try:
                default_vector = self._vector(32)
            except KeyError:
                default_vector = self._entry

            for number in range(0, 32):
                self._insert_thunk(number, default_vector)

        # prune values outside the writable ROM space
        for address in sorted(self._hexbytes):
            if ((address < 0x2200) or (address > 0xbdff)):
                # log(f"remove {address:04x}")
                del self._hexbytes[address]

    def _insert_bytes(self, address, hexbytes):
        while len(hexbytes) > 0:
            self._hexbytes[address] = hexbytes[0:2]
            hexbytes = hexbytes[2:]
            address += 1

    def _insert_thunk(self, number, default_vector):
        try:
            vector = self._vector(number)
        except KeyError:
            vector = default_vector
        self._insert_bytes(0xaffc - (number * 4), "CC" + vector + "9D")

    def _vector(self, number):
        address = 0xfffe - (number * 2)
        return self._word(address)

    def _word(self, address):
        try:
            hi = self._hexbytes[address]
            lo = self._hexbytes[address + 1]
            return hi + lo
        except KeyError:
            return None

    def text_records(self, upload_only=False):
        """generator yielding text S-records"""
        if not upload_only:
            for line in self._s0_records:
                yield line

        addresses = sorted(self._hexbytes)
        while len(addresses):

            # address of next S1 record to emit
            srec_addr = addresses[0]
            srec_hexbytes = ""
            byte_addr = srec_addr

            while (byte_addr in addresses) and len(srec_hexbytes) < 64:
                srec_hexbytes += self._hexbytes[byte_addr]
                addresses = addresses[1:]
                byte_addr += 1

            srec = f"S1{(len(srec_hexbytes) >> 1) + 3:02X}{srec_addr:04X}{srec_hexbytes}"
            srec += f"{self.sum(srec):02X}"
            yield srec

        yield "S9030000FC"

    def upload_records(self):
        """generator yielding S-records in ready-to-send format"""
        for srec in self.text_records(upload_only=True):
            # first two bytes to send are ascii, remainder are literals
            yield bytearray(srec[0:2], 'ascii') + bytes.fromhex(srec[2:])


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='MRS Microplex 7* and CC16 CAN flasher')
    actiongroup = parser.add_mutually_exclusive_group(required=True)
    actiongroup.add_argument('--print-fixed-hcs08-srecords',
                             type=Path,
                             metavar='SRECORD_FILE',
                             help='translate an S-record file as it would be for upload to an HCS08 module')
    actiongroup.add_argument('--print-fixed-s32k-srecords',
                             type=Path,
                             metavar='SRECORD_FILE',
                             help='translate an S-record file as it would be for upload to an S32K module')

    args = parser.parse_args()
    if args.print_fixed_hcs08_srecords is not None:
        do_print_hcs08_srecords(args.print_fixed_hcs08_srecords, args)
    if args.print_fixed_s32k_srecords is not None:
        do_print_s32k_srecords(args.print_fixed_s32k_srecords, args)
