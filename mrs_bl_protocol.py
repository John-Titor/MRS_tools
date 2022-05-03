#
# CAN bootloader protocol handling for MRS Microplex and CC16 modules.
#

import can
import struct
import time

# Module parameters (stored in EEPROM)
PARAMETER_MAGIC = 1331
PARAMETER_MAP = [
    ('2B',  '_'),
    ('>H',  '_ParameterMagic'),
    ('>I',  'SerialNumber'),
    ('12s', 'PartNumber'),
    ('12s', 'DrawingNumber'),
    ('20s', 'Name'),
    ('8s',  'OrderNumber'),
    ('8s',  'TestDate'),
    ('>H',  'HardwareVersion'),
    ('B',   'ResetCounter'),
    ('>H',  'LibraryVersion'),
    ('B',   'ResetReasonLVD'),
    ('B',   'ResetReasonLOC'),
    ('B',   'ResetReasonILAD'),
    ('B',   'ResetReasonILOP'),
    ('B',   'ResetReasonCOP'),
    ('B',   'MCUType'),
    ('B',   'HardwareCANActive'),
    ('3B',  'Reserved1'),
    ('>H',  'BootloaderVersion'),
    ('>H',  'ProgramState'),
    ('>H',  'Portbyte1'),
    ('>H',  'Portbyte2'),
    ('>H',  'BaudrateBootloader1'),
    ('>H',  'BaudrateBootloader2'),
    ('B',   'BootloaderIDExt1'),
    ('>I',  'BootloaderID1'),
    ('B',   'BootloaderIDCRC1'),
    ('B',   'BootloaderIDExt2'),
    ('>I',  'BootloaderID2'),
    ('B',   'BootloaderIDCRC2'),
    ('20s', 'SoftwareVersion'),
    ('30s', 'ModuleName'),
    ('B',   'BootloaderCANBus'),
    ('>H',  'COPWatchdogTimeout'),
    ('7B',  'Reserved2')
]

# parameters we allow to be written
PARAMETER_WRITE_OK = [
    'BaudrateBootloader1',
    'SoftwareVersion',
    'ModuleName'
]

# encoding for BaudrateBootloader* parameters
# (there are more below 100 but not interesting)
BAUD_MAP = {
    1000: b'\xfe\x01',
    800: b'\xfd\x02',
    500: b'\xfc\x03',
    250: b'\xfb\x04',
    125: b'\xfa\x05',
    100: b'\xf6\x09',
}


def print_param_offsets():
    offset = 0
    for fmt, name in PARAMETER_MAP:
        print(f'{offset:#02x}: {name}')
        offset += struct.calcsize(fmt)


class MessageError(Exception):
    """a received message was not as expected"""
    pass


class ModuleError(Exception):
    """the module did something unexpected"""
    pass


# MRS-used CAN IDs
ACK_ID = 0x1ffffff0
CMD_ID = 0x1ffffff1
RSP_ID = 0x1ffffff2
SREC_ID = 0x1ffffff3
DATA_ID = 0x1ffffff4
EEPROM_ID = 0x1ffffff5

# Firmware-specific messages
CONSOLE_ID = 0x1ffffffe

# Messages that we care about receiving
RECEIVE_FILTER = [
    ACK_ID, RSP_ID, DATA_ID, CONSOLE_ID
]


class TXMessage(object):
    """
    Abstract for messages that will be sent.

    Concrete classes set self._format, and pass corresponding
    arguments to __init__.
    """
    def __init__(self, message_id, *args):
        self.raw = can.Message(arbitration_id=message_id,
                               data=struct.pack(self._format, *args))

    def __str__(self):
        return f'{self.raw}'


class MSG_ping(TXMessage):
    """all-call message, solicits 'ack' from every module"""
    _format = '>H'

    def __init__(self):
        super().__init__(CMD_ID, 0)


class MSG_select(TXMessage):
    """selects a specific module for subsequent non-addressed commands"""
    _format = '>HI'

    def __init__(self, module_id):
        super().__init__(CMD_ID,
                         0x2010,
                         module_id)


class MSG_read_eeprom(TXMessage):
    """requests data from the EEPROM"""
    _format = '>HHB'

    def __init__(self, address, count):
        super().__init__(CMD_ID,
                         0x2003,
                         address,
                         count)


class MSG_write_eeprom(TXMessage):
    """enables writing to the EEPROM"""
    _format = '>H3c'

    def __init__(self):
        super().__init__(CMD_ID,
                         0x2011,
                         b'\xf3\x33\xaf')


class MSG_write_eeprom_data(object):
    """writes data to the EEPROM"""
    def __init__(self, address, data):
        payload = struct.pack('>H', address) + data
        self.raw = can.Message(arbitration_id=EEPROM_ID, data=payload)


class MSG_close_eeprom(TXMessage):
    """disable writing to the EEPROM"""
    _format = ">H"

    def __init__(self):
        super().__init__(CMD_ID, 0x2002)


class MSG_program(TXMessage):
    """commands the selected device to enter programming mode"""
    _format = '>H'

    def __init__(self):
        super().__init__(CMD_ID, 0x2000)


class MSG_erase(TXMessage):
    """commands the selected device to erase the flash"""
    _format = '>H'

    def __init__(self):
        super().__init__(CMD_ID, 0x0202)


class MSG_srecord(object):
    """raw S-record data"""
    def __init__(self, data):
        self.raw = can.Message(arbitration_id=SREC_ID, data=data)

    def __str__(self):
        return f"{self.raw}"


class RXMessage(object):
    """
    Abstract for messages that have been received.

    Concretes set self._format to struct.unpack() received bytes,
    and self._filter to a list of tuple-per-unpacked-item with each
    tuple containing True/False and, if True, the required value.
    """
    def __init__(self, expected_id, raw):
        self.raw = raw

        if raw.arbitration_id != expected_id:
            raise MessageError(f'expected reply with ID {expected_id} '
                               f'but got {raw}')
        expected_dlc = struct.calcsize(self._format)
        if raw.dlc != expected_dlc:
            raise MessageError(f'expected reply with length {expected_dlc} '
                               f'but got {raw}')

        self._values = struct.unpack(self._format, raw.data)
        for (index, (check, value)) in enumerate(self._filter):
            if check and value != self._values[index]:
                raise MessageError(f'reply field {index} is '
                                   f'0x{self._values[index]} '
                                   f'but expected {value}')

    @classmethod
    def len(self):
        return struct.calcsize(self._format)


class MSG_ack(RXMessage):
    """broadcast message sent by module on power-up, reboot or crash"""
    _format = '>BIBH'
    _filter = [(False, 0),
               (False, 0),
               (False, 0),
               (False, 0)]
    REASON_MAP = {
        0x00: 'power-on',
        0x01: 'reset',
        0x11: 'low-voltage reset',
        0x21: 'clock lost',
        0x31: 'address error',
        0x41: 'illegal opcode',
        0x51: 'watchdog timeout'
    }
    STATUS_MAP = {
        0: 'OK',
        4: 'NO PROG'
    }

    def __init__(self, raw):
        super().__init__(expected_id=ACK_ID,
                         raw=raw)
        (self.reason_code,
         self.module_id,
         self.status_code,
         self.sw_version) = self._values
        try:
            self.reason = self.REASON_MAP[self.reason_code]
        except KeyError:
            self.reason = 'unknown'
        try:
            self.status = self.STATUS_MAP[self.status_code]
        except KeyError:
            self.status = "unknown"


class MSG_selected(RXMessage):
    """
    Response to MSG_select confirming selection.

    self.sw_version appears to be 0 if the app is running,
    or !0 if in program mode
    """
    _format = '>HIH'
    _filter = [(True, 0x2110),
               (False, 0),
               (False, 0)]

    def __init__(self, raw):
        super().__init__(expected_id=RSP_ID,
                         raw=raw)
        (_, self.module_id, self.sw_version) = self._values


class MSG_eeprom_open(RXMessage):
    """
    Response to MSG_eeprom_write
    """
    _format = ">H3s"
    _filter = [(True, 0x2111),
               (True, b'\x01\x00\x00')]

    def __init__(self, raw):
        super().__init__(expected_id=RSP_ID,
                         raw=raw)


class MSG_eeprom_write_ok(RXMessage):
    """
    Response to MSG_eeprom_write_data
    """
    _format = ">H3s"
    _filter = [(True, 0x20e8),
               (True, b'\x00\x00\x00')]

    def __init__(self, raw):
        super().__init__(expected_id=RSP_ID,
                         raw=raw)


class MSG_eeprom_closed(RXMessage):
    """
    Response to MSG_close_eeprom
    """
    _format = ">H3s"
    _filter = [(True, 0x20f0),
               (True, b'\x02\x00\x00')]

    def __init__(self, raw):
        super().__init__(expected_id=RSP_ID,
                         raw=raw)


class MSG_program_nak(RXMessage):
    """
    Response sent to MSG_program when the app is running.
    Module reboots after sending this message (and sends MSG_ack
    with reason='reboot'), apparently into the bootloader.
    """
    _format = '>HIH'
    _filter = [(True, 0x2fff),
               (False, 0),
               (False, 0)]

    def __init__(self, raw):
        super().__init__(expected_id=RSP_ID,
                         raw=raw)
        (_, self.module_id, _) = self._values


class MSG_program_ack(RXMessage):
    """response sent to MSG_program when running the bootloader"""
    _format = '>HIH'
    _filter = [(True, 0x2100),
               (False, 0),
               (False, 0)]

    def __init__(self, raw):
        super().__init__(expected_id=RSP_ID,
                         raw=raw)
        (_, self.module_id, _) = self._values


class MSG_progress(RXMessage):
    """
    Sent in a stream after MSG_erase; self.progress counts from
    zero to self.limit.
    """
    _format = '>BBBB'
    _filter = [(True, 0),
               (False, 0),
               (False, 0),
               (True, 0)]

    def __init__(self, raw):
        super().__init__(expected_id=RSP_ID,
                         raw=raw)
        (_, self.progress, self.limit, _) = self._values


class MSG_erase_done(RXMessage):
    """sent after erase is completed"""
    _format = '>HBB'
    _filter = [(True, 0x0000),
               (False, 0x00),
               (True, 0x01)]

    def __init__(self, raw):
        super().__init__(expected_id=RSP_ID,
                         raw=raw)
        if self._values[1] not in [0, 0xff]:
            raise MessageError(f'unexpected data[2] {self._values[2]:#02x}')


class MSG_srec_start_ok(RXMessage):
    """sent in response to the first part of an S-record"""
    _format = '>5s'
    _filter = [(True, b'\x00\x01\x01\x01\x01')]

    def __init__(self, raw):
        super().__init__(expected_id=RSP_ID,
                         raw=raw)


class MSG_srec_cont_ok(RXMessage):
    """sent in response to an internal part of an S-record"""
    _format = '>2s'
    _filter = [(True, b'\x00\x01')]

    def __init__(self, raw):
        super().__init__(expected_id=RSP_ID,
                         raw=raw)


class MSG_srec_end_ok(RXMessage):
    """sent in response to an internal part of an S-record"""
    _format = '>3s'
    _filter = [(True, b'\x00\x00\x01')]

    def __init__(self, raw):
        super().__init__(expected_id=RSP_ID,
                         raw=raw)


class MSG_srecords_done(RXMessage):
    """sent in response to an S9 record at the end of the file"""
    _format = '>3s'
    _filter = [(True, b'\x00\x12\x34')]

    def __init__(self, raw):
        super().__init__(expected_id=RSP_ID,
                         raw=raw)


class MSG_no_program(RXMessage):
    """
    Sent after MSG_srecords_done if the ROM doesn't like the program,
    e.g. it doesn't have a reset vector.
    """
    _format = '>5s'
    _filter = [(True, b'\x00\x02\x02\x02\x02')]

    def __init__(self, raw):
        super().__init__(expected_id=RSP_ID,
                         raw=raw)


class ManualPower(object):
    def set_power_off(self):
        print('POWER: turn module power off')

    def set_power_t30(self):
        print('POWER: turn module power on (T30 only)')

    def set_power_t30_t15(self):
        print('POWER: turn module power on (T30 and T15)')


class AnaGatePower(object):
    def __init__(self, can_bus):
        self._connection = can_bus.connection

    def set_power_off(self):
        self._connection.set_analog_out(1, 0)
        self._connection.set_analog_out(2, 0)
        self._power_on = False

    def set_power_t30(self):
        self._connection.set_analog_out(1, 12000)
        self._connection.set_analog_out(2, 0)
        self._power_on = True

    def set_power_t30_t15(self):
        self._connection.set_analog_out(1, 12000)
        self._connection.set_analog_out(2, 12000)
        self._power_on = True


class Interface(object):
    def __init__(self, args):

        self._power_on = False
        self._verbose = args.verbose

        self._bus = can.Bus(interface=args.interface_name,
                            channel=args.interface_channel,
                            bitrate=args.bitrate * 1000)

        if args.interface_name == 'anagate':
            self._power_agent = AnaGatePower(self._bus)
        else:
            self._power_agent = ManualPower()

    def detect(self):
        """
        Power on the module and listen for it to sign on.
        Send it a select to keep it in the bootloader for a while.
        Returns the ID of the detected module.
        """
        self.set_power_off()
        time.sleep(0.25)
        self.drain()
        self.set_power_t30()
        while True:
            rsp = self.recv(5)
            if rsp is None:
                raise ModuleError('no power-on message from module')
            try:
                signon = MSG_ack(rsp)
                break
            except MessageError as e:
                raise ModuleError(f'unexpected power-on message from module: {rsp}')
        self.send(MSG_select(signon.module_id))
        rsp = self.recv(1)
        if rsp is None:
            raise ModuleError('no select response from module')
        try:
            signon = MSG_selected(rsp)
        except MessageError as e:
            raise ModuleError(f'unexpected select response from module : {rsp}')
        return signon.module_id

    def scan(self):
        """
        Send the all-call message and collect replies.

        We spam the message for a while at a short interval
        and collect / de-duplicate replies. If a module is in
        a crashloop it may not respond to the first ping, but
        in theory we'll catch it in the bootloader eventually.
        """
        print('Scanning...')
        modules = dict()
        scan_end_time = time.time() + 1.0
        self.send(MSG_ping())
        while True:
            rsp = self.recv(0.05)
            if rsp is not None:
                try:
                    ack = MSG_ack(rsp)
                except MessageError as e:
                    raise MessageError('unexpected programming traffic '
                                       'on CAN bus during scan')
                modules[ack.module_id] = {
                    'status': ack.status,
                    'reason': ack.reason,
                    'sw_ver': ack.sw_version
                }
            elif time.time() < scan_end_time:
                self.send(MSG_ping())
            else:
                break
        return modules

    def get_console_data(self):
        """fetch console packets"""
        while True:
            msg = self.recv(1)
            if msg is not None:
                try:
                    status = MSG_ack(msg)
                    print(f'module reset due to {status.reason}')
                except MessageError:
                    pass
                if msg.arbitration_id == CONSOLE_ID:
                    return msg.data

    def send(self, message):
        """send the message"""

        self._trace(f'CAN TX: {message}')
        self._bus.send(message.raw)

    def recv(self, timeout):
        """
        wait for a message
        """
        now = time.time()
        deadline = now + timeout
        while time.time() < deadline:
            msg = self._bus.recv(timeout=deadline - time.time())
            if msg is not None:
                if msg.arbitration_id in RECEIVE_FILTER:
                    return msg

    def set_power_off(self):
        self._power_agent.set_power_off()

    def set_power_t30(self):
        self._power_agent.set_power_t30()

    def set_power_t30_t15(self):
        self._power_agent.set_power_t30_t15()

    def drain(self):
        """
        Try to drain any buffered CAN messages - give up if the bus is
        too chatty.
        """
        count = 100
        while count:
            count -= 1
            msg = self.recv(0.25)
            if msg is None:
                break

    def _trace(self, msg):
        if self._verbose:
            print(msg)


class Module(object):
    def __init__(self, interface, module_id, args):
        self._interface = interface
        self._module_id = module_id
        self._verbose = args.verbose

#        if self.parameter('_ParameterMagic') != PARAMETER_MAGIC:
#            print(f'WARNING: EEPROM may be corrupted - bad magic number')

    def _cmd(self, message, reply_class=None):
        """send a message, wait for a response"""
        self._interface.send(message)
        rsp = self._interface.recv(1)
        if rsp is None:
            raise ModuleError(f'timed out waiting for a reply to {message} ')
        if reply_class is not None:
            try:
                repl = reply_class(rsp)
            except MessageError:
                raise ModuleError('unexpected reply type')
        return rsp

    def _select(self):
        """select the module for further commands"""
        rsp = self._cmd(MSG_select(self._module_id))
        sel = MSG_selected(rsp)
        if (sel.module_id != self._module_id):
            raise CanError('wrong module responded to selection')
        return sel.sw_version

    def _read_eeprom(self, address, length):
        """
        read bytes from the EEPROM

        Note no defined DATA_ID message as it has variable length, we just poke at the
        raw CAN message instead.
        """
        self._select()
        result = bytearray()
        while length > 0:
            amount = length if length <= 8 else 8
            rsp = self._cmd(MSG_read_eeprom(address, amount))
            if rsp.arbitration_id != DATA_ID:
                raise ModuleError(f'unexpected reply to EEPROM read {rsp}')
            length -= amount
            address += amount
            result += bytes(rsp.data)
        return result

    def _write_eeprom(self, address, data):
        """
        write bytes to the EEPROM
        """
        self._select()
        try:
            self._cmd(MSG_write_eeprom(), MSG_eeprom_open)
        except ModuleError:
            raise ModuleError('could not write to module EEPROM')
        while len(data) > 0:
            buf = data[:6]
            data = data[6:]
            try:
                self._cmd(MSG_write_eeprom_data(address, buf), MSG_eeprom_write_ok)
            except ModuleError:
                raise ModuleError('could not write EEPROM data')
        try:
            self._cmd(MSG_close_eeprom(), MSG_eeprom_closed)
        except ModuleError:
            raise ModuleError('error disabling EEPROM write mode')

    def _wait_for_boot(self, timeout):
        """wait for the message broadcast by a module rebooting"""
        while True:
            rsp = self._interface.recv(timeout)
            if rsp is None:
                raise ModuleError('timed out waiting for module reboot message')
            try:
                boot_message = MSG_ack(rsp)
                if boot_message.module_id != self._module_id:
                    continue
                if boot_message.reason != 'reboot':
                    continue
                break
            except MessageError:
                pass

    def _enter_flash_mode(self):
        """put the module into flash/erase mode"""
        self._select()
        rsp = self._cmd(MSG_program())
        try:
            will_reboot = MSG_program_nak(rsp)
            self._wait_for_boot(2)
            self._select()
            rsp = self._cmd(MSG_program())
        except MessageError:
            pass
        ready = MSG_program_ack(rsp)

    def _print_progress(self, title, limit, position):
        if limit < 1:
            limit = 1
        scale = 60 / limit
        if position > limit:
            position = limit
        hashes = int(position * scale)
        bar = '#' * hashes + '.' * (60 - hashes)
        print(f'\r{title:<8} [{bar}] {position}/{limit}', end='')

    def _erase(self):
        """erase the currently-selected module"""
        self._interface.send(MSG_erase())
        while True:
            rsp = self._interface.recv(2)
            if rsp is None:
                raise ModuleError('timed out waiting for module progress message')
            try:
                done = MSG_erase_done(rsp)
                print('')
                break
            except MessageError as e:
                pass
            try:
                progress = MSG_progress(rsp)
            except MessageError as e:
                raise ModuleError(f'got unexpected message {rsp} '
                                  f'instead of erase progress / completion')
            self._print_progress("ERASE", progress.limit - 1, progress.progress)

    def _program(self, srecords):
        """flash srecords to the currently-selected module"""
        progress = 1
        records = list(srecords.upload_records())
        memory_records = records[:-1]
        terminal_record = records[-1]

        # send memory records (S[13])
        #
        # expected response varies based on whether this is the first, 
        # intermediate or last fragment of a record.
        progress = 0
        progress_limit = len(memory_records) - 1
        for srec in memory_records:

            self._print_progress("FLASH", progress_limit, progress)
            progress += 1
            if self._verbose:
                print(f"PGM: {srec[0:2].decode()}{srec[2:].hex()}")

            # do we have enough to send an initial fragment?
            if len(srec) > 8:
                rsp = self._cmd(MSG_srecord(srec[:8]))
                srec = srec[8:]
                try:
                    ack = MSG_srec_start_ok(rsp)
                except MessageError:
                    raise ModuleError(f'unexpected response to S-record {rsp}')

            # do we have enough to send intermediate fragments?
            while len(srec) > 8:
                rsp = self._cmd(MSG_srecord(srec[:8]))
                srec = srec[8:]
                try:
                    ack = MSG_srec_cont_ok(rsp)
                except MessageError:
                    raise ModuleError(f'unexpected response to S-record {rsp}')

            # send the last fragment of the record
            rsp = self._cmd(MSG_srecord(srec[:8]))
            srec = srec[8:]
            try:
                ack = MSG_srec_end_ok(rsp)
            except MessageError:
                raise ModuleError(f'unexpected response to S-record {rsp}')

        # send the terminal record
        rsp = self._cmd(MSG_srecord(terminal_record))
        try:
            ack = MSG_srecords_done(rsp)
        except MessageError:
            raise ModuleError(f'unexpected response to terminal S-record {rsp}')
        print('')

        # The module will start running the program (or try at least). HCS08 modules
        # will reset; S32K modules just jump to the program, so the output from this
        # point on varies considerably.

    def _parameter(self, parameter_name):
        """look up a parameter by name"""

        # find it in the parameter map
        offset = 0
        for fmt, name in PARAMETER_MAP:
            if name == parameter_name:
                return offset, fmt
                address = offset
                break
            offset += struct.calcsize(fmt)
        raise RuntimeError(f'attempt to lookup non-existent parameter {parameter_name}')

    def parameter(self, parameter_name):
        address, fmt = self._parameter(parameter_name)
        # read it from the EEPROM and make it usable
        value = struct.unpack(fmt, self._read_eeprom(address, struct.calcsize(fmt)))
        if fmt[-1] == 's':
            value = value[0].decode('ascii')
        elif len(value) == 1:
            value = f'{value[0]:#x}'
        return value

    def set_parameter(self, parameter_name, value):
        if parameter_name not in PARAMETER_WRITE_OK:
            raise RuntimeError(f'parameter "{parameter_name}" not writable')
        address, fmt = self._parameter(parameter_name)
        if parameter_name == 'BaudrateBootloader1':
            try:
                self._write_eeprom(address, BAUD_MAP[value])
            except KeyError:
                raise RuntimeError(f'unsupported CAN baudrate {value}')
        elif fmt[-1] == 's':
            fmtlen = struct.calcsize(fmt)
            if len(value) > fmtlen:
                raise RuntimeError(f'value "{value}" too long, max {fmt} bytes')
            while len(value) < fmtlen:
                value += '\0'
            self._write_eeprom(address, value.encode('ascii'))
        else:
            raise RuntimeError(f'writing {parameter_name} not supported')

    @property
    def parameter_names(self):
        """generator yielding valid parameter names"""
        for (_, name) in PARAMETER_MAP:
            # ignore hidden names
            if name[0] != '_':
                yield name

    def upload(self, srecords):
        """flash the module with the supplied program"""
        self._enter_flash_mode()
        self._erase()
        self._program(srecords)

    def erase(self):
        """erase the module"""
        self._enter_flash_mode()
        self._erase()

    def x(self):
        """hacking"""
        while True:
            self._interface.send(MSG_ping())
            rsp = self._interface.recv(0.02)
