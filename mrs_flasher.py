#!python3
#
# Flash programmer for various MRS devices.
#
# Should work with any CAN interface supported by Python-CAN,
# but for best results use with an adapter or secondary device
# that can also control power to the module.
#
# The current implementation supports the AnaGate CAN X* modules;
# see the ManualPower and AnaGatePower classes for examples.
#
# Unlike the MRS flashers which depend on the application
# participating in the reboot-to-flash process, this script
# captures the module in the bootloader immediately out of reset,
# and so it works even if the application is bad.
#

import argparse
import time
import rich
from pathlib import Path
from mrs_srecord import S32K_Srecords, HCS08_Srecords
from mrs_bl_protocol import Interface, Module, ModuleError, MessageError


def do_upload(module, args):
    """implement the --upload option"""

    # detect module type, handle Srecords appropriately
    mcu_type = module.parameter('MCUType')
    if mcu_type == '0x1':
        srecords = HCS08_Srecords(args.upload, args)
    elif mcu_type in ['0x6', '0x8']:
        srecords = S32K_Srecords(args.upload, args, int(mcu_type, 0))
    else:
        raise RuntimeError(f'Unsupported module MCU {mcu_type}')

    module.upload(srecords)


def do_console(module, args):
    """implement the --console option"""
    line = ''
    while True:
        fragment = module.get_console_data()
        line += bytes(fragment).decode()
        if line.endswith('\0'):
            print(line)
            line = ''


def do_erase(module, args):
    """implement the --erase option"""
    module.erase()


def do_print_parameters(module, args):
    """implement the --print-module_parameters option"""
    for name in module.parameter_names:
        print(f'{name:<30} {module.parameter(name)}')


def do_set_bootloader_bitrate(module, args):
    module.set_parameter('BaudrateBootloader1', args.set_bootloader_bitrate)


def do_print_hcs08_srecords(srec_file, args):
    srecords = HCS08_Srecords(srec_file, args)
    for srec in srecords.text_records():
        if args.crlf:
            srec += '\r'
        print(srec)


def do_print_s32k_srecords(srec_file, args):
    srecords = S32K_Srecords(srec_file, args, 6)
    for srec in srecords.text_records():
        if args.crlf:
            srec += '\r'
        print(srec)


parser = argparse.ArgumentParser(description='MRS Microplex 7* and CC16 CAN flasher')
parser.add_argument('--interface-name',
                    type=str,
                    metavar='INTERFACE',
                    required=True,
                    help='name of the interface as known to python-can')
parser.add_argument('--interface-channel',
                    type=str,
                    metavar='CHANNEL',
                    required=True,
                    help='interface channel name (e.g. for Anagate units, hostname:portname')
parser.add_argument('--bitrate',
                    type=int,
                    default=500,
                    metavar='BITRATE_KBPS',
                    help='CAN bitrate (kBps')
parser.add_argument('--console-after-upload',
                    action='store_true',
                    help='monitor console messages after upload')
parser.add_argument('--power-cycle-after-upload',
                    action='store_true',
                    help='cycle power and leave KL30 on after upload')
parser.add_argument('--kl15-after-upload',
                    action='store_true',
                    help='turn KL15 on after upload')
parser.add_argument('--no-power-off',
                    action='store_true',
                    help='do not turn power off at exit')
parser.add_argument('--crlf',
                    action='store_true',
                    help='print S-records with Windows-style line endings')
parser.add_argument('--verbose',
                    action='store_true',
                    help='print verbose progress information')

actiongroup = parser.add_mutually_exclusive_group(required=True)
actiongroup.add_argument('--upload',
                         type=Path,
                         metavar='SRECORD_FILE',
                         help='S-record file to upload')
actiongroup.add_argument('--erase',
                         action='store_true',
                         help='erase the program')
actiongroup.add_argument('--console',
                         action='store_true',
                         help='turn on module power and monitor the console')
actiongroup.add_argument('--print-module-parameters',
                         action='store_true',
                         help='print all module parameters')
actiongroup.add_argument('--set-bootloader-can-bitrate',
                         type=int,
                         metavar='CAN_BITRATE_KBPS',
                         help='set bootloader CAN bitrate, takes effect after reset')
actiongroup.add_argument('--set-module-name',
                         type=str,
                         metavar='MODULE_NAME',
                         help='set EEPROM module name')
actiongroup.add_argument('--set-software-version',
                         type=str,
                         metavar='SOFTWARE_VERSION',
                         help='set EEPROM software version')

args = parser.parse_args()
# find and connect to a module
try:
    interface = Interface(args)
    module_id = interface.detect()
    module = Module(interface, module_id, args)

    # Upload S-records
    if args.upload is not None:
        do_upload(module, args)
        if args.power_cycle_after_upload:
            interface.set_power_off()
            time.sleep(0.25)
            interface.set_power_t30_t15()

        if args.console_after_upload:
            do_console(interface, args)

    # Erase the module
    if args.erase:
        do_erase(module, args)

    # Print module parameters
    if args.print_module_parameters:
        do_print_parameters(module, args)

    # Set the bootloader CAN bitrate
    if args.set_bootloader_can_bitrate is not None:
        do_set_bootloader_bitrate(module, args)

    # Reset the module and run the console
    # If we don't reset, it will sit for a (long) while in the bootloader
    # after detection before timing out and starting the app. This is faster.
    if args.console:
        interface.set_power_off()
        time.sleep(0.25)
        interface.set_power_t30_t15()
        do_console(interface, args)

except KeyboardInterrupt:
    pass
except (ModuleError, MessageError) as e:
    print(f'ERROR: {e}')

if not args.no_power_off:
    interface.set_power_off()
