#!/usr/bin/env python3

import argparse
import logging
import time
import sys
import struct
import enum
import functools
import threading
from pyModbusTCP.client import ModbusClient

# --- Helper Functions (pack, unpack, explode_bits, implode_bits - unchanged) ---
def pack(pattern, x):
    num_bytes = struct.calcsize(pattern)
    num_words = num_bytes // 2
    if num_bytes % 2 != 0 or num_words == 0:
        raise ValueError("Pattern must represent an even number of bytes >= 2")
    packed_bytes = struct.pack(f'>{pattern}', x)
    return struct.unpack(f'>{num_words}H', packed_bytes)

def unpack(pattern, *words):
    num_words = len(words)
    if num_words == 0:
        raise ValueError("No words provided to unpack")
    packed_bytes = struct.pack(f'>{num_words}H', *words)
    values = struct.unpack(f'>{pattern}', packed_bytes)
    return values[0]

def explode_bits(x, n):
    return [ (x >> i) & 1 for i in range(n) ]

def implode_bits(bits):
    n = len(bits)
    val = 0
    for i in range(n):
        if bits[i]:
            val |= (1 << i)
    return val

# --- Register Class (with description) and Enum (unchanged) ---
class Register:
    def __init__(self, name, num, decode=None, encode=None, read_only=False, description=None, value_map=None):
        self.name, self.num = name, num
        self.decode = decode or functools.partial(unpack, 'l')
        self.encode = encode or functools.partial(pack, 'l')
        self.read_only = read_only
        self.description = description
        self.value_map = value_map

    def __eq__(self, other):
        if isinstance(other, str): return self.name.lower() == other.lower()
        elif isinstance(other, Register): return self.num == other.num
        return NotImplemented
    def __hash__(self): return hash(self.num)
    def __repr__(self): return f'{self.__class__.__name__}({repr(self.name)}, {self.num})'
    @property
    def addr(self): return (2*self.num, 2*self.num+1)
    @property
    def num_words(self): return 2
    @property
    def size_bits(self): return self.num_words * 16
    @property
    def rw_status(self): return "RO" if self.read_only else "RW"

class MODE(enum.IntEnum):
    PASSIVE = 0; VELOCITY = 1; POSITION = 2; GEAR = 3; TORQUE = 4
    ANALOG_VELOCITY = 5; ANALOG_VELOCITY_GEAR = 6; MANUAL_CURRENT = 7
    STEP_RESPONSE_TEST = 8; INTERNAL_TEST_9 = 9; BRAKE = 10; STOP = 11
    TORQUE_HOMING = 12; SENSOR1_HOMING = 13; SENSOR2_HOMING = 14
    SAFE_MODE = 15; ANALOG_VELOCITY_DEADBAND = 16
    VELOCITY_LIMITED_ANALOG_TORQUE = 17; ANALOG_GEAR = 18; COIL = 19
    ANALOG_BI_POSITION = 20; ANALOG_TO_POSITION = 21
    INTERNAL_TEST_22 = 22; INTERNAL_TEST_23 = 23; GEAR_FOLLOW = 24
    IHOME = 25; IIHOME = 26

ERR_STAT_BITS = {
    0: "I2T_ERR", 1: "FLW_ERR", 2: "FNC_ERR", 3: "UIT_ERR", 4: "IN_POS",
    5: "ACC_FLAG", 6: "DEC_FLAG", 7: "PLIM_ERR", 8: "DEGC_ERR", 9: "UV_ERR",
    10:"UV_DETECT", 11:"OV_ERR", 12:"IPEAK_ERR", 13:"SPEED_ERR",
    14:"DIS_P_LIM", 15:"INDEX_ERR", 16:"OLDFILTERR", 17:"U24V_ERR",
    18:"SHORT_CIRC", 19:"VAC_ON", 20:"PWM_LOCKED", 21:"COMM_ERR",
    22:"CURLOOP_ERR", 23:"SLAVE_ERR", 24:"ANY_ERR", 25:"INIT_ERR",
    26:"FLASH_ERR", 27:"STO_ALARM_ERR", 28:"FPGA_ERROR", 30:"OUT1_STATUS", 31:"OUT2_STATUS"
}
CNTRL_BITS_MAP = {
     0:"USRINTF0", 1:"USRINTF1", 2:"PULSEDIR", 3:"INPSIGN", 4:"HICLK", 5:"HALL_INT",
     6:"RECORDBIT", 7:"REWINDBIT", 8:"RECINNERBIT", 9:"AUTO_RESYNC", 10:"MAN_RESYNC",
    11:"INDEX_HOME", 12:"REL_RESYNC", 13:"HALL_C", 14:"HALL_B", 15:"HALL_A"
}

all_registers_defs = [
    Register(name='PROG_VERSION',    num=1, read_only=True, description="Firmware version info"),
    Register(name='MODE_REG',        num=2, decode=lambda hi, lo: unpack('l', hi, lo), encode=lambda x: pack('l', x.value if isinstance(x, MODE) else int(x)), description="Operating Mode", value_map=MODE),
    Register(name='P_SOLL',          num=3, decode=functools.partial(unpack, 'l'), encode=functools.partial(pack, 'l'), description="Target Position (counts)"),
    Register(name='P_NEW',           num=4, decode=functools.partial(unpack, 'l'), encode=functools.partial(pack, 'l'), description="New Position for atomic update"),
    Register(name='V_SOLL',          num=5, decode=lambda hi, lo: unpack('l', hi, lo) / 2.77056, encode=lambda x: pack('l', int(x * 2.77056)), description="Target Velocity (RPM)"),
    Register(name='A_SOLL',          num=6, decode=lambda hi, lo: unpack('l', hi, lo) / 3.598133e-3, encode=lambda x: pack('l', int(x * 3.598133e-3)), description="Target Acceleration (RPM/s)"),
    Register(name='T_SOLL',          num=7, decode=lambda hi, lo: unpack('l', hi, lo) * 300 / 1023, encode=lambda x: pack('l', int(x * 1023 / 300)), description="Target Torque (% peak)"),
    Register(name='P_FNC_LO',        num=8, decode=functools.partial(unpack, 'l'), encode=functools.partial(pack, 'l'), description="Internal Function Pos (Low Word)"),
    Register(name='INDEX_OFFSET',    num=9, decode=functools.partial(unpack, 'l'), encode=functools.partial(pack, 'l'), description="Internal Function Pos (High Word) / Index Offset"),
    Register(name='P_IST',           num=10, decode=functools.partial(unpack, 'l'), read_only=True, description="Actual Position (counts)"),
    Register(name='V_IST_16',        num=11, decode=lambda hi, lo: unpack('l', hi, lo) / 2.77056, read_only=True, description="Actual Velocity (RPM, 16 sample avg)"),
    Register(name='V_IST',           num=12, decode=lambda hi, lo: unpack('l', hi, lo) / 2.77056, read_only=True, description="Actual Velocity (RPM, instant)"),
    Register(name='KVOUT',           num=13, decode=functools.partial(unpack, 'L'), encode=functools.partial(pack, 'L'), description="Load Factor / Velocity Gain"),
    Register(name='GEARF1',          num=14, decode=functools.partial(unpack, 'L'), encode=functools.partial(pack, 'L'), description="Gear Factor Numerator"),
    Register(name='GEARF2',          num=15, decode=functools.partial(unpack, 'L'), encode=functools.partial(pack, 'L'), description="Gear Factor Denominator"),
    Register(name='I2T',             num=16, decode=functools.partial(unpack, 'L'), read_only=True, description="Motor thermal load integral"),
    Register(name='I2TLIM',          num=17, decode=functools.partial(unpack, 'L'), encode=functools.partial(pack, 'L'), description="Motor thermal load limit"),
    Register(name='UIT',             num=18, decode=functools.partial(unpack, 'L'), read_only=True, description="Power dump thermal load integral"),
    Register(name='UITLIM',          num=19, decode=functools.partial(unpack, 'L'), encode=functools.partial(pack, 'L'), description="Power dump thermal load limit"),
    Register(name='FLWERR',          num=20, decode=functools.partial(unpack, 'l'), read_only=True, description="Following Error (counts)"),
    Register(name='U_24V',           num=21, decode=lambda hi, lo: unpack('L', hi, lo) * 0.01, read_only=True, description="Control Voltage (V, approx)"),
    Register(name='FLWERRMAX',       num=22, decode=functools.partial(unpack, 'L'), encode=functools.partial(pack, 'L'), description="Max Following Error Limit (counts)"),
    Register(name='UV_HANDLE',       num=23, decode=functools.partial(unpack, 'L'), encode=functools.partial(pack, 'L'), description="Undervoltage Handling Config"),
    Register(name='FNCERR',          num=24, decode=functools.partial(unpack, 'l'), read_only=True, description="Function Error"),
    Register(name='P_IST_TURNTAB',   num=25, decode=functools.partial(unpack, 'l'), read_only=True, description="Actual Turntable Position"),
    Register(name='FNCERRMAX',       num=26, decode=functools.partial(unpack, 'L'), encode=functools.partial(pack, 'L'), description="Max Function Error Limit"),
    Register(name='TURNTAB_COUNT',   num=27, decode=functools.partial(unpack, 'l'), read_only=True, description="Turntable Wrap Count"),
    Register(name='MIN_P_IST',       num=28, decode=functools.partial(unpack, 'l'), encode=functools.partial(pack, 'l'), description="Min Software Position Limit"),
    Register(name='DEGC',            num=29, decode=lambda hi, lo: unpack('l', hi, lo) * 0.1221, read_only=True, description="Internal Temperature (DegC, approx)"),
    Register(name='MAX_P_IST',       num=30, decode=functools.partial(unpack, 'l'), encode=functools.partial(pack, 'l'), description="Max Software Position Limit"),
    Register(name='DEGCMAX',         num=31, decode=functools.partial(unpack, 'L'), encode=functools.partial(pack, 'L'), description="Max Temperature Limit"),
    Register(name='ACC_EMERG',       num=32, decode=functools.partial(unpack, 'L'), encode=functools.partial(pack, 'L'), description="Emergency Acceleration"),
    Register(name='INPOSWIN',        num=33, decode=functools.partial(unpack, 'L'), encode=functools.partial(pack, 'L'), description="In Position Window Size (counts)"),
    Register(name='INPOSCNT',        num=34, decode=functools.partial(unpack, 'L'), encode=functools.partial(pack, 'L'), description="In Position Sample Count"),
    Register(name='ERR_STAT',        num=35, decode=lambda hi, lo: unpack('L', hi, lo), encode=lambda x: pack('L', implode_bits(x) if isinstance(x, list) else int(x)), description="Error/Status Bits", value_map=ERR_STAT_BITS),
    Register(name='CNTRL_BITS',      num=36, decode=lambda hi, lo: unpack('L', hi, lo), encode=lambda x: pack('L', implode_bits(x) if isinstance(x, list) else int(x)), description="Control Bits", value_map=CNTRL_BITS_MAP),
    Register(name='START_MODE',      num=37, decode=functools.partial(unpack, 'L'), encode=functools.partial(pack, 'L'), description="Start Mode Config"),
    Register(name='P_HOME',          num=38, decode=functools.partial(unpack, 'l'), encode=functools.partial(pack, 'l'), description="Homing Position Offset"),
    Register(name='HW_SETUP',        num=39, decode=functools.partial(unpack, 'L'), encode=functools.partial(pack, 'L'), description="Hardware Setup Bits"),
    Register(name='V_HOME',          num=40, decode=functools.partial(unpack, 'l'), encode=functools.partial(pack, 'l'), description="Homing Velocity"),
    Register(name='T_HOME',          num=41, decode=functools.partial(unpack, 'l'), encode=functools.partial(pack, 'l'), description="Homing Torque"),
    Register(name='HOME_MODE',       num=42, decode=functools.partial(unpack, 'L'), encode=functools.partial(pack, 'L'), description="Homing Mode Type"),
    Register(name='COMMAND_REG',     num=211, decode=functools.partial(unpack, 'L'), encode=functools.partial(pack, 'L'), description="Special Command Register"),
    Register(name='U_BUS',           num=198, decode=lambda hi, lo: unpack('L', hi, lo) * 0.888, read_only=True, description="DC Bus Voltage (V)"),
]

REGISTERS = { reg.name.upper(): reg for reg in all_registers_defs }

CMD_RESET = 1
CMD_SAVE_IN_FLASH = 2

def setup_logging(level=logging.INFO):
    logging.basicConfig(level=level, format='%(asctime)s - %(levelname)s - %(message)s')

def get_register_info(reg_id):
    if isinstance(reg_id, str):
        reg_id_upper = reg_id.upper()
        if reg_id_upper in REGISTERS: return REGISTERS[reg_id_upper]
        else:
            try:
                addr_num = int(reg_id)
                for reg in REGISTERS.values():
                    if reg.num == addr_num: return reg
                logging.error(f"No register found: {reg_id}"); return None
            except ValueError: logging.error(f"Unknown register: {reg_id}"); return None
    elif isinstance(reg_id, int):
        for reg in REGISTERS.values():
            if reg.num == reg_id: return reg
        logging.error(f"No register found: {reg_id}"); return None
    else: logging.error(f"Invalid ID type: {type(reg_id)}"); return None

def decode_bitfield(value, bit_map):
    set_bits = []
    for bit_num in range(32):
        if (value >> bit_num) & 1:
            description = bit_map.get(bit_num, f"Bit {bit_num}")
            set_bits.append(description)
    return set_bits if set_bits else ["None_Set"] # Changed "None" to "None_Set"

# --- Command Handlers (Corrected error access) ---
def handle_read(client, args):
    logging.info(f"Read register(s): {args.register}")
    for reg_id in args.register:
        reg = get_register_info(reg_id)
        if not reg: continue
        modbus_addr, num_modbus_words = reg.addr[0], reg.num_words
        try:
            read_words = client.read_holding_registers(modbus_addr, num_modbus_words)
            if read_words is not None:
                value = reg.decode(*read_words)
                raw_hex = [f"{w:04X}" for w in read_words]
                meaning_str = ""
                if reg.value_map:
                    if isinstance(reg.value_map, enum.EnumMeta):
                        try: meaning_str = f" ({reg.value_map(value).name})"
                        except ValueError: meaning_str = " (Unknown Enum Value)"
                    elif isinstance(reg.value_map, dict):
                        bits = decode_bitfield(value, reg.value_map)
                        meaning_str = f" (Bits: {', '.join(bits)})"
                logging.info(f"Read {reg.name}({reg.num}): Val={value}{meaning_str}, Raw={read_words}(Hex:{raw_hex})")
            else:
                logging.error(f"Failed read {reg.name}. Modbus Error Code: {client.last_error}") # Corrected: no ()
        except Exception as e: logging.error(f"Error reading {reg.name}: {e}", exc_info=True)

def handle_write(client, args):
    logging.info(f"Write '{args.value}' to {args.register}")
    reg = get_register_info(args.register)
    if not reg: return
    if reg.read_only: logging.error(f"{reg.name} is RO."); return
    addr = reg.addr[0]
    try:
        try: val_enc = float(args.value)
        except ValueError: val_enc = int(args.value)
        words = reg.encode(val_enc)
        logging.debug(f"Write Addr:{addr}, Words:{words}, Val:{val_enc}")
        success = client.write_multiple_registers(addr, words)
        if success:
            logging.info(f"Success writing '{args.value}' to {reg.name}")
        else:
            logging.error(f"Failed write {reg.name}. Modbus Error Code: {client.last_error}") # Corrected: no ()
    except ValueError: logging.error(f"Invalid value: {args.value}.")
    except Exception as e: logging.error(f"Error write {reg.name}: {e}", exc_info=True)

def handle_mode(client, args):
    mode_upper = args.mode_name.upper()
    if mode_upper in MODE.__members__:
        mode_enum = MODE[mode_upper]
        logging.info(f"Set mode {mode_upper}({mode_enum.value})")
        args.register = 'MODE_REG'; args.value = mode_enum.value
        handle_write(client, args)
    else:
        logging.error(f"Unknown mode: {args.mode_name}. Available: {', '.join(MODE.__members__.keys())}")

def handle_watch(client, args):
    logging.info(f"Watch: {args.register} every {args.rate}s. Ctrl+C exit.")
    regs_watch = [get_register_info(r) for r in args.register if get_register_info(r)]
    if not regs_watch: logging.error("No valid registers."); return
    try:
        while True:
            print("---")
            for reg in regs_watch:
                addr, num_words = reg.addr[0], reg.num_words
                try:
                    read_words = client.read_holding_registers(addr, num_words)
                    if read_words is not None:
                        value = reg.decode(*read_words)
                        meaning_str = ""
                        if reg.value_map:
                            if isinstance(reg.value_map, enum.EnumMeta):
                                try: meaning_str = f" ({reg.value_map(value).name})"
                                except ValueError: meaning_str = " (Unknown Enum Val)"
                            elif isinstance(reg.value_map, dict):
                                bits = decode_bitfield(value, reg.value_map)
                                meaning_str = f" (Bits: {', '.join(bits)})"
                        print(f"{reg.name}({reg.num}): {value}{meaning_str}")
                    else:
                        print(f"{reg.name}({reg.num}): FAILED (Modbus Err: {client.last_error})") # Corrected: no ()
                except Exception as e:
                    print(f"{reg.name}({reg.num}): COMM ERROR ({e})")
            time.sleep(args.rate)
    except KeyboardInterrupt: logging.info("Stopping watch.")

def handle_reset(client, args):
    logging.warning("RESET (Verify CMD_REG/CMD_RESET values!)")
    if 'COMMAND_REG' not in REGISTERS: logging.error("COMMAND_REG undef!"); return
    args.register = 'COMMAND_REG'; args.value = CMD_RESET
    handle_write(client, args)

def handle_save(client, args):
    logging.warning("SAVE FLASH (Verify CMD_REG/CMD_SAVE values!)")
    if 'COMMAND_REG' not in REGISTERS: logging.error("COMMAND_REG undef!"); return
    args.register = 'COMMAND_REG'; args.value = CMD_SAVE_IN_FLASH
    handle_write(client, args)

def handle_list(args):
    print(f"\nDefined Registers (Filter: {args.filter or 'None'}):")
    print("-" * 90)
    print(f"{'Num':<5} {'Name':<20} {'Size':<7} {'R/W':<4} {'Mapped':<7} {'Description'}")
    print("-" * 90)
    sorted_regs = sorted(all_registers_defs, key=lambda r: r.num)
    count = 0
    for reg in sorted_regs:
        if args.filter:
            filter_term = args.filter.lower()
            if filter_term not in reg.name.lower() and filter_term != str(reg.num):
                continue
        size_str = f"{reg.size_bits}-bit"; mapped_str = "Yes" if reg.value_map else "No"; desc = reg.description or ""
        print(f"{reg.num:<5} {reg.name:<20} {size_str:<7} {reg.rw_status:<4} {mapped_str:<7} {desc}")
        count += 1
    print("-" * 90); print(f"Total matching: {count}")

# --- Main Execution (Unchanged) ---
def main():
    parser = argparse.ArgumentParser(description="JVL Motor Config Utility (Modbus TCP)")
    parser.add_argument("-i", "--ip-address", help="IP of motor/gateway (required for most commands)")
    parser.add_argument("-p", "--port", type=int, default=502, help="Port (def: 502)")
    parser.add_argument("-u", "--unit-id", type=int, default=1, help="Modbus Unit ID (def: 1)")
    parser.add_argument("-v", "--verbose", action='store_const', const=logging.DEBUG, default=logging.INFO, help="Debug logging")

    subparsers = parser.add_subparsers(dest='command', help='Commands', required=True)

    parser_read = subparsers.add_parser('read', help='Read register(s)')
    parser_read.add_argument('register', nargs='+', help='Name(s) or JVL number(s)')
    parser_read.set_defaults(func=handle_read)
    parser_write = subparsers.add_parser('write', help='Write register')
    parser_write.add_argument('register', help='Name or JVL number')
    parser_write.add_argument('value', help='Value')
    parser_write.set_defaults(func=handle_write)
    parser_mode = subparsers.add_parser('mode', help='Set mode')
    parser_mode.add_argument('mode_name', help=f"Mode ({', '.join(MODE.__members__.keys())})")
    parser_mode.set_defaults(func=handle_mode)
    parser_watch = subparsers.add_parser('watch', help='Monitor registers')
    parser_watch.add_argument('register', nargs='+', help='Name(s) or JVL number(s)')
    parser_watch.add_argument('--rate', type=float, default=1.0, help='Rate (s, def: 1.0)')
    parser_watch.set_defaults(func=handle_watch)
    parser_reset = subparsers.add_parser('reset', help='Send reset (VERIFY!)')
    parser_reset.set_defaults(func=handle_reset)
    parser_save = subparsers.add_parser('save', help='Send save (VERIFY!)')
    parser_save.set_defaults(func=handle_save)
    parser_list = subparsers.add_parser('list', help='List defined registers')
    parser_list.add_argument('filter', nargs='?', default=None, help='Optional filter string')
    parser_list.set_defaults(func=handle_list)

    args = parser.parse_args()
    setup_logging(args.verbose)

    if args.command == 'list':
        handle_list(args)
        sys.exit(0)

    if not args.ip_address:
        parser.error("The --ip-address argument is required for commands other than 'list'")

    logging.info(f"Connecting to {args.ip_address}:{args.port} (Unit ID: {args.unit_id})")
    client = ModbusClient(host=args.ip_address, port=args.port, unit_id=args.unit_id,
                          auto_open=True, auto_close=True,
                          timeout=5.0)
    client.lock = threading.Lock()

    if not client.is_open:
        if not client.open():
            logging.error(f"Failed connection to {args.ip_address}:{args.port}.")
            sys.exit(1)

    logging.info("Connection successful.")

    try:
        with client.lock:
            args.func(client, args)
    except AttributeError: logging.error("Invalid command."); parser.print_help()
    except Exception as e: logging.error(f"Unexpected error: {e}", exc_info=True)
    finally: logging.info("Operation finished.")

if __name__ == "__main__":
    main()
