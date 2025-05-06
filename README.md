# JVL Motor Configuration Utility (via Modbus TCP)

**⚠️ Experimental Code - Use at Your Own Risk! ⚠️**

This Python script is a general-purpose command-line interface (CLI) for interacting with JVL motors (tested with MAC400) that support Modbus TCP communication (directly or via a gateway). It allows reading and writing motor registers, setting operating modes, and monitoring register values.

**This code was generated with the assistance of an AI model and is provided as-is. It has undergone some testing but may contain bugs or unexpected behavior. Always verify register addresses, command values, and motor behavior against the official JVL documentation for your specific motor model and firmware before performing any write operations or critical configurations. Incorrect use could lead to unexpected motor behavior, equipment damage, or safety hazards.**


## Prerequisites

*   Python 3.6+
*   `pyModbusTCP` library: Install using `pip install pyModbusTCP`

## Usage


**General Syntax:**

```bash
python jvl_util.py [global options] <command> [command-specific arguments]
```

**Global Options:**

*   `-i IP_ADDRESS`, `--ip-address IP_ADDRESS`:
    The IP address of the JVL motor or the Modbus TCP gateway it's connected to. This is **required** for all commands except `list`.
*   `-p PORT`, `--port PORT`:
    The Modbus TCP port number. (Default: 502)
*   `-u UNIT_ID`, `--unit-id UNIT_ID`:
    The Modbus Unit ID (slave ID) of the motor. (Default: 1)
*   `-v`, `--verbose`:
    Enable detailed debug logging for troubleshooting.

**Commands:**

### `list`

Lists all registers defined within the script, along with their JVL number, name, size, R/W status, and a brief description.

**Syntax:**

```bash
python jvl_util.py list [filter_string]
```

*   `filter_string` (optional): A case-insensitive string to filter the register list by name or JVL number.

**Examples:**

```bash
python jvl_util.py list
python jvl_util.py list MODE
python jvl_util.py list ERR_STAT
python jvl_util.py list 35
```

### `read`

Reads the current value(s) from one or more specified registers.

**Syntax:**

```bash
python jvl_util.py -i <ip_address> read <register_name_or_num> [register_name_or_num_2 ...]
```

**Examples:**

```bash
python jvl_util.py -i 192.168.1.10 read MODE_REG
python jvl_util.py -i 192.168.1.10 read P_IST V_IST_16 ERR_STAT
python jvl_util.py -i 192.168.1.10 read 35 # Read register with JVL number 35 (ERR_STAT)
```
The output will show the raw numerical value and, if a `value_map` is defined for the register (like for `MODE_REG` or `ERR_STAT`), an interpretation of that value (e.g., mode name or active error bits).

### `write`

Writes a specified value to a single motor register.

**⚠️ Use with extreme caution, especially when writing to configuration registers. Verify values with the JVL manual. ⚠️**

**Syntax:**

```bash
python jvl_util.py -i <ip_address> write <register_name_or_num> <value>
```

**Examples:**

```bash
# Write raw value 1 to MODE_REG (JVL Reg 2) - equivalent to 'mode VELOCITY'
python jvl_util.py -i 192.168.1.10 write MODE_REG 1
```

### `mode`

Sets the motor to a predefined operating mode. This is a convenience wrapper around writing to `MODE_REG`.

**Syntax:**

```bash
python jvl_util.py -i <ip_address> mode <MODE_NAME>
```

*   `<MODE_NAME>`: One of the modes defined in the script (e.g., `PASSIVE`, `VELOCITY`, `POSITION`). Case-insensitive.

**Example:**

```bash
python jvl_util.py -i 192.168.1.10 mode VELOCITY
python jvl_util.py -i 192.168.1.10 mode PASSIVE
```

### `watch`

Continuously polls and displays the values of specified registers. Press `Ctrl+C` to stop.

**Syntax:**

```bash
python jvl_util.py -i <ip_address> watch <register_name_or_num> [register_name_or_num_2 ...] [--rate <seconds>]
```

*   `--rate <seconds>` (optional): The polling interval in seconds. (Default: 1.0)

**Examples:**

```bash
python jvl_util.py -i 192.168.1.10 watch P_IST V_IST_16 ERR_STAT
python jvl_util.py -i 192.168.1.10 watch MODE_REG --rate 0.2
```

### `reset`

**⚠️ Experimental - Verify Command Register and Value! ⚠️**
Sends a "Reset Motor" command. The specific Modbus register and value for this command **MUST** be verified in your JVL motor's technical manual. The script uses placeholders.

**Syntax:**

```bash
python jvl_util.py -i <ip_address> reset
```

### `save`

**⚠️ Experimental - Verify Command Register and Value! ⚠️**
Sends a "Save Configuration to Flash" command. The specific Modbus register and value **MUST** be verified. Saving to flash has a limited number of write cycles.

**Syntax:**

```bash
python jvl_util.py -i <ip_address> save
```

## Register Definitions

The script contains a dictionary (`REGISTERS`) defining known JVL registers. This includes:

*   **JVL Register Number:** The numerical identifier from the manual.
*   **Name:** A human-readable name.
*   **Encode/Decode Functions:** Custom functions for scaling values (e.g., RPM, percentages) or interpreting bitfields.
*   **Read-Only Status:** Indicates if a register can be written to.
*   **Description:** A brief explanation of the register's purpose.
*   **Value Map:** For registers like `MODE_REG` or `ERR_STAT`, this provides a mapping from raw values to their meanings (e.g., enum names or active bits).

You can extend this dictionary in the script to support more registers specific to your motor and application. Refer to the JVL MAC motor user manual (pages 406+ for MAC050-141, pages 416+ for MAC400-4500) for detailed register lists.

## Important Considerations

*   **Modbus Endianness:** The script assumes JVL motors use Big Endian byte order for multi-word (32-bit) Modbus registers. This is typical but verify if you encounter unexpected 32-bit values.
*   **Scaling Factors:** Many registers (like `V_SOLL`, `A_SOLL`, `U_BUS`, `DEGC`) require scaling to convert between raw Modbus values and engineering units (RPM, V, °C). The script includes scaling factors based on the provided driver snippet; **these may need adjustment for your specific motor model or if the internal scaling of the motor/gateway differs.**
*   **Command Register (CMD_REG):** The address and values for special commands like `reset` and `save` are critical. **The current values (Reg 211, values 1 and 2) are placeholders and MUST be verified from your motor's specific documentation.**


## Disclaimer

This script is provided for utility and educational purposes. The author(s) are not responsible for any damage to equipment or other issues that may arise from its use. Always test thoroughly in a non-critical environment first.
