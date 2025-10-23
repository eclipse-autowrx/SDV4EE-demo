# Copyright (c) 2025 Robert Bosch GmbH.
#
# This program and the accompanying materials are made available under the
# terms of the MIT License which is available at
# https://opensource.org/licenses/MIT.
# 
# SPDX-License-Identifier: MIT

import os
import sys
import argparse
import re
import yaml
import datetime
from typing import Any, Dict, List, Tuple, Optional
from dataclasses import dataclass, field, asdict
from collections import defaultdict
from getpass import getuser

# Constants
RX_MSG_QUEUE_MAXFRAMES = 2  # Maximum number of frames in the RX message queue
FILENAME_MSGS_CFG = "can_msgs_cfg"  # Default output filename for messages configuration

@dataclass
class Signal:
    """Class representing a CAN signal."""
    name: str
    start_bit: int
    length: int
    byte_order: str
    value_type: str
    factor: float
    offset: float
    minimum: float
    maximum: float
    unit: str
    receivers: List[str]
    multiplex_group: Optional[str] = None  # For multiplexed signals
    initial_value: int = 0  # Default initial value
    value_table: Dict[str, str] = field(default_factory=dict)  # Value table for discrete signal values
    comment: str = ""  # Optional comment

@dataclass
class Message:
    """Class representing a CAN message."""
    name: str
    id: int
    id_format: str
    dlc: int
    transmitter: str
    signals: Dict[str, Signal] = field(default_factory=dict)  # Dictionary of signals contained in the message
    tx_method: str = "NON_CYCLIC"  # Default transmission method
    cycle_time: int = 0  # Default cycle time
    transmitters_altern: List[str] = field(default_factory=list)  # List of alternative transmitters

def parse_dbc(file_path: str, encoder: Optional[str] = None) -> Dict[str, List[Message]]:
    """
    Parse a DBC file and return a dictionary of messages categorized by transmitter.
    
    Args:
        file_path (str): Path to the DBC file.
        encoder (Optional[str]): Encoder to use for reading the file.
    
    Returns:
        Dict[str, List[Message]]: Parsed data categorized by transmitter.
    """
    def read_file(file_path: str, encodings: List[str]) -> List[str]:
        """Try reading the file with different encodings."""
        for encoding in encodings:
            try:
                with open(file_path, 'r', encoding=encoding) as file:
                    return file.readlines()
            except UnicodeDecodeError:
                continue
        raise ValueError("No working encoder found")  # Raise an error if no encoder works

    def parse_message(match: re.Match) -> Message:
        """Parse a message line from the DBC file."""
        frame_id, name, dlc, transmitter = match.groups()
        return Message(
            name=name,
            id=int(frame_id),
            id_format="EXTENDED" if int(frame_id) > 0x7FF else "STANDARD",  # Determine ID format
            dlc=int(dlc),
            transmitter=transmitter
        )

    def parse_signal(match: re.Match, is_multiplexed: bool = False) -> Signal:
        """Parse a signal line from the DBC file."""
        if is_multiplexed:
            # For multiplexed signals
            name, mux, start_bit, length, byte_order, value_type, factor, offset, minimum, maximum, unit, receivers = match.groups()
        else:
            # For regular signals
            name, start_bit, length, byte_order, value_type, factor, offset, minimum, maximum, unit, receivers = match.groups()
            mux = None
  
        if byte_order == '0': # Big-endian       
            startbit_int = int(start_bit)                   
            startbit_int += 7 - 2 * (startbit_int % 8)
            startbit_int += int(length) - 1
            startbit_int += 7 - 2 * (startbit_int % 8)
            start_bit = str(startbit_int)    

        return Signal(
            name=name,
            start_bit=int(start_bit),
            length=int(length),
            byte_order='LITTLE_ENDIAN' if byte_order == '1' else 'BIG_ENDIAN',  # Determine byte order
            value_type='SIGNED' if value_type == '-' else 'UNSIGNED',  # Determine value type
            factor=float(factor),
            offset=float(offset),
            minimum=float(minimum),
            maximum=float(maximum),
            unit=unit,
            receivers=[r.strip() for r in receivers.split(',')],  # Parse receivers list
            multiplex_group=mux
        )

    # Try reading the file with different encodings
    encodings = [encoder] if encoder else ['utf-8', 'latin-1', 'cp1252']
    lines = read_file(file_path, encodings)
    
    # Dictionary to store parsed data categorized by transmitter
    parsed_data: Dict[str, List[Message]] = {}
    current_message: Optional[Message] = None
    
    # Regular expressions for matching messages and signals
    message_pattern = re.compile(r'BO_ (\d+) (\w+) *: (\d+) (\w+)')
    signal_pattern = re.compile(r'SG_ (\w+) : (\d+)\|(\d+)@(\d+)([+-]) \(([0-9\.\-]+),([0-9\.\-]+)\) \[([0-9\.\-]+)\|([0-9\.\-]+)\] "([^"]*)" (.*)')
    signal_pattern_multiplexed = re.compile(r'SG_ (\w+) m([0-9\.\-]+) : (\d+)\|(\d+)@(\d+)([+-]) \(([0-9\.\-]+),([0-9\.\-]+)\) \[([0-9\.\-]+)\|([0-9\.\-]+)\] "([^"]*)" (.*)')
    signal_pattern_multiplexor = re.compile(r'SG_ (\w+) M : (\d+)\|(\d+)@(\d+)([+-]) \(([0-9\.\-]+),([0-9\.\-]+)\) \[([0-9\.\-]+)\|([0-9\.\-]+)\] "([^"]*)" (.*)')

    # Parse the lines in the DBC file
    for line in lines:
        line = line.strip()
        
        if match := message_pattern.match(line):
            # If a new message is found, save the previous one and start a new message
            if current_message:
                parsed_data.setdefault(current_message.transmitter, []).append(current_message)
            current_message = parse_message(match)
        elif match := signal_pattern.match(line) or signal_pattern_multiplexor.match(line):
            if current_message:
                # Parse signal and add to the current message
                signal = parse_signal(match)
                current_message.signals[signal.name] = signal
        elif match := signal_pattern_multiplexed.match(line):
            if current_message:
                # Parse multiplexed signal and add to the current message
                signal = parse_signal(match, is_multiplexed=True)
                current_message.signals[signal.name] = signal

    # Add the last message if any
    if current_message:
        parsed_data.setdefault(current_message.transmitter, []).append(current_message)

    # Process additional information
    for line in lines:
        if 'BA_ "GenMsgCycleTime" BO_' in line:
            # Parse cyclic transmission settings
            id_, cycle_time = map(int, re.findall(r'\d+', line))
            for messages in parsed_data.values():
                for message in messages:
                    if message.id == id_:
                        message.tx_method = "CYCLIC"
                        message.cycle_time = cycle_time
                        break

        elif "BO_TX_BU_ " in line:
            # Parse additional transmitters
            parts = line.split()
            id_ = int(parts[1])
            new_nodes = parts[3].rstrip(';').split(',')
            for messages in parsed_data.values():
                for message in messages:
                    if message.id == id_:
                        message.transmitters_altern.extend(node for node in new_nodes if node != message.transmitter)

        elif "BA_ \"GenSigStartValue\" SG_ " in line:
            # Parse initial values for signals
            id_, signal_name, initial_value = re.findall(r'\w+', line)[3:]
            for messages in parsed_data.values():
                for message in messages:
                    if message.id == int(id_) and signal_name in message.signals:
                        message.signals[signal_name].initial_value = int(initial_value)

        elif re.search(r'VAL_ \d+', line):
            # Parse value tables for signals
            id_, signal_name, *value_table = re.findall(r'[""]?\w+[""]?', line)[1:]
            value_table = {value_table[i+1]: value_table[i] for i in range(0, len(value_table), 2)}
            for messages in parsed_data.values():
                for message in messages:
                    if message.id == int(id_) and signal_name in message.signals:
                        message.signals[signal_name].value_table = dict(sorted(value_table.items(), key=lambda item: item[1]))

        elif re.search(r"CM_ SG_ \d+", line):
            # Parse comments for signals
            id_, signal_name, comment = re.findall(r'\w+|"[^"]*"', line)[2:]
            for messages in parsed_data.values():
                for message in messages:
                    if message.id == int(id_) and signal_name in message.signals:
                        message.signals[signal_name].comment = comment.strip('"')

    # Return the parsed data sorted by transmitter
    return dict(sorted(parsed_data.items()))

def parse_args() -> argparse.Namespace:
    """
    Parse command-line arguments.
    
    Returns:
        argparse.Namespace: Parsed command-line arguments.
    """
    parser = argparse.ArgumentParser(description="DBC Parser")
    parser.add_argument("--inputFile", type=str, help="Input file", required=True)
    parser.add_argument('--ecus', nargs='+', help='List of ECUs to import CAN messages from', required=True)
    parser.add_argument("--config", type=str, help="Configuration YAML file")
    parser.add_argument("--outputFolder", type=str, help="Output-folder", default="can/cfg")
    parser.add_argument("--encoder", type=str, help="Encoder to read input file")
    return parser.parse_args()
    

def get_messages(parsed_data: Dict[str, List[Message]], ecu: str) -> Dict[str, Dict[str, Dict[str, Any]]]:
    """
    Organizes messages from parsed data based on their cyclicity and 
    whether they are transmitted or received by a specified ECU.

    Args:
        parsed_data (Dict[str, List[Message]]): A dictionary where keys are transmitter names 
                                                and values are lists of messages associated 
                                                with each transmitter.
        ecu (str): The name of the ECU to filter messages for.

    Returns:
        Dict[str, Dict[str, Dict[str, Any]]]: A nested dictionary categorizing messages as 
                                              'TX' (transmitted) or 'RX' (received), 
                                              grouped by their cyclicity (e.g., '10ms', 'NonCyclic'), 
                                              and then by message name.
    """

    def categorize_message(message: Message) -> str:
        """Categorizes the message based on its cycle time."""
        return f"{message.cycle_time}ms" if message.cycle_time else "NonCyclic"

    def process_message(message: Message, is_tx: bool) -> Dict[str, Any]:
        """Processes a message and filters its signals based on the ECU."""
        message_dict = asdict(message)  # Convert the message to a dictionary.
        
        # If the message is not a transmission, filter its signals.
        if not is_tx:
            message_dict['signals'] = {
                name: signal for name, signal in message_dict['signals'].items()
                if ecu in signal['receivers']
            }
        
        # Return the message dictionary if it contains any signals.
        return message_dict if message_dict['signals'] else {}

    # Initialize a nested defaultdict to organize messages by category, group, and name.
    ecu_messages = defaultdict(lambda: defaultdict(dict))

    # Iterate over all transmitters and their associated messages.
    for transmitter, messages in parsed_data.items():
        for message in messages:
            # Determine if the message is a transmission or reception based on the ECU.
            is_tx = transmitter == ecu or ecu in message.transmitters_altern
            category = 'TX' if is_tx else 'RX'
            
            # Process the message.
            processed_message = process_message(message, is_tx)
            
            # If the processed message is not empty, categorize it and add it to the dictionary.
            if processed_message:
                group = categorize_message(message)
                ecu_messages[category][group][message.name] = processed_message

    # Convert the defaultdict to a regular dictionary before returning.
    return dict(ecu_messages)


def generate_can_messages_configuration(output_folder: str, ecu_messages: Dict[str, Any], cfg_yaml_path: str, dbc_filename: str):
    """
    Generates CAN driver Messages configuration files from provided ECU messages and a YAML configuration.

    This function creates both header and source files necessary for CAN driver Messages configuration. It reads ECU message definitions and YAML configuration to generate the appropriate configurations, including CAN frame definitions, mutexes, message queues, and function prototypes.

    Args:
        output_folder (str): The path to the directory where the generated configuration files will be saved.
        ecu_messages (Dict[str, Any]): A dictionary containing ECU message configurations. It should have keys for different ECU groups, each containing RX and TX message definitions.
        cfg_yaml_path (str): The path to the YAML configuration file which includes function pointers and other configuration details.
        dbc_filename (str): The filename of the DBC file used for reference or documentation purposes.

    Returns:
        None: The function writes the generated header and source files to the specified output folder.
    """

    def load_yaml_config(path: str) -> Dict[str, Any]:
        """
        Load and parse the YAML configuration file into a dictionary.
        If the path is not provided, return an empty dictionary.
        """
        if not path:
            return {}  # Return an empty dictionary if no path is specified
        with open(path) as stream:
            return yaml.safe_load(stream)  # Parse YAML file content

    def get_function_ptr(msg_name: str, cfg_yaml: Dict[str, Any]) -> Tuple[str, str]:
        """
        Retrieve the function pointer and its declaration for a given message name from the YAML configuration.
        Returns a tuple containing the function reference and declaration string.
        """
        cfg_entry = cfg_yaml.get(msg_name, {})  # Get message configuration entry
        fct_ptr = cfg_entry.get("Function", {})  # Get function pointer configuration
        if not fct_ptr:
            return "NULL", None  # Return default values if no function pointer is found

        # Extract function name, arguments, and return type
        fct_name = fct_ptr["name"]
        args = ", ".join(f"{v} {k}" for k, v in fct_ptr.get("args", {}).items()) or "void"
        ret_val = fct_ptr.get("return", "void")
        decl = f"extern {ret_val} {fct_name}({args});\n"  # Generate function declaration
        return f"&{fct_name}", decl

    def build_frame(msg: Dict[str, Any]) -> str:
        """
        Build a CAN frame definition based on message details.
        Returns a string representation of the CAN frame.
        """
        msg_name, values = next(iter(msg.items()))  # Extract message name and details
        id_format = values.get("id_format", "")  # Get ID format for the frame
        # Map ID format to CAN frame flags
        flags = {
            "EXTENDED": "CAN_FRAME_IDE",
            "STANDARD": "0",
            "FD_CAN": "CAN_FRAME_FDF",
            "FD_CAN_BRS": "(CAN_FRAME_FDF | CAN_FRAME_BRS)",
            "FD_CAN_EXTENDED": "(CAN_FRAME_FDF | CAN_FRAME_IDE)",
            "FD_CAN_BRS_EXTENDED": "(CAN_FRAME_FDF | CAN_FRAME_BRS | CAN_FRAME_IDE)"
        }.get(id_format, "0")
        # Construct and return the CAN frame string
        return f"{{ .id = {hex(values.get('id'))}, .dlc = {values.get('dlc')}, .flags = {flags}}}"

    def get_names(msg: Dict[str, Any], is_rx: bool) -> Tuple[str, str, str, str, str, str, str]:
        """
        Generate names and definitions for CAN frames, mutexes, and message queues.
        Returns a tuple with names and definitions for these elements.
        """
        msg_name = next(iter(msg.keys()))  # Get the message name
        postfix = "_Rx" if is_rx else "_Tx"  # Determine postfix based on message type (RX or TX)
        # Construct names for CAN frame, mutex, and message queue
        can_frame_name = f"can_Cfg_Frame_{msg_name}{postfix}"
        can_frame_var = f"struct can_frame {can_frame_name} = {build_frame(msg)};\n"
        mutex_name = f"can_Cfg_Mutex_{msg_name}{postfix}"
        mutex_var = f"struct k_mutex {mutex_name};\n"
        rx_msg_queue_name = f"can_Cfg_MsgQueue_{msg_name}{postfix}"
        rx_msg_queue_var = f"CAN_MSGQ_DEFINE({rx_msg_queue_name}, {RX_MSG_QUEUE_MAXFRAMES});\n"
        return msg_name, can_frame_name, can_frame_var, mutex_name, mutex_var, rx_msg_queue_name, rx_msg_queue_var

    def generate_header_content(include_guard: str, title_str: str, rx_msg_count: int,
                                tx_messages_counts: List[str], rx_msgs_enum_list: List[str],
                                tx_msgs_enum_list: List[str], rx_array_name: str,
                                tx_msgs_vars: List[str], tx_map_array_name: str,
                                rx_callout_functions: List[str], pre_tx_functions: List[str]) -> List[str]:
        """
        Generate the content for the header file.
        This includes include guards, message counts, enums, extern declarations, and function prototypes.
        """
        content = [
            f"{title_str}#ifndef {include_guard}\n#define {include_guard}\n\n#include \"can_types.h\"\n\n",
            f"#define can_CFG_MAX_MSGS_RX {rx_msg_count}u\n",  # Define maximum number of RX messages
            *tx_messages_counts,  # Include TX message counts
            *rx_msgs_enum_list,  # Include RX message enums
            *tx_msgs_enum_list,  # Include TX message enums
        ]
        if rx_msg_count:
            content.append(f"\n// Rx Messages:\nextern {rx_array_name};\n\n")  # Declare RX message array
        if tx_msgs_vars:
            content.extend([
                "// Tx Messages:\n",
                *[f"extern {var}\n" for var in tx_msgs_vars],  # Declare TX message variables
                "// Mapping structure between Tx Messages and the can_Tx_Message_e enum\n",
                f"extern {tx_map_array_name};\n"  # Declare TX map array
            ])
        if rx_callout_functions:
            content.extend(["\n// Rx Callout Functions:\n", *rx_callout_functions])  # Declare RX callout functions
        if pre_tx_functions:
            content.extend(["\n// Pre-Tx Functions:\n", *pre_tx_functions])  # Declare pre-TX functions
        content.append(f"\n\n#endif // {include_guard}\n")  # End include guard
        return content

    def generate_source_content(title_str: str, mutexes: List[str], can_frames: List[str],
                                rx_msg_queues: List[str], rx_array_entries: List[str],
                                tx_array_entries: List[str], tx_map_list: List[str]) -> List[str]:
        """
        Generate the content for the source file.
        This includes mutexes, CAN frames, RX message queues, and entries for both RX and TX messages.
        """
        return [
            f"{title_str}#include \"can_msgs_cfg.h\"\n",  # Include header file
            "\n// Mutexes\n", *mutexes,  # Add mutex definitions
            "\n// Can Frames\n", *can_frames,  # Add CAN frame definitions
            "\n// Configuration of the RX messages queues\n", *rx_msg_queues,  # Add RX message queues
            "\n// The Rx Messages:\n", *rx_array_entries,  # Add RX message array entries
            "\n// The Tx Messages:\n", *tx_array_entries,  # Add TX message array entries
            "\n// The Tx Map List:\n", *tx_map_list  # Add TX map list
        ]

    # Load the YAML configuration
    cfg_yaml = load_yaml_config(cfg_yaml_path)
    
    # Define include guard based on the filename
    include_guard = f"{FILENAME_MSGS_CFG.replace('.', '_').upper()}_H"

    # Create header with file generation info
    date_time = datetime.datetime.now().strftime("%Y-%m-%d, %H:%M:%S")
    user = getuser().upper()
    title_subStr = os.path.basename(cfg_yaml_path) if cfg_yaml_path else "None"
    title_str = f"// File automatically generated by {user} on {date_time}\n// \t- DBC file:\n//\t\t{dbc_filename}\n//\t- YAML Message Configuration file:\n//\t\t{title_subStr}\n\n"

    # Initialize lists to store configurations
    rx_array_entries, can_frames, mutexes, rx_msg_queues = [], [], [], []
    rx_msg_count, rx_msgs_enum_list, rx_callout_functions = 0, [], []
    rx_array_name = "const can_CanRxMsg_Config_t can_Cfg_RxMsgs[can_RXPDU_MAX]"

    # Process RX messages
    for msgs in ecu_messages.values():
        rx_msgs = msgs.get("RX", {})
        rx_msg_count += len(rx_msgs)  # Count the number of RX messages
        for rx_msg in rx_msgs.values():
            # Generate names and variables for each RX message
            msg_name, can_frame_name, can_frame_var, mutex_name, mutex_var, rx_msg_queue_name, rx_msg_queue_var = get_names(rx_msg, True)
            
            # Retrieve function pointer declaration for the RX message
            fct_ptr_ref, fct_ptr_decl = get_function_ptr(msg_name, cfg_yaml)
            if fct_ptr_decl:
                rx_callout_functions.append(fct_ptr_decl)
            
            # Append CAN frame, mutex, and message queue definitions
            can_frames.append(can_frame_var)
            mutexes.append(mutex_var)
            rx_msg_queues.append(rx_msg_queue_var)
            
            # Initialize RX message array entry if not already done
            if not rx_array_entries:
                rx_array_entries.append(f"{rx_array_name} = {{\n")
            
            # Add RX message entry to the array
            rx_array_entries.append(f"\t// {msg_name}\n\t{{ .frame = &{can_frame_name}, .CanMutex = &{mutex_name}, .msgQueue = &{rx_msg_queue_name}, .CanRxCalloutFct = {fct_ptr_ref} }},\n")
            
            # Create enum name and add to RX message enum list
            enum_name = f"\tcan_RXPDU_{msg_name.upper()}"
            if not rx_msgs_enum_list:
                rx_msgs_enum_list.append(f"\ntypedef enum\n{{")
                enum_name += " = 0u"
            rx_msgs_enum_list.append(f"{enum_name},\n")

    # Finalize RX array entries and enum list
    if rx_array_entries:
        rx_array_entries.append("};\n")
    if rx_msgs_enum_list:
        rx_msgs_enum_list.extend(["\tcan_RXPDU_MAX", "\n} can_Rx_Message_e;\n"])

    # Initialize variables for TX messages
    non_cyclic_msgs = False
    tx_messages_counts, tx_msgs_vars, tx_msgs_enum_list, tx_map_list, tx_array_entries, pre_tx_functions = [], [], [], [], [], []
    tx_map_array_name = "const can_CanTxMsg_Config_t can_CanTxMsg_Map[can_TXPDU_MAX]"

    # Process TX messages
    for msgs in ecu_messages.values():
        tx_messages = msgs.get("TX", {})
        for group_title, messages in tx_messages.items():
            # Initialize TX map list if not already done
            if not tx_map_list:
                tx_map_list.append(f"{tx_map_array_name} =\n{{\n")
            if group_title == "NonCyclic":
                non_cyclic_msgs = True  # Mark if non-cyclic messages are present
            
            # Define macro for TX message count
            define_title = f"can_CFG_MAX_MSGS_TX_{group_title.upper()}"
            tx_messages_counts.append(f"#define {define_title} {len(messages)}u\n")
            tx_msg_varname_only = f"can_Cfg_TxMsgs_{group_title}"
            tx_msg_varname = f"const can_CanTxMsg_Config_t {tx_msg_varname_only}[{define_title}]"
            
            # Add TX message variables to the list
            tx_msgs_vars.append(f"{tx_msg_varname};")
            if tx_array_entries:
                tx_array_entries.append("\n")
            tx_array_entries.append(f"{tx_msg_varname} = {{\n")
            for i, (tx_msg_name, values) in enumerate(messages.items()):
                tx_msg = {tx_msg_name: values}
                
                # Generate names and variables for each TX message
                msg_name, can_frame_name, can_frame_var, mutex_name, mutex_var, _, _ = get_names(tx_msg, False)
                
                # Retrieve function pointer declaration for the TX message
                fct_ptr_ref, fct_ptr_decl = get_function_ptr(msg_name, cfg_yaml)
                if fct_ptr_decl:
                    pre_tx_functions.append(fct_ptr_decl)
                
                # Append CAN frame and mutex definitions
                can_frames.append(can_frame_var)
                mutexes.append(mutex_var)
                
                # Add TX message entry to the array
                tx_array_entries.append(f"\t// {msg_name}\n\t{{ .frame = &{can_frame_name}, .CanMutex = &{mutex_name}, .Pre_TxFct = {fct_ptr_ref}}},\n")
                
                # Create enum name and add to TX message enum list
                enum_name = f"can_TXPDU_{msg_name.upper()}"
                if not tx_msgs_enum_list:
                    tx_msgs_enum_list.append(f"\ntypedef enum\n{{")
                    enum_name += " = 0u"
                tx_msgs_enum_list.append(f"\t{enum_name},\n")
                tx_map_list.append(f"\t{tx_msg_varname_only}[{i}], // {enum_name}\n")
            tx_array_entries.append("};\n")

    # Finalize TX message enum list and map list
    if tx_msgs_enum_list:
        tx_msgs_enum_list.extend(["\tcan_TXPDU_MAX", "\n} can_Tx_Message_e;\n"])
    tx_map_list.append("};\n")

    # Add definition for non-cyclic TX messages if none are found
    if not non_cyclic_msgs:
        tx_messages_counts.append("#define can_CFG_MAX_MSGS_TX_NONCYCLIC 0u\n")

    # Generate header and source file contents
    header_content = generate_header_content(include_guard, title_str, rx_msg_count, tx_messages_counts,
                                             rx_msgs_enum_list, tx_msgs_enum_list, rx_array_name,
                                             tx_msgs_vars, tx_map_array_name, rx_callout_functions, pre_tx_functions)

    source_content = generate_source_content(title_str, mutexes, can_frames, rx_msg_queues,
                                             rx_array_entries, tx_array_entries, tx_map_list)

    # Write header content to the header file
    with open(os.path.join(output_folder, f"{FILENAME_MSGS_CFG}.h"), "w") as f_out:
        f_out.writelines(header_content)

    # Write source content to the source file
    with open(os.path.join(output_folder, f"{FILENAME_MSGS_CFG}.c"), "w") as f_out:
        f_out.writelines(source_content)



def main() -> int:
    args = parse_args()
    
    parsed_data = parse_dbc(args.inputFile, args.encoder)

    if not parsed_data:
        print("No data parsed")
        return 1
    
    messages = dict()
    # Get the messages for the selected ECUs
    for ecu in args.ecus:
        messages[ecu] = get_messages(parsed_data, ecu)

    
    generate_can_messages_configuration(args.outputFolder, messages, args.config, os.path.basename(args.inputFile))

    return 0

if __name__ == "__main__":
    sys.exit(main())