"""
=============================================
 CAN Bus Reader (All-in-One)
=============================================
Manages all CAN bus communication by running two dedicated threads:
1. A listener for broadcast messages (BMS, Gear, Blinkers).
2. A poller for SEVCON Gen4 using the CANopen protocol.
This class directly updates the DataModel object that is passed to it.
"""

import threading
import time
import can
import canopen

# --- CONFIGURATION - VERIFY THESE VALUES ---

# 1. CAN Interface
CAN_INTERFACE = 'can0' # 'sudo ip link set can0 up type can bitrate 500000'

# 2. SEVCON Node ID (CANopen)
SEVCON_NODE_ID = 1  # Example: 1 (Verify this in your SEVCON config)

# 3. Orion BMS CAN IDs (Extended 29-bit IDs)
# These are examples. Replace with your actual IDs.
BMS_BROADCAST_ID_1 = 0x1806E5F4  # (BMS) Contains Pack Voltage and Current
BMS_BROADCAST_ID_2 = 0x1806E7F4  # (BMS) Contains High/Low Cell V, SOC

# 4. Other Vehicle CAN IDs (Standard 11-bit IDs)
# These are placeholders. Replace with your actual IDs.
ID_GEAR = 0x124                  # (Vehicle) Gear Status (P, R, N, D)
ID_BLINK = 0x126                 # (Vehicle) Turn signals

# --- END OF CONFIGURATION ---


class CANReader:
    def __init__(self, data_model):
        """
        Initializes the CAN reader.
        Args:
            data_model: The main DataModel instance to update.
        """
        self.data_model = data_model
        self.is_running = False
        
        # Thread for SEVCON (CANopen)
        self.canopen_thread = threading.Thread(
            target=self.sevcon_canopen_poller, 
            daemon=True
        )
        
        # Thread for BMS/Vehicle (Standard CAN)
        self.broadcast_thread = threading.Thread(
            target=self.bms_broadcast_listener, 
            daemon=True
        )

    def start(self):
        """Starts the CAN reader threads."""
        if not self.is_running:
            self.is_running = True
            #self.canopen_thread.start()
            self.broadcast_thread.start()
            print("[CANReader] All CAN threads started.")

    def stop(self):
        """Stops the CAN reader threads."""
        self.is_running = False
        print("[CANReader] Stop signal sent.")

    def bms_broadcast_listener(self):
        """
        Listens for standard 11-bit and extended 29-bit CAN messages
        from BMS, gear selector, blinkers, etc.
        """
        print("[CANReader] BMS/Broadcast Listener thread started.")
        bus = None
        while self.is_running:
            try:
                if bus is None:
                    bus = can.interface.Bus(CAN_INTERFACE, bustype='socketcan')
                    print("[CANReader] Broadcast Listener: CAN bus connected.")
                
                msg = bus.recv(timeout=1.0) # Wait for a message
                if msg is None:
                    continue

                # --- 29-bit Extended ID (BMS) ---
                if msg.is_extended_id:
                    if msg.arbitration_id == BMS_BROADCAST_ID_1:
                        # --- DECODE BMS_BROADCAST_ID_1 ---
                        # Example: Bytes 0-1 = Pack V (0.1V scale)
                        # Example: Bytes 2-3 = Pack I (0.1A scale, signed)
                        pack_v = ((msg.data[0] << 8) | msg.data[1]) * 0.1
                        pack_i = ((msg.data[2] << 8) | msg.data[3]) * 0.1
                        
                        self.data_model.update_value('v_pack', pack_v)
                        self.data_model.update_value('i_pack', pack_i)

                    elif msg.arbitration_id == BMS_BROADCAST_ID_2:
                        # --- DECODE BMS_BROADCAST_ID_2 ---
                        # Example: Byte 0 = SOC (1% scale)
                        soc = msg.data[0] * 1.0
                        self.data_model.update_value('soc', soc)
                
                # --- 11-bit Standard ID (Vehicle) ---
                else:
                    if msg.arbitration_id == ID_GEAR:
                        # --- DECODE ID_GEAR ---
                        gear_map = {0: "P", 1: "R", 2: "N", 3: "D"}
                        gear_val = msg.data[0]
                        self.data_model.update_value('gear', gear_map.get(gear_val, "?"))

                    elif msg.arbitration_id == ID_BLINK:
                        # --- DECODE ID_BLINK ---
                        # This code will be overridden if GPIO is enabled
                        blink_val = msg.data[0]
                        self.data_model.update_value('turn_left', blink_val in [1, 3])
                        self.data_model.update_value('turn_right', blink_val in [2, 3])
                        
            except Exception as e:
                if self.is_running:
                    print(f"[CANReader] Broadcast Listener Error: {e}")
                    if bus:
                        bus.shutdown()
                    bus = None
                    time.sleep(3) # Wait before retrying

    def sevcon_canopen_poller(self):
        """Polls the SEVCON controller via CANopen and updates the data model."""
        print("[CANReader] SEVCON Poller thread started.")
        network = None
        node = None

        while self.is_running:
            try:
                if network is None:
                    network = canopen.Network()
                    network.connect(channel=CAN_INTERFACE, bustype='socketcan')
                    node = network.add_node(SEVCON_NODE_ID)
                    print("[CANReader] SEVCON Poller: CANopen network connected.")

                # --- Poll for data (using SDO read) ---
                # These are examples, replace with your SDO objects
                
                # Example: Motor Speed (RPM)
                motor_speed_rpm = node.sdo[0x2915].raw # Adjust object index
                
                # --- !! CONVERT RPM to KPH !! ---
                # This is a placeholder. You MUST calculate this.
                WHEEL_CIRCUMFERENCE_M = 1.8 # Example: 1.8 meters
                GEAR_RATIO = 10.0             # Example: 10:1
                KPH = (motor_speed_rpm * (WHEEL_CIRCUMFERENCE_M / 1000) * 60) / GEAR_RATIO
                
                self.data_model.update_value('speed', KPH)

                # Example: Motor Temp
                motor_temp = node.sdo[0x2904].raw * 0.1 # Adjust object/scale
                self.data_model.update_value('temp', motor_temp)
                
                # Example: Controller Temp
                controller_temp = node.sdo[0x2903].raw * 0.1 # Adjust object/scale
                self.data_model.update_value('controller_temp', controller_temp)

                time.sleep(0.1) # Poll at 10Hz

            except Exception as e:
                if self.is_running:
                    print(f"[CANReader] SEVCON Poller Error: {e}")
                    if network:
                        network.disconnect()
                    network = None
                    node = None
                    time.sleep(3) # Wait before retrying
