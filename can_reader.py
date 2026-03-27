import can
import threading
import time

# --- CONFIGURATION ---
BMS_ID_SOC_DATA = 0x69  # Contains: SOC, Avg Temp, High Temp, Current, DCL
BMS_ID_VOLT_DATA = 0x67 # Contains: Voltage, CCL, Ah, Res

class CANReader:
    def __init__(self, data_model):
        self.data_model = data_model
        self.bus = None
        self.running = False
        self.thread = None

    def start(self):
        """Starts the CAN reading thread."""
        if self.running:
            return
            
        print("Initializing CAN interface...")
        try:
            # We assume the interface is already 'up' via the terminal command
            self.bus = can.interface.Bus(channel='can0', bustype='socketcan')
            self.running = True
            self.thread = threading.Thread(target=self.bms_broadcast_listener, daemon=True)
            self.thread.start()
            print("CAN Listener Thread Started.")
        except OSError:
            print("ERROR: Could not connect to can0. Did you run: sudo ip link set can0 up...?")
        except Exception as e:
            print(f"ERROR: CAN Init failed: {e}")

    def stop(self):
        self.running = False
        if self.bus:
            self.bus.shutdown()

    def bms_broadcast_listener(self):
        """
        Listens for BMS messages on 0x69 and 0x67
        """
        print(f"CAN Listener started... waiting for {hex(BMS_ID_SOC_DATA)} and {hex(BMS_ID_VOLT_DATA)}")
        
        while self.running:
            try:
                msg = self.bus.recv(timeout=1.0)
                if msg is None:
                    continue

                # --- MESSAGE 1: SOC, TEMPS, CURRENT, DCL (ID 0x69) ---
                if msg.arbitration_id == BMS_ID_SOC_DATA:
                    # Byte 0: Pack SOC
                    self.data_model.update_value('soc', msg.data[0])

                    # Byte 1: Avg Temp
                    self.data_model.update_value('battery_temp', msg.data[1])

                    # Byte 2: High Temp
                    self.data_model.update_value('temp_high', msg.data[2])

                    # Byte 3 & 4: Pack Current (2 Bytes)
                    # Combined High Byte and Low Byte
                    current_raw = (msg.data[3] << 8) | msg.data[4]
                    
                    # Handle Negative Numbers (Signed 16-bit Integer)
                    # If the number is > 32767, it represents a negative value (Discharge)
                    if current_raw > 32767:
                        current_raw -= 65536
                    
                    # Apply Scaling (Assuming 0.1, e.g., 100 = 10.0A)
                    self.data_model.update_value('current', current_raw * 0.1)

                    # Byte 5 & 6: DCL (Discharge Current Limit)
                    dcl_raw = (msg.data[5] << 8) | msg.data[6]
                    self.data_model.update_value('pack_dcl', dcl_raw * 0.1)

                # --- MESSAGE 2: VOLTAGE, CCL, AH, RES (ID 0x67) ---
                elif msg.arbitration_id == BMS_ID_VOLT_DATA:
                    # Byte 0 & 1: Pack Inst. Voltage (2 Bytes)
                    voltage_raw = (msg.data[0] << 8) | msg.data[1]
                    self.data_model.update_value('pack_voltage', voltage_raw * 0.1)

                    # Byte 2 & 3: CCL (Charge Current Limit)
                    ccl_raw = (msg.data[2] << 8) | msg.data[3]
                    self.data_model.update_value('pack_ccl', ccl_raw * 0.1)

                    # Byte 4: Pack Amp-Hours
                    self.data_model.update_value('pack_ah', msg.data[4])

                    # Byte 5: Pack Resistance (mOhm)
                    self.data_model.update_value('pack_res', msg.data[5])

            except Exception as e:
                print(f"CAN Error: {e}")
                time.sleep(1)
