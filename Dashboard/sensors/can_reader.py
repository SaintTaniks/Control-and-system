import can
import threading
import time

# --- CONFIGURATION ---
BMS_ID_SOC_DATA = 0x69
BMS_ID_VOLT_DATA = 0x67
BMS_ID_CELL_DATA = 0x420
SEVCON_ID_RPM = 0x454 

class CANReader(threading.Thread):
    def __init__(self, model, channel='can0'):
        super().__init__()
        self.model = model
        self.channel = channel
        self.daemon = True
        self.running = True
        self.bus = None
        
        # --- NEW: Dictionary to hold all individual cell voltages ---
        self.cell_voltages = {}  

        print("Initializing CAN interface...")
        try:
            self.bus = can.interface.Bus(channel=self.channel, bustype='socketcan')
            print(f"CAN Listener started on {self.channel}")
        except Exception as e:
            print(f"ERROR: CAN Init failed on {self.channel}: {e}")

    def run(self):
        if self.bus is None:
            return
            
        print(f"CAN Listener waiting for data...")
        
        while self.running:
            try:
                msg = self.bus.recv(timeout=1.0)
                if msg is None:
                    continue

                if msg.arbitration_id == BMS_ID_SOC_DATA:
                    self.model.update_value('soc', msg.data[0])
                    self.model.update_value('battery_temp', msg.data[1])
                    self.model.update_value('temp_high', msg.data[2])
                    current_raw = (msg.data[3] << 8) | msg.data[4]
                    if current_raw > 32767:
                        current_raw -= 65536
                    self.model.update_value('current', current_raw * 0.1)
                    dcl_raw = (msg.data[5] << 8) | msg.data[6]
                    self.model.update_value('pack_dcl', dcl_raw * 0.1)

                elif msg.arbitration_id == BMS_ID_VOLT_DATA:
                    voltage_raw = (msg.data[0] << 8) | msg.data[1]
                    self.model.update_value('pack_voltage', voltage_raw * 0.1)
                    ccl_raw = (msg.data[2] << 8) | msg.data[3]
                    self.model.update_value('pack_ccl', ccl_raw * 0.1)
                    self.model.update_value('pack_ah', msg.data[4])
                    self.model.update_value('pack_res', msg.data[5])

                # --- THE MULTIPLEXED CELL DATA FIX ---
                elif msg.arbitration_id == BMS_ID_CELL_DATA:
                    if len(msg.data) >= 4:
                        cell_id = msg.data[0] # Byte 0 is the Cell Number
                        voltage_raw = (msg.data[2] << 8) | msg.data[3] # Bytes 2 & 3 are the Voltage
                        voltage = voltage_raw * 0.0001
                        
                        # Store/Update this specific cell's voltage in our dictionary
                        self.cell_voltages[cell_id] = voltage
                        
                        # Once we have data, dynamically find the absolute max and min across all cells
                        if len(self.cell_voltages) > 0:
                            high_v = max(self.cell_voltages.values())
                            low_v = min(self.cell_voltages.values())
                            
                            self.model.update_value('high_cell_voltage', high_v)
                            self.model.update_value('low_cell_voltage', low_v)

                # --- MOTOR RPM TO VEHICLE SPEED ---
                elif msg.arbitration_id == SEVCON_ID_RPM:
                    if len(msg.data) >= 3:
                        rpm_raw = int.from_bytes(msg.data[1:3], byteorder='little', signed=True)
                        self.model.update_value('rpm', rpm_raw)
                        
                        # Apply the math formula: (RPM / Gear Ratio) * Circumference * (Mins / Inches in Mile)
                        speed_mph = abs((rpm_raw / 2.5) * 34.0 * 60.0 / 63360.0)
                        self.model.update_value('speed', speed_mph)

            except Exception as e:
                pass
                
    def stop(self):
        self.running = False
        if self.bus:
            self.bus.shutdown()
