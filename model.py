"""
=============================================
 Vehicle Dashboard Data Model
=============================================

This module defines the DataModel class: a single source of truth
for all runtime values. It manages sensor inputs from CAN, GPIO,
or runs in simulation mode.
"""

import time
import math
import threading
from typing import Any, Dict

# KPH to MPH conversion factor
KPH_TO_MPH = 0.621371

class DataModel:
    """
    Holds all dashboard state. UI elements read from this class.
    Sensor classes write to this class.
    """
    def __init__(self,
                 units: str = "km/h",
                 simulate: bool = False,
                 enable_can: bool = True,
                 enable_gpio: bool = False
                 ):
        
        print(f"[DataModel] Initializing (Sim: {simulate}, CAN: {enable_can}, GPIO: {enable_gpio})")
        self.units = units
        self.is_simulating = simulate
        self.lock = threading.Lock() # For thread-safety

        # --- Public Attributes (The Source of Truth) ---
        self.speed: float = 0.0
        self.gear: str = "P"
        
        # Battery Data (CAN ID 0x69 & 0x67)
        self.soc: float = 0.0            # %
        self.pack_voltage: float = 0.0   # V
        self.current: float = 0.0        # A
        self.battery_temp: float = 0.0   # Avg Temp (C) - NEW NAME
        self.temp: float = 0.0           # Avg Temp (C) - OLD NAME (Kept for UI compatibility)
        self.temp_high: float = 0.0      # High Temp (C)
        self.pack_dcl: float = 0.0       # Discharge Limit (A)
        self.pack_ccl: float = 0.0       # Charge Limit (A)
        self.pack_ah: float = 0.0        # Amp Hours
        self.pack_res: float = 0.0       # Resistance (mOhm)
        
        # Controller/Motor Data
        self.controller_temp: float = 0.0
        
        # GPIO / Status Flags
        self.turn_left: bool = False
        self.turn_right: bool = False
        self.headlight: bool = False
        self.warn: bool = False

    def update_value(self, key: str, value: Any):
        """
        Thread-safe method for SENSOR threads (like CANReader)
        to update a value.
        """
        with self.lock:
            # Handle Unit Conversion for Speed
            if key == 'speed' and self.units == 'mph':
                value = value * KPH_TO_MPH
            
            # Update the value if the key exists
            if hasattr(self, key):
                setattr(self, key, value)
                
                # --- SYNC OLD VARIABLES FOR UI ---
                # If we update 'battery_temp', also update 'temp' so the UI doesn't crash
                if key == 'battery_temp':
                    self.temp = value
            else:
                pass

    def get_all(self) -> Dict[str, Any]:
        """
        Returns a thread-safe copy of all data for logging/sending.
        """
        with self.lock:
            return {
                "speed": self.speed,
                "soc": self.soc,
                "pack_voltage": self.pack_voltage,
                "current": self.current,
                "battery_temp": self.battery_temp,
                "temp": self.temp,
                "temp_high": self.temp_high,
                "pack_dcl": self.pack_dcl,
                "pack_ccl": self.pack_ccl,
                "pack_ah": self.pack_ah,
                "pack_res": self.pack_res,
                "controller_temp": self.controller_temp,
                "gear": self.gear,
                "turn_left": self.turn_left,
                "turn_right": self.turn_right,
                "headlight": self.headlight,
                "warn": self.warn
            }

    def update(self, dt: float):
        """Main update loop. Called by main.py."""
        if self.is_simulating:
            self._simulate(dt)
        else:
            # Real Mode: CAN is handled by background threads.
            # We perform logic checks here.
            self._check_warnings()

    def _check_warnings(self):
        """Internal: check for warning conditions."""
        # Example Warning Logic
        if self.controller_temp > 90.0:
            self.warn = True
        elif self.soc < 10.0:
            self.warn = True
        elif self.temp_high > 60.0:
            self.warn = True
        else:
            self.warn = False

    def _simulate(self, dt: float):
        """Generates fake data for UI development/Testing."""
        t = time.time()
        with self.lock:
            # 1. Simulate Motion
            base_speed_kph = abs(80 + (20 * math.sin(t * 0.5)))
            if self.units == 'mph':
                self.speed = base_speed_kph * KPH_TO_MPH
            else:
                self.speed = base_speed_kph

            # 2. Simulate Battery Data
            self.soc = abs(75 + (20 * math.sin(t * 0.1)))
            self.pack_voltage = 52.0 + (2.0 * math.sin(t * 0.2))
            self.current = 30 + (100 * math.sin(t * 0.5)) 
            
            # Update both temp variables
            sim_temp = 35 + (5 * math.sin(t * 0.05))
            self.battery_temp = sim_temp
            self.temp = sim_temp  # Keep synced
            
            self.temp_high = sim_temp + 5
            
            # Simulate Limits & Health
            self.pack_dcl = 150.0
            self.pack_ccl = 50.0
            self.pack_ah = 100.0 - (t % 10) 
            self.pack_res = 15 + (math.sin(t) * 2)

            # 3. Simulate GPIO/Status
            blink_state = int(t * 2) % 2 == 0
            if int(t) % 10 < 5:
                self.turn_left = blink_state
                self.turn_right = False
            else:
                self.turn_left = False
                self.turn_right = blink_state
                
            gears = ["P", "R", "N", "D"]
            self.gear = gears[int(t / 8) % len(gears)]

    def stop(self):
        """Stops sensors if needed (CAN handles this via its own stop method)."""
        pass
