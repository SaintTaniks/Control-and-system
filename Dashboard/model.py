"""
=============================================
 Vehicle Dashboard Data Model
=============================================
"""

import time
import math
import threading
from typing import Any, Dict

KPH_TO_MPH = 0.621371

class DataModel:
    def __init__(self, units: str = "mph", simulate: bool = False, enable_can: bool = True, enable_gpio: bool = False):
        self.units = units
        self.is_simulating = simulate
        self.lock = threading.Lock()

        # Public Attributes
        self.speed: float = 0.0
        self.rpm: int = 0                # --- NEW RPM VARIABLE ---
        self.gear: str = "P"
        
        # Battery Data 
        self.soc: float = 0.0            
        self.pack_voltage: float = 0.0   
        self.current: float = 0.0        
        self.battery_temp: float = 0.0   
        self.temp: float = 0.0           
        self.temp_high: float = 0.0      
        
        self.pack_dcl: float = 0.0
        self.pack_ccl: float = 0.0
        self.pack_ah: float = 0.0
        self.pack_res: float = 0.0
        
        self.high_cell_voltage: float = 0.0  
        self.low_cell_voltage: float = 0.0   

        self.turn_left: bool = False
        self.turn_right: bool = False
        self.headlight: bool = False
        self.warn: bool = False

    def update_value(self, key: str, value: Any):
        with self.lock:
            if hasattr(self, key):
                setattr(self, key, value)
                if key == 'battery_temp':
                    self.temp = value

    def get_all(self) -> Dict[str, Any]:
        with self.lock:
            return {
                "speed": self.speed,
                "rpm": self.rpm,         # --- NEW ---
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
                "high_cell_voltage": self.high_cell_voltage,
                "low_cell_voltage": self.low_cell_voltage,
                "gear": self.gear,
                "warn": self.warn
            }

    def update(self, dt: float):
        if self.is_simulating:
            self._simulate(dt)

    def _simulate(self, dt: float):
        t = time.time()
        with self.lock:
            self.soc = abs(75 + (20 * math.sin(t * 0.1)))
            self.pack_voltage = 52.0 + (2.0 * math.sin(t * 0.2))
            self.current = 30 + (100 * math.sin(t * 0.5)) 
            
            sim_temp = 35 + (5 * math.sin(t * 0.05))
            self.battery_temp = sim_temp
            self.temp = sim_temp  
            self.temp_high = sim_temp + 5
            
            self.pack_dcl = 150.0
            self.pack_ccl = 50.0
            self.pack_ah = 100.0 - (t % 10) 
            self.pack_res = 15 + (math.sin(t) * 2)

            self.high_cell_voltage = 4.12 + (0.05 * math.sin(t * 0.2))
            self.low_cell_voltage = 3.95 + (0.05 * math.sin(t * 0.2))
            
            # Fake RPM Data bouncing from 1000 to 4000
            self.rpm = int(2500 + (1500 * math.sin(t * 0.8)))

    def stop(self):
        pass
