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

# Import all sensor readers
from dashboard.sensors.can_reader import CANReader
from dashboard.sensors.gpio_inputs import GPIOInputs, GPIOConfig

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

        # Public attributes read by the UI
        self.speed: float = 0.0
        self.soc: float = 0.0
        self.temp: float = 0.0
        self.v_pack: float = 0.0
        self.i_pack: float = 0.0
        self.controller_temp: float = 0.0
        self.gear: str = "P"
        self.turn_left: bool = False
        self.turn_right: bool = False
        self.headlight: bool = False
        self.warn: bool = False

        # --- Sensor Objects ---
        self._can = None
        self._gpio = None

        if not self.is_simulating:
            if enable_can:
                try:
                    # Pass 'self' (the DataModel instance) to the CANReader
                    self._can = CANReader(self) 
                    print("[DataModel] CANReader initialized.")
                except Exception as e:
                    print(f"[DataModel] FAILED to initialize CANReader: {e}")
                    self._can = None
            
            if enable_gpio:
                try:
                    # --- CONFIGURE YOUR GPIO PINS HERE ---
                    # These are examples from your gpio_inputs.py file
                    cfg = GPIOConfig(
                        pin_left=17, 
                        pin_right=27, 
                        pin_headlight=22,
                        active_low=True, 
                        debounce_ms=40
                    )
                    self._gpio = GPIOInputs(cfg)
                    print("[DataModel] GPIOInputs initialized.")
                except Exception as e:
                    print(f"[DataModel] FAILED to initialize GPIOInputs: {e}")
                    self._gpio = None

    def start_sensors(self):
        """Starts all sensor threads (e.g., CAN)."""
        if self._can:
            print("[DataModel] Starting CANReader threads...")
            self._can.start()
        if self._gpio:
            print("[DataModel] GPIO is active (polled in update loop).")
        elif self.is_simulating:
            print("[DataModel] Running in simulation mode. No sensors started.")

    def update(self, dt: float):
        """Main update loop. Called by main.py."""
        if self.is_simulating:
            self._simulate(dt)
        else:
            # CAN data is pushed in by its own background threads.
            # We only need to actively poll GPIO here.
            if self._gpio:
                self._poll_gpio()
            
            # Check for warnings based on the latest data
            self._check_warnings()

    def _poll_gpio(self):
        """Reads from GPIO and updates the model."""
        try:
            # The .read() method is from your gpio_inputs.py file
            snap = self._gpio.read() 
            
            # Use lock to update values
            with self.lock:
                # Let GPIO override CAN for these signals
                self.turn_left = snap["left"]
                self.turn_right = snap["right"]
                self.headlight = snap["headlight"]
                
                # You can also read snap["hazard"], snap["brake"], etc.
                
        except Exception as e:
            print(f"[DataModel] GPIO read error: {e}")

    def update_value(self, key: str, value: Any):
        """
        Thread-safe method for SENSOR threads (like CANReader)
        to update a value.
        """
        with self.lock:
            if hasattr(self, key):
                
                if key == 'speed' and self.units == 'mph':
                    # Assuming speed from CAN is *always* KPH
                    value = value * KPH_TO_MPH
                
                setattr(self, key, value)

    def get_all_data(self) -> Dict[str, Any]:
        """
        Returns a thread-safe copy of all data for logging/sending.
        """
        with self.lock:
            return {
                "speed": self.speed,
                "soc": self.soc,
                "temp": self.temp,
                "v_pack": self.v_pack,
                "i_pack": self.i_pack,
                "controller_temp": self.controller_temp,
                "gear": self.gear,
                "turn_left": self.turn_left,
                "turn_right": self.turn_right,
                "headlight": self.headlight,
                "warn": self.warn
            }

    def _check_warnings(self):
        """Internal: check for warning conditions."""
        if self.controller_temp > 90.0:
            self.warn = True
        elif self.soc < 10.0:
            self.warn = True
        else:
            self.warn = False

    def stop(self):
        """Tells all sensor threads to stop."""
        print("[DataModel] Stop signal received.")
        if self._can:
            self._can.stop()
        if self._gpio:
            # gpiozero usually cleans up on its own
            print("[DataModel] GPIO stopping.")

    def _simulate(self, dt: float):
        """Generates fake data for UI development."""
        t = time.time()
        with self.lock:
            # Simulate CAN data
            self.speed = abs(60 + (20 * math.sin(t * 0.5)))
            self.temp = 45 + (15 * math.sin(t * 0.3))
            self.controller_temp = 55 + (10 * math.sin(t * 0.2))
            self.soc = abs(75 + (20 * math.sin(t * 0.2)))
            self.v_pack = 52.5 + (2.1 * math.sin(t * 0.2))
            self.i_pack = 30 + (25 * math.sin(t * 0.8))
            
            # Simulate GPIO data (blinkers)
            blink_state = int(t * 2) % 2 == 0
            if int(t) % 10 < 5:
                self.turn_left = blink_state
                self.turn_right = False
            else:
                self.turn_left = False
                self.turn_right = blink_state
                
            gears = ["P", "R", "N", "D"]
            self.gear = gears[int(t / 8) % len(gears)]