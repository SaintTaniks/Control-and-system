"""
=============================================
 Battery Monitor (INA219 / INA260 over I²C)
=============================================

Provides a unified interface to read pack voltage (V), current (A),
power (W), and estimate state-of-charge (SOC %) from popular Adafruit
INA219/INA260 breakouts. Uses CircuitPython libraries if available.

Features
--------
- Auto-detect INA219 or INA260 (or force one).
- Light low-pass filtering to reduce jitter.
- SOC estimation:
  * OCV (open-circuit voltage) lookup table with linear interpolation.
  * Optional coulomb counting (requires nominal Wh and dt updates).
- Graceful fallback if hardware/libs are missing (returns None values).

Dependencies
------------
pip install adafruit-circuitpython-ina219 adafruit-circuitpython-ina260

Usage
-----
from sensors.battery_monitor import BatteryMonitor, BatteryConfig, OCVTable

cfg = BatteryConfig(
    series_cells=10,
    nominal_wh=500.0,
    ocv_table=OCVTable.lithium_ion_nmc(),  # or .lithium_ion_lfp()
)
mon = BatteryMonitor(cfg)  # auto-detect INA219/INA260

# In your main loop:
v, i, p, soc = mon.read(dt=0.1)  # dt in seconds (for coulomb counter)
print(v, i, p, soc)
"""

from __future__ import annotations
from dataclasses import dataclass
from typing import Iterable, List, Optional, Tuple
import math

# Optional imports; we handle None gracefully
try:
    import board
    import busio
except Exception:
    board = None
    busio = None

try:
    from adafruit_ina219 import INA219 as _INA219
except Exception:
    _INA219 = None

try:
    from adafruit_ina260 import INA260 as _INA260
except Exception:
    _INA260 = None


# ---------- Helpers ----------
def _lin_interp(x: float, x0: float, y0: float, x1: float, y1: float) -> float:
    if x1 == x0:
        return (y0 + y1) / 2.0
    t = max(0.0, min(1.0, (x - x0) / (x1 - x0)))
    return y0 + t * (y1 - y0)


class LowPass:
    """1st-order low-pass filter."""
    def __init__(self, alpha: float, init: float = 0.0):
        self.alpha = max(0.0, min(1.0, alpha))
        self.y = float(init)

    def step(self, x: float) -> float:
        self.y += self.alpha * (x - self.y)
        return self.y


# ---------- OCV Tables ----------
class OCVTable:
    """
    Pack-level or per-cell OCV table in Volts -> SOC%.
    Provide per-cell OCV; we'll multiply by series_cells for pack-level compare.
    Values should be under *rest* (no load).
    """
    def __init__(self, points: Iterable[Tuple[float, float]]):
        # points: list[(volts_per_cell, soc_percent)]
        self.points = sorted(points, key=lambda p: p[0])

    def soc_from_voltage(self, v_cell: float) -> float:
        pts = self.points
        if not pts:
            return 0.0
        if v_cell <= pts[0][0]:
            return max(0.0, min(100.0, pts[0][1]))
        if v_cell >= pts[-1][0]:
            return max(0.0, min(100.0, pts[-1][1]))
        for (x0, y0), (x1, y1) in zip(pts[:-1], pts[1:]):
            if x0 <= v_cell <= x1:
                return max(0.0, min(100.0, _lin_interp(v_cell, x0, y0, x1, y1)))
        return 0.0

    @staticmethod
    def lithium_ion_nmc() -> "OCVTable":
        # Rough per-cell OCV (V) vs SOC (%) for typical NMC chemistry
        # (You should replace with your pack's characterization if available.)
        pts = [
            (3.20, 0), (3.35, 5), (3.50, 10), (3.60, 20),
            (3.70, 35), (3.75, 45), (3.80, 55), (3.85, 65),
            (3.90, 75), (4.00, 90), (4.10, 97), (4.20, 100),
        ]
        return OCVTable(pts)

    @staticmethod
    def lithium_ion_lfp() -> "OCVTable":
        # Very flat mid-band typical of LFP; again, tune for your cells.
        pts = [
            (3.10, 0), (3.20, 10), (3.25, 20), (3.30, 40),
            (3.32, 60), (3.35, 80), (3.38, 90), (3.40, 95),
            (3.45, 100),
        ]
        return OCVTable(pts)


# ---------- Configuration ----------
@dataclass
class BatteryConfig:
    i2c_scl: Optional[object] = None  # if None, auto board.SCL
    i2c_sda: Optional[object] = None  # if None, auto board.SDA
    address: Optional[int] = None     # I2C address; None = default
    prefer: Optional[str] = None      # "INA219" | "INA260" | None (auto)
    series_cells: int = 1             # S count for your pack
    nominal_wh: float = 0.0           # pack energy for coulomb counting (Wh)
    ocv_table: Optional[OCVTable] = None  # per-cell OCV mapping, optional
    # Filters (0=no filter, 1=heavy)
    alpha_v: float = 0.2
    alpha_i: float = 0.2
    alpha_p: float = 0.2
    alpha_soc: float = 0.1
    # Coulomb counter config
    enable_coulomb_count: bool = False
    soc_init: float = 60.0  # starting SOC % if coulomb counting is enabled


# ---------- Battery Monitor ----------
class BatteryMonitor:
    """
    Unified interface: read() -> (volts, amps, watts, soc%)
    If hardware or libs are missing, returns (None, None, None, None).
    """
    def __init__(self, config: BatteryConfig):
        self.cfg = config
        self._dev = None   # underlying INA219/INA260 instance
        self._kind = None  # "INA219" or "INA260"

        self._f_v = LowPass(self.cfg.alpha_v, 0.0)
        self._f_i = LowPass(self.cfg.alpha_i, 0.0)
        self._f_p = LowPass(self.cfg.alpha_p, 0.0)
        self._f_soc = LowPass(self.cfg.alpha_soc, self.cfg.soc_init)

        # Coulomb counter state (Wh used; positive current = discharge)
        self._soc_cc = float(self.cfg.soc_init)
        self._wh_remaining = (self.cfg.nominal_wh * (self._soc_cc / 100.0)) if self.cfg.nominal_wh > 0 else 0.0

        self._init_i2c_device()

    # ----- Public API -----
    def read(self, dt: Optional[float] = None) -> Tuple[Optional[float], Optional[float],
                                                        Optional[float], Optional[float]]:
        """
        Returns (volts, amps, watts, soc_percent).
        dt is the elapsed time in seconds since last call; required for coulomb counting.
        """
        if not self._dev:
            return None, None, None, None

        try:
            if self._kind == "INA219":
                v = float(self._dev.bus_voltage)
                # INA219 current is in mA (per CircuitPython lib); convert to A
                i = float(getattr(self._dev, "current", 0.0)) / 1000.0
                p = v * i
            elif self._kind == "INA260":
                v = float(self._dev.voltage)
                i = float(self._dev.current) / 1000.0  # INA260 current in mA
                p = v * i
            else:
                return None, None, None, None
        except Exception:
            return None, None, None, None

        # Smooth
        v = self._f_v.step(v)
        i = self._f_i.step(i)
        p = self._f_p.step(p)

        # SOC: OCV (if available) + optional coulomb counting blend
        soc = None
        if self.cfg.ocv_table:
            v_cell = v / max(1, self.cfg.series_cells)
            soc_ocv = self.cfg.ocv_table.soc_from_voltage(v_cell)
            soc = soc_ocv

        if self.cfg.enable_coulomb_count and dt is not None and self.cfg.nominal_wh > 0.0:
            # Energy change in this interval (Wh). Convention: +i discharging.
            wh_delta = (v * i) * (dt / 3600.0)
            self._wh_remaining = max(0.0, min(self.cfg.nominal_wh, self._wh_remaining - wh_delta))
            soc_cc = (self._wh_remaining / self.cfg.nominal_wh) * 100.0
            self._soc_cc = soc_cc if 0.0 <= soc_cc <= 100.0 else self._soc_cc

            if soc is None:
                soc = self._soc_cc
            else:
                # Blend OCV and CC (tune weights as you prefer)
                soc = 0.6 * soc + 0.4 * self._soc_cc

        # Final SOC smoothing & bounds
        if soc is not None:
            soc = self._f_soc.step(max(0.0, min(100.0, soc)))

        return v, i, p, soc

    # ----- Internals -----
    def _init_i2c_device(self) -> None:
        if not (board and busio):
            print("[battery_monitor] board/busio not available; I²C disabled.")
            return
        try:
            scl = self.cfg.i2c_scl or board.SCL
            sda = self.cfg.i2c_sda or board.SDA
            i2c = busio.I2C(scl, sda)

            # Force selection if requested
            if self.cfg.prefer == "INA219" and _INA219:
                self._dev = _INA219(i2c, addr=self.cfg.address) if self.cfg.address else _INA219(i2c)
                self._kind = "INA219"
                print("[battery_monitor] Using INA219 at auto addr" if self.cfg.address is None
                      else f"[battery_monitor] Using INA219 at 0x{self.cfg.address:02X}")
                return
            if self.cfg.prefer == "INA260" and _INA260:
                self._dev = _INA260(i2c, address=self.cfg.address) if self.cfg.address else _INA260(i2c)
                self._kind = "INA260"
                print("[battery_monitor] Using INA260 at auto addr" if self.cfg.address is None
                      else f"[battery_monitor] Using INA260 at 0x{self.cfg.address:02X}")
                return

            # Auto-detect: try INA219 first, then INA260
            if _INA219:
                try:
                    self._dev = _INA219(i2c, addr=self.cfg.address) if self.cfg.address else _INA219(i2c)
                    self._kind = "INA219"
                    print("[battery_monitor] Auto-detected INA219")
                    return
                except Exception:
                    self._dev = None

            if _INA260:
                try:
                    self._dev = _INA260(i2c, address=self.cfg.address) if self.cfg.address else _INA260(i2c)
                    self._kind = "INA260"
                    print("[battery_monitor] Auto-detected INA260")
                    return
                except Exception:
                    self._dev = None

            print("[battery_monitor] No INA219/INA260 detected.")
        except Exception as ex:
            print(f"[battery_monitor] I²C init failed: {ex}")
            self._dev = None
            self._kind = None
