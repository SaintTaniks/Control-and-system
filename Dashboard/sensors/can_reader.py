# dashboard/sensors/can_reader.py
"""
=============================================
 CAN Bus Reader for Vehicle Dashboard
=============================================

Encapsulates CAN interface setup and decoding of vehicle messages:
- Speed, gear, controller temp, blinkers
- Battery telemetry: pack voltage (V), current (A), state-of-charge (%)

Requires: python-can  (pip install python-can)

Integration (example)
---------------------
from sensors.can_reader import CANReader, CANConfig

cfg = CANConfig(
    channel="can0", bustype="socketcan", bitrate=500000,
    id_speed=0x123, id_gear=0x124, id_temp=0x125, id_blink=0x126,
    id_batt_soc=0x180, id_batt_v=0x181, id_batt_i=0x182
)
reader = CANReader(cfg)

data = reader.poll()
if data.soc is not None:
    print("SOC:", data.soc, "%")
if data.v_pack is not None and data.i_pack is not None:
    print("Power ~", data.v_pack * data.i_pack, "W")

Notes
-----
- All scaling shown is EXAMPLE ONLY. Replace with your DBC mappings.
- Endianness and signedness matter! (big-endian vs little-endian; signed current).
"""

from __future__ import annotations
import typing as t

try:
    import can
except ImportError:
    can = None


class CANConfig:
    """CAN IDs and scaling functions for your vehicle signals."""
    def __init__(
        self,
        *,
        channel: str = "can0",
        bustype: str = "socketcan",
        bitrate: int = 500000,
        # Drivetrain / status
        id_speed: int = 0x123,
        id_gear: int = 0x124,
        id_temp: int = 0x125,
        id_blink: int = 0x126,
        # Battery telemetry (set to your real IDs)
        id_batt_soc: int | None = 0x180,
        id_batt_v: int | None = 0x181,
        id_batt_i: int | None = 0x182,
        # Optional combined frame (if your BMS packs V/I/SOC together)
        id_batt_combo: int | None = None,
    ):
        self.channel = channel
        self.bustype = bustype
        self.bitrate = bitrate

        self.id_speed = id_speed
        self.id_gear = id_gear
        self.id_temp = id_temp
        self.id_blink = id_blink

        self.id_batt_soc = id_batt_soc
        self.id_batt_v = id_batt_v
        self.id_batt_i = id_batt_i
        self.id_batt_combo = id_batt_combo

    # ------------- Decoders (EDIT to match your DBC) -------------
    # Speed: 2 bytes big-endian; 0.01 km/h per unit
    def decode_speed(self, data: bytes) -> float:
        if len(data) < 2:
            return 0.0
        raw = int.from_bytes(data[0:2], "big", signed=False)
        return raw * 0.01

    # Gear: 1 byte enum
    def decode_gear(self, data: bytes) -> str:
        if not data:
            return "N"
        mapping = {0: "P", 1: "R", 2: "N", 3: "D"}
        return mapping.get(data[0], "N")

    # Controller temp: 1 byte °C
    def decode_temp(self, data: bytes) -> float:
        return float(data[0]) if data else 25.0

    # Blinkers: bit0=left, bit1=right
    def decode_blinkers(self, data: bytes) -> tuple[bool, bool]:
        if not data:
            return False, False
        b = data[0]
        return bool(b & 0x01), bool(b & 0x02)

    # Battery SOC: 1 byte => 0.5% per unit (EXAMPLE)
    def decode_batt_soc(self, data: bytes) -> float:
        if not data:
            return 0.0
        return min(100.0, max(0.0, data[0] * 0.5))

    # Battery voltage: 2 bytes big-endian; 0.1 V per unit (EXAMPLE)
    def decode_batt_voltage(self, data: bytes) -> float:
        if len(data) < 2:
            return 0.0
        raw = int.from_bytes(data[0:2], "big", signed=False)
        return raw * 0.1

    # Battery current: 2 bytes big-endian; signed; 0.1 A per unit (EXAMPLE)
    # Convention: positive = discharge, negative = charge (adjust to BMS)
    def decode_batt_current(self, data: bytes) -> float:
        if len(data) < 2:
            return 0.0
        raw = int.from_bytes(data[0:2], "big", signed=True)
        return raw * 0.1

    # If your BMS sends combined frame (example layout):
    # Byte0: SOC_raw (0.5%/LSB)
    # Byte1-2: V_pack (0.1V/LSB, big-endian)
    # Byte3-4: I_pack (0.1A/LSB, big-endian, signed)
    def decode_batt_combo(self, data: bytes) -> tuple[float | None, float | None, float | None]:
        if len(data) < 5:
            return None, None, None
        soc = self.decode_batt_soc(data[0:1])
        v = self.decode_batt_voltage(data[1:3])
        i = self.decode_batt_current(data[3:5])
        return soc, v, i


class CANData(t.NamedTuple):
    """Latest decoded values; fields are None if not updated this poll."""
    # Drivetrain / status
    speed: t.Optional[float] = None      # km/h
    gear: t.Optional[str] = None
    temp: t.Optional[float] = None       # °C
    turn_left: t.Optional[bool] = None
    turn_right: t.Optional[bool] = None
    # Battery
    soc: t.Optional[float] = None        # %
    v_pack: t.Optional[float] = None     # V
    i_pack: t.Optional[float] = None     # A (sign convention per BMS)


class CANReader:
    def __init__(self, config: CANConfig):
        if can is None:
            raise RuntimeError("python-can not installed; please `pip install python-can`.")
        self.cfg = config
        try:
            self.bus = can.interface.Bus(
                channel=config.channel,
                bustype=config.bustype,
                bitrate=config.bitrate
            )
            print(f"[can_reader] CAN bus up on {config.channel} ({config.bitrate} bps)")
        except Exception as ex:
            raise RuntimeError(f"[can_reader] Failed to open CAN: {ex}")

    def poll(self, max_msgs: int = 32, timeout: float = 0.0) -> CANData:
        """
        Non-blocking poll. Reads up to `max_msgs` messages from the bus and
        returns a CANData with the latest values seen in this call.
        """
        values: dict[str, t.Any] = {}
        cnt = 0
        try:
            msg = self.bus.recv(timeout=timeout)
            while msg and cnt < max_msgs:
                arb = msg.arbitration_id
                data = bytes(msg.data)

                if arb == self.cfg.id_speed:
                    values["speed"] = self.cfg.decode_speed(data)

                elif arb == self.cfg.id_gear:
                    values["gear"] = self.cfg.decode_gear(data)

                elif arb == self.cfg.id_temp:
                    values["temp"] = self.cfg.decode_temp(data)

                elif arb == self.cfg.id_blink:
                    left, right = self.cfg.decode_blinkers(data)
                    values["turn_left"], values["turn_right"] = left, right

                # Battery frames
                elif self.cfg.id_batt_combo is not None and arb == self.cfg.id_batt_combo:
                    soc, v, i = self.cfg.decode_batt_combo(data)
                    if soc is not None:
                        values["soc"] = soc
                    if v is not None:
                        values["v_pack"] = v
                    if i is not None:
                        values["i_pack"] = i

                else:
                    if self.cfg.id_batt_soc is not None and arb == self.cfg.id_batt_soc:
                        values["soc"] = self.cfg.decode_batt_soc(data)
                    if self.cfg.id_batt_v is not None and arb == self.cfg.id_batt_v:
                        values["v_pack"] = self.cfg.decode_batt_voltage(data)
                    if self.cfg.id_batt_i is not None and arb == self.cfg.id_batt_i:
                        values["i_pack"] = self.cfg.decode_batt_current(data)

                msg = self.bus.recv(timeout=0.0)
                cnt += 1
        except Exception:
            # swallow transient CAN errors; keep last known values upstream
            pass

        return CANData(**values)
