import asyncio
import pytest

from dsos.control.backends.mavsdk_backend import MAVSDKBackend


class FakeSystem:
    def __init__(self):
        self.action = self
        self.offboard = self
        self.telemetry = self
        self.core = self
        self.mavlink_direct = self
        self.calls = []

    async def arm(self):
        self.calls.append("arm")
        raise RuntimeError("command denied")

    async def arm_force(self):
        self.calls.append("arm_force")

    async def takeoff(self):
        self.calls.append("takeoff")
        raise RuntimeError("command denied")

    async def set_takeoff_altitude(self, altitude):
        self.calls.append(("set_takeoff_altitude", altitude))

    async def disarm(self):
        self.calls.append("disarm")

    async def land(self):
        self.calls.append("land")

    async def return_to_launch(self):
        self.calls.append("return_to_launch")

    async def set_position_ned(self, pos):
        self.calls.append(("set_position_ned", pos))

    async def start(self):
        self.calls.append("offboard_start")

    async def set_velocity_ned(self, vel):
        self.calls.append(("set_velocity_ned", vel))

    async def connection_state(self):
        async def gen():
            yield type("Conn", (), {"is_connected": True})()
        return gen()

    async def position(self):
        async def gen():
            yield type("Pos", (), {"relative_altitude_m": 0.0, "latitude_deg": 0.0, "longitude_deg": 0.0, "absolute_altitude_m": 0.0})()
        return gen()

    async def battery(self):
        async def gen():
            yield type("Bat", (), {"remaining_percent": 1.0, "voltage_v": 12.0})()
        return gen()

    async def flight_mode(self):
        async def gen():
            yield "GUIDED"
        return gen()

    async def armed(self):
        async def gen():
            yield True
        return gen()

    async def in_air(self):
        async def gen():
            yield False
        return gen()

    async def health(self):
        async def gen():
            yield type("Health", (), {"is_global_position_ok": True, "is_local_position_ok": True})()
        return gen()

    async def position_velocity_ned(self):
        async def gen():
            yield type("PosVel", (), {"position": type("Pos", (), {"east_m": 0.0, "north_m": 0.0}), "velocity": type("Vel", (), {"east_m_s": 0.0, "north_m_s": 0.0, "down_m_s": 0.0})})()
        return gen()


@pytest.mark.asyncio
async def test_takeoff_falls_back_to_guided_mode_when_arm_denied(monkeypatch):
    backend = MAVSDKBackend({"vehicle_id": "drone_1"})
    fake_system = FakeSystem()
    backend.systems["drone_1"] = fake_system
    backend.vehicles["drone_1"] = type("VehicleState", (), {"is_connected": True, "flight_mode": "GUIDED", "battery_remaining": 100.0, "is_armed": False, "position": [0.0, 0.0, 0.0], "velocity": [0.0, 0.0, 0.0], "orientation": [0.0, 0.0, 0.0], "land_state": "landed"})()

    calls = []

    async def fake_set_mode(*args, **kwargs):
        calls.append("guided")
        return True

    monkeypatch.setattr(backend, "_set_vehicle_mode", fake_set_mode)

    assert await backend.arm("drone_1") is True
    assert calls == ["guided"]
    assert fake_system.calls == ["arm", "arm_force"]
