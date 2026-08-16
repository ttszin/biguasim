"""
Vehicle profiles for ArduPilot JSON SITL integration with BiguaSim.

Each VehicleProfile maps BiguaSim actuator indices to ArduPilot PWM channels
and specifies the correct conversion for each actuator type.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Callable


# ---------------------------------------------------------------------------
# PWM converter factories
# ---------------------------------------------------------------------------

def _bipolar_pwm(min_cmd: float, max_cmd: float) -> Callable[[float], float]:
    """PWM [1100, 1900] → [min_cmd, max_cmd]. Below 1100 → 0 (disarmed)."""
    def convert(pwm_val: float) -> float:
        if pwm_val < 1100:
            return 0.0
        pwm_val = min(1900.0, float(pwm_val))
        t = (pwm_val - 1100.0) / 800.0  # 0..1
        return min_cmd + t * (max_cmd - min_cmd)
    return convert


def _unipolar_pwm(max_cmd: float) -> Callable[[float], float]:
    """PWM [1000, 2000] → [0, max_cmd]. Below 1100 → 0 (disarmed)."""
    def convert(pwm_val: float) -> float:
        if pwm_val < 1100:
            return 0.0
        pwm_val = min(2000.0, float(pwm_val))
        t = (pwm_val - 1000.0) / 1000.0  # 0..1
        return t * max_cmd
    return convert


def _angle_pwm(min_deg: float, max_deg: float) -> Callable[[float], float]:
    """PWM [1100, 1900] → [min_deg, max_deg] degrees."""
    return _bipolar_pwm(min_deg, max_deg)


def _rpm_pwm(min_rpm: float, max_rpm: float) -> Callable[[float], float]:
    """PWM [1100, 1900] → [min_rpm, max_rpm]."""
    return _bipolar_pwm(min_rpm, max_rpm)


# ---------------------------------------------------------------------------
# VehicleProfile dataclass
# ---------------------------------------------------------------------------

@dataclass
class VehicleProfile:
    """All configuration needed to bridge one BiguaSim vehicle to ArduPilot SITL."""

    name: str
    """BiguaSim agent_type string."""

    num_motors: int
    """Number of actuators sent to BiguaSim env.step()."""

    motor_mapping: list
    """motor_mapping[i] = ArduPilot PWM channel index for sim actuator i."""

    motor_signs: list
    """±1 per actuator to flip thrust direction without changing wiring."""

    pwm_converters: list
    """Per-actuator callables: pwm_val (float) → command value."""

    control_abstraction: str
    """BiguaSim control abstraction string passed to the scenario config."""

    ardupilot_vehicle: str
    """ArduPilot vehicle type: 'ArduSub', 'ArduCopter', or 'Rover'."""

    sitl_args: str
    """Suggested sim_vehicle.py invocation for documentation purposes."""

    include_depth_sensor: bool
    """Add DepthSensor to auto-built scenario (enables pressure field in JSON)."""

    warmup_frames: int = 1700
    """Number of frames to hold motors at zero on startup."""


# ---------------------------------------------------------------------------
# VEHICLE_REGISTRY
# ---------------------------------------------------------------------------

_sub_bipolar = _bipolar_pwm(-278.9, 278.9)
_heavy_bipolar = _bipolar_pwm(-278.9, 278.9)

VEHICLE_REGISTRY: dict[str, VehicleProfile] = {

    "BlueROV2": VehicleProfile(
        name="BlueROV2",
        num_motors=6,
        # sim actuators r1..r6 ← AP PWM channels MOT6,5,2,1,4,3
        motor_mapping=[5, 4, 1, 0, 3, 2],
        # All six flipped from [1]*6, via an isolated dynamics-only
        # simulation (no SITL) reproducing ArduSub's own SUB_FRAME_VECTORED
        # per-motor factor table (AP_Motors6DOF.cpp), mapped through
        # motor_mapping above and fed into
        # HexaCopterFiveDoF._compute_body_wrench(): with the original
        # [1]*6, ArduSub's "forward" command produced *backward* thrust and
        # its "ascend" command produced *downward* thrust -- a full sign
        # inversion on the tested axes. This would explain a full day of
        # position/depth-hold overshoot that no PSC gain change (P halved,
        # I cut, twice, plus WP_SPD) could fix: a sign-inverted closed loop
        # can't be tuned into stability by adjusting gain magnitude alone.
        # Flipping all six made the isolated simulation self-consistent
        # (forward -> forward force, ascend -> upward force, lateral/yaw
        # inverted together). NOT CONFIRMED LIVE: the one live end-to-end
        # test after this change showed the vehicle tumbling (roll/yaw
        # 40-65 deg/s, quaternion near 180 deg) and drifting away from the
        # target rather than holding -- unclear yet whether that means this
        # fix is wrong/incomplete, or a separate pre-existing rotation
        # instability (see aerial-autonomy-stack's biguasim_bridge/
        # README.md, "bug #9") is confounding the result. Needs a test that
        # isolates translation from rotation before trusting this value.
        motor_signs=[-1, -1, -1, -1, -1, -1],
        pwm_converters=[_sub_bipolar] * 6,
        control_abstraction="cmd_motor_speeds",
        ardupilot_vehicle="ArduSub",
        sitl_args=(
            "sim_vehicle.py -v ArduSub -L RATBeach --console --map "
            "-f JSON:127.0.0.1 --add-param-file=bluerov2.parm"
        ),
        include_depth_sensor=True,
    ),

    "BlueROVHeavy": VehicleProfile(
        name="BlueROVHeavy",
        num_motors=8,
        # 4 vertical (MOT1-4) + 4 horizontal (MOT5-8)
        motor_mapping=[0, 1, 2, 3, 4, 5, 6, 7],
        motor_signs=[1, 1, 1, 1, 1, 1, 1, 1],
        pwm_converters=[_heavy_bipolar] * 8,
        control_abstraction="cmd_motor_speeds",
        ardupilot_vehicle="ArduSub",
        sitl_args=(
            "sim_vehicle.py -v ArduSub -L RATBeach --console --map "
            "-f JSON:127.0.0.1 --add-param-file=bluerovheavy.parm"
        ),
        include_depth_sensor=True,
    ),

    "BlueBoat": VehicleProfile(
        name="BlueBoat",
        num_motors=2,
        # port motor ← MOT1, starboard ← MOT3
        motor_mapping=[0, 2],
        motor_signs=[1, 1],
        pwm_converters=[
            _bipolar_pwm(-295.75, 295.75),  # port
            _bipolar_pwm(-288.08, 288.08),  # starboard
        ],
        control_abstraction="cmd_motor_speeds",
        ardupilot_vehicle="Rover",
        sitl_args=(
            "sim_vehicle.py -v Rover -L RATBeach --console --map "
            "-f JSON:127.0.0.1"
        ),
        include_depth_sensor=False,
        warmup_frames=200,
    ),

    "DjiMatrice": VehicleProfile(
        name="DjiMatrice",
        num_motors=4,
        motor_mapping=[2, 0, 3, 1],
        motor_signs=[1, 1, 1, 1],
        pwm_converters=[_unipolar_pwm(592.4)] * 4,
        control_abstraction="cmd_motor_speeds",
        ardupilot_vehicle="ArduCopter",
        sitl_args=(
            "sim_vehicle.py -v ArduCopter -L RATBeach --console --map "
            "-f quadx --model JSON:127.0.0.1 --no-mavproxy"
        ),
        include_depth_sensor=False,
        warmup_frames=500,
    ),

    "TorpedoAUV": VehicleProfile(
        name="TorpedoAUV",
        num_motors=5,
        # actuators 0-3 are fins (angles), actuator 4 is propeller RPM
        motor_mapping=[0, 1, 2, 3, 4],
        motor_signs=[1, 1, 1, 1, 1],
        pwm_converters=[
            _angle_pwm(-90.0, 90.0),  # fin 0
            _angle_pwm(-90.0, 90.0),  # fin 1
            _angle_pwm(-90.0, 90.0),  # fin 2
            _angle_pwm(-90.0, 90.0),  # fin 3
            _rpm_pwm(-1525.0, 1525.0),  # propeller RPM
        ],
        control_abstraction="cmd_rudders_sterns_motor_speed",
        ardupilot_vehicle="ArduSub",
        sitl_args=(
            "sim_vehicle.py -v ArduSub -L RATBeach --console --map "
            "-f JSON:127.0.0.1 --add-param-file=torpedo.parm"
        ),
        include_depth_sensor=True,
    ),
}
