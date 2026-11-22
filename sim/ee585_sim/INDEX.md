folder: sim/ee585_sim
purpose: MuJoCo provider, kinematics, and the loopback broker
files:
- __init__.py — package marker; deps: none
- glsetup.py — set MUJOCO_GL before mujoco is imported; deps: none; io: global state
  - def configure() -> str | None
  - MUJOCO_GL: str | None
- broker.py — aMQTT broker on 127.0.0.1:1883; deps: none; io: network
  - CONFIG: dict[str, Any]
  - def main() -> None
- maps.py — stub for map yaml, landmarks, and MJCF; deps: none
- kinematics.py — stub for the closed-form unicycle; deps: none
- imu.py — stub for analytic specific force and gyro; deps: none
- projection.py — stub for pinhole projections; deps: none
- follower.py — stub for pure pursuit; deps: none
- model.py — stub for MJCF compile, qpos, and render; deps: none
- provider_mj.py — stub for the live provider CLI; deps: none
