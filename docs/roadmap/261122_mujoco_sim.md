# Roadmap: MuJoCo Simulation Environment and Streaming Contract

Scope: everything on the data-provider side of the boundary, plus the Python library that the algorithm side uses to receive data. The MSCKF algorithm, its baselines, the dropout harness, and the evaluation metrics are NOT in this roadmap.

This file replaces the provider, the physics, the rendering, and the install path of `docs/roadmap/261120_sim_environment.md`. It does not replace that file. Do not edit that file. An executing agent implements this file only and does not need the Gazebo file open.

Audience: an executing agent with no prior context. Everything it needs is in this file. Do not re-open decisions in section 2.

---

## 0. Rules for the executing agent

- Follow `AGENTS.md` and the `folder-index` skill (`.cursor/skills/folder-index/SKILL.md`). Every folder that directly contains code gets an `INDEX.md` in the same change as the code. `tests/` folders and generated files are exempt.
- Simple and direct. Do not add abstractions, plugin systems, or configuration beyond what is written here.
- Do NOT commit or push. The owner decides when. When asked, use the `commit-helper` skill.
- Do NOT edit `docs/roadmap/261120_sim_environment.md`.
- Do NOT install ROS, Gazebo, `ros_gz`, or Mosquitto. Do not import `rclpy`, `sensor_msgs`, or `mujoco` from `ee585_stream`.
- Work phase by phase in the order of section 7. Do not start a phase before the previous phase's acceptance checks pass.
- Python is CPython 3.12, 64-bit, in a normal venv (no `--system-site-packages`). Dependencies come from pip. The same commands must be valid on Ubuntu, macOS (Apple Silicon and Intel), and Windows. Use `pathlib` in code. No apt, brew, or shell-only step is part of the library.
- If a MuJoCo or aMQTT call name in this file does not match the installed version, use the call that implements the stated behavior, and record the deviation in section 10. Do not change a number, frame, sign, topic, or schema in section 2.
- If the static accelerometer cross-check in section 2 does not read about `(0, 0, +9.81)`, stop and report. Do not negate the published specific force to make the check pass.
- Append to section 10 as you go. Do not edit other sections except to fix an error you can prove.
- Run pytest without xdist. Tests share a broker on port 1883.

---

## 1. Goal

Provide an environment in which a visual-inertial algorithm can be run against:

1. a live MuJoCo simulation, or
2. a replay of a recorded run (no MuJoCo process),

through one identical interface: a stream of timestamped records on MQTT. The algorithm never imports MuJoCo and never reads run files. When the provider stops, the algorithm stops producing new estimates.

The live viewer, when requested, is MuJoCo's Python window (`mujoco.viewer.launch_passive`). Images on the stream are rendered offscreen by `mujoco.Renderer`. No Gazebo GUI is involved. A window is not required to generate data.

---

## 2. Locked decisions

| Topic | Decision |
|---|---|
| Simulator | MuJoCo, PyPI package `mujoco>=3.2`, imported as `import mujoco`. World file is one MJCF XML document generated per map. |
| Platforms | CPython 3.12 venv + pip on Ubuntu, macOS (arm64 and x86_64), and Windows (x86_64). No ROS. No Gazebo. Ubuntu is not required. |
| Viewer | Optional. `--viewer` opens `mujoco.viewer.launch_passive` on the main thread. Closing the window does not end the run and does not stop rendering. Default is headless. |
| Images | `mujoco.Renderer` (classic rasterizer). Do not enable Filament. |
| GL | Linux: `MUJOCO_GL=egl`, and if that cannot create a context, `MUJOCO_GL=osmesa`. macOS and Windows: leave `MUJOCO_GL` unset (the default glfw / CGL / WGL context). Set the variable before the first `import mujoco`. A user-provided `MUJOCO_GL` is left as set and recorded in section 10. |
| Robot | Planar wheeled robot. The wheels are visual. Motion is a kinematic unicycle integrated in closed form (section 2.3), not MuJoCo contact and not `mj_step`. Ground truth is the full 6-DoF pose of that motion: roll = 0, pitch = 0, `z = 0.05` m, yaw free. |
| Scene | 3D (walls, obstacles, textured surfaces, landmarks), static. |
| Camera | ONE undistorted pinhole camera. No depth, no stereo. Square pixels, principal point at the image center. |
| Camera channels | Both, from the same pose and the same intrinsics: (a) the rendered mono8 image, with no pixel noise; (b) synthetic projections of known 3D landmarks, with pixel noise. |
| IMU | Analytic specific force and gyro (section 2.2), then independent Gaussian noise. Bias is identically zero: no turn-on bias, no constant bias draw, no bias random walk, no scale factor, no misalignment. MJCF `noise="0"` and `cutoff="0"`. The published sample is never copied from `data.sensordata`. |
| Maps | Variable between maps: geometry, path length, texture and feature density, lighting. Fixed: robot, intrinsics, IMU noise model, kinematic model. (Moving occluders: optional phase 8.) |
| Ground truth / dropout | Truth is a separate MQTT topic. Feature dropout is not applied in the simulator or the provider. A later harness may drop observations after the consumer yields them. |
| Boundary | The provider publishes a stream. The algorithm consumes it. Providers: MuJoCo live provider, replay provider. `ee585_stream` does not import `mujoco`. |
| Transport | MQTT. See the paragraph under this table. |
| Broker | aMQTT (`amqtt>=0.11`), pure Python, MQTT 3.1.1, `127.0.0.1:1883`, anonymous, no persistence. Started with `python -m ee585_sim.broker`. Mosquitto is not required. |
| Stream layout | ONE ordered `stream` topic. Calibration, scene, status, truth on separate topics. Prefix `ee585/v1/`. |
| Payload | Protobuf. All numeric fields `double`. Timestamps `int64` simulation nanoseconds. |
| Image | mono8 raw bytes, row-major, tightly packed. Row 0 is the top of the image. |
| Delivery | QoS 2 on `stream` and `truth`. Gapless `seq` on every record. Provider is paced. |
| Run ending | `end_of_stream` is metadata only; the algorithm never waits for it. Status in {completed, aborted, stalled}. Abort detected by MQTT Last Will. Stall timeout 5 s (wall clock, configurable). |
| Language | Python 3.12 for everything in this roadmap. The contract is language-neutral, so the algorithm side may differ. |
| Schema | `stream.proto` in section 5 is the wire schema. `schema_version` stays 1. Do not add fields. |

MQTT stays because the boundary is a stream of bytes, not a function call into the physics library. Live and replay publish the same protobuf payloads. Calibration is retained, so a consumer that connects slightly late still receives it. A provider killed with `kill -9` (or a hard process death on Windows) never sends MQTT `DISCONNECT`, and the broker delivers the Last Will as `Status.ABORTED`. The MSCKF imports `ee585_stream` only. It does not import `mujoco` or `ee585_sim` and does not open `runs/`. An in-process callback would put the estimator in the simulator process. ROS topics are not used.

Replacements relative to the Gazebo roadmap (the items above that are new; everything else in that contract is kept):

| Item | This roadmap |
|---|---|
| Physics and scene | MuJoCo MJCF, stepped only for kinematics and rendering (`mj_forward`) |
| Visualizer and images | Python viewer and `mujoco.Renderer` |
| Install | venv + pip on Ubuntu, macOS, and Windows |
| Robot motion | Closed-form unicycle. A 4 s command `v = 0.5`, `ω = 0` moves the robot 2 m within `1e-6` m |
| IMU | Formula and noise in `imu.py`. Engine accelerometer is a static cross-check |
| Clock | Provider integer nanoseconds, not `data.time` |
| Broker process | `amqtt`, loopback |
| Roll, pitch, height | Exactly 0 rad, 0 rad, and 0.05 m |

### 2.1 Frames, units, quaternions

Written into `stream.proto` comments as well:

- World W: ENU, z up. Body B: FLU, origin at the IMU. Camera C: optical frame (x right, y down, z forward).
- Rotations are Hamilton unit quaternions `(w, x, y, z)`, with `w >= 0` when a sign choice exists. `q_WB` rotates body vectors into world: `v_W = R(q_WB) v_B`.
- `R(q)` for a unit quaternion, the matrix used everywhere in this roadmap:

```text
R = [[1 - 2(y² + z²),  2(xy - wz),      2(xz + wy)],
     [2(xy + wz),      1 - 2(x² + z²),  2(yz - wx)],
     [2(xz - wy),      2(yz + wx),      1 - 2(x² + y²)]]
```

- `p_BC, q_BC` is the pose of C in B: `x_B = R(q_BC) x_C + p_BC`.
- Default forward camera: `p_BC = (0.15, 0, 0.10)` and

```text
R_BC = [[0, 0, 1],
        [-1, 0, 0],
        [0, -1, 0]]
q_BC = (0.5, -0.5, 0.5, -0.5)
```

  The generator computes `q_BC` from `R_BC` and must match this quaternion within `1e-12`.
- Yaw about world +Z from the identity: `q_WB = (cos(yaw/2), 0, 0, sin(yaw/2))`. Positive yaw is a left turn.
- IMU gyro: rad/s, body frame. IMU accel: m/s², body frame, specific force `f_B = a_B - R(q_WB)^T g_W` with `g_W = (0, 0, -9.81)`. At rest `f_B = (0, 0, +9.81)`.
- Pinhole, `p_C = (X, Y, Z)`, `Z > 0`: `u = fx X / Z + cx`, `v = fy Y / Z + cy`. No distortion.
- Time: `t_ns` is the provider's simulation clock, integer nanoseconds, 0 at the start. Wall-clock time is used only for MQTT pacing and stall detection. Never publish a stamp derived from `data.time` or from `time.time()`.
- Pixel axes: `u` increases right, `v` increases down. `u = 0` is the left edge of column 0, `v = 0` is the top edge of row 0. Pixel `(col, row)` covers `[col, col+1) × [row, row+1)` and its center is `(col + 0.5, row + 0.5)`. `cx = width/2`, `cy = height/2` (for the default image, `(320, 240)`), on the corner shared by four pixels. The optical axis projects to `(cx, cy)`.

MuJoCo's world is z-up and its quaternion is `(w, x, y, z)`. That matches this section. MuJoCo's camera frame does not match C: a MuJoCo camera looks along its own −Z, with +X right and +Y up. The MJCF camera is oriented with `xyaxes` so that its look direction is body +X and its up direction is body +Z (section 7, phase 3). Projection math uses C and `R_BC`. It does not use the MuJoCo camera axes.

### 2.2 IMU model

Published gyro and specific force are computed here, then noise is added. `data.sensordata` is not the stream.

Symbols: interval length `Δt = 0.005` s (200 Hz). On the interval that ends at sample `t_ns > 0`, the commanded body twist `(v, ω)` is constant. `v` is forward speed in m/s (body +X). `ω` is yaw rate in rad/s (body +Z). `v_prev` is the forward speed of the previous interval (`0` on the first interval). `q_WB` is the pose at the end of the interval.

True values, before noise:

```text
ω_B = (0, 0, ω)
a_B = ( (v - v_prev) / Δt ,  ω * v ,  0 )
f_B = a_B - R(q_WB)^T g_W
```

For this robot `R(q_WB)` is a pure yaw, so `R^T g_W = g_W` and `f_B = (a_Bx, a_By, 9.81)`.

The sample at `t_ns = 0` is the stationary prior, not an interval: `ω_B = (0, 0, 0)`, `f_B = (0, 0, 9.81)`, then noise.

Noise, added after the true values, independently on each sample, in this order: gyro x, y, z, then accel x, y, z.

```text
σ_g = gyro_noise_density * sqrt(imu_rate_hz)    # 1.6968e-4 * sqrt(200) = 2.3996e-3 rad/s
σ_a = accel_noise_density * sqrt(imu_rate_hz)   # 2.0e-3 * sqrt(200) = 2.8284e-2 m/s²
z   = Generator(PCG64(map_seed + 2)).standard_normal(6)   # one generator for the run
gyro  = ω_B + σ_g * z[0:3]
accel = f_B + σ_a * z[3:6]
```

`map_seed + 2` is the IMU seed. The projection generator uses a different seed (section 2.4). Zero densities produce samples equal to `(ω_B, f_B)` with no other term. There is no bias state to initialize.

MJCF, for the cross-check only:

```xml
<sensor>
  <gyro name="gyro" site="imu" noise="0" cutoff="0"/>
  <accelerometer name="accel" site="imu" noise="0" cutoff="0"/>
</sensor>
```

Do not set `delay`, `interval`, or `nsample`. The site `imu` is at the base origin, identity orientation, so the site frame is B.

Engine cross-check (not the published sample): with `qvel = 0` and `qacc = 0`, evaluate sensors without integrating (try `mj_forward`, then `mj_fwdAcceleration` if the version needs it, then `mj_sensorAcc`, or the installed equivalents). The accelerometer must lie within `1e-4` of `(0, 0, +9.81)` on each axis and the gyro within `1e-6` of `(0, 0, 0)`. With yaw rate `0.3` rad/s written into `qvel` and `v = 0`, the gyro must lie within `1e-6` of `(0, 0, 0.3)`. Log the call sequence in section 10. A sign error is a stop condition (section 0).

### 2.3 Kinematics

The robot state is `(x, y, yaw)` in the world, plus constants `z = 0.05` m, roll = 0, pitch = 0. `z = 0.05` is the wheel radius: the IMU sits on the axle line.

Twist `(v, ω)` is held constant on each 5 ms IMU interval. Propagation by `τ` seconds (exact unicycle; use `τ = 0.005` at interval ends, and the fractional `τ` at a camera time inside the interval):

```text
if |ω| < 1e-12:
    x' = x + v * cos(yaw) * τ
    y' = y + v * sin(yaw) * τ
    yaw' = yaw
else:
    x' = x + (v/ω) * (sin(yaw + ωτ) - sin(yaw))
    y' = y + (v/ω) * (-cos(yaw + ωτ) + cos(yaw))
    yaw' = yaw + ωτ
```

Wrap `yaw` to `(-π, π]` only for logging readability. The quaternion uses the unwrapped angle's half-angle; both are valid if `cos` and `sin` are applied to the same value. Do not call `mj_step`. After writing `qpos`, call `mj_forward` so the camera and the body pose follow this state.

`qpos` layout is discovered from `jnt_qposadr`, not hardcoded. The three joints are nested so the slides stay world-aligned (phase 3). Joint damping is 0. There are no actuators.

The follower (phase 5) is evaluated at IMU stamps `t_k` and its command applies to the interval `(t_k, t_{k+1}]`. During the first `initial_rest_s` of simulation time the command is `(0, 0)` and the follower state does not ramp. `initial_rest_s = 1.0` is exactly `1_000_000_000` ns.

### 2.4 Camera model

Intrinsics, from `calibration.yaml`. These doubles are what `Calibration` publishes and what the projector uses:

```text
fx = fy = (width / 2) / tan(hfov_rad / 2)
cx = width / 2
cy = height / 2
fovy_deg = degrees(2 * atan((height / 2) / fy))
```

With the default yaml, `fx = fy = 320` within `1e-6`, and `fovy_deg ≈ 73.7397938`. Write `fovy` into MJCF from this formula. Do not also set `sensorsize`, `focal`, `focalpixel`, `principal`, or `principalpixel` (those override `fovy`). Reject a calibration in which `|fx - fy| > 1e-6` or the principal point is not the image center: the `fovy` camera cannot represent it.

Renderer: `mujoco.Renderer(model, height=height, width=width)`. `<visual><global offwidth offheight>` is at least that size. `renderer.update_scene(data, camera="cam")` then `rgb = renderer.render()`. The returned array is `(height, width, 3)` uint8, RGB, row 0 at the top. Do not flip it again. If the phase 4 centroid test shows the image upside down, flip once in the provider, log it in section 10, and keep the published convention (row 0 is the top).

mono8, per pixel, no noise, no blur, no distortion, no exposure:

```text
gray = 0.299 * R + 0.587 * G + 0.114 * B     # floats, R,G,B in 0..255
mono = uint8 clip(floor(gray + 0.5), 0, 255)
```

`mono8` length is `width * height`, row-major.

Clock. Physics display step is 1 ms only as a documentation value in MJCF (`timestep="0.001"`). Published stamps:

```text
t_imu(k) = k * 5_000_000                         # k = 0, 1, 2, ...
t_cam(k) = ((k * 1000 + 15) // 30) * 1_000_000   # integer arithmetic
```

So `t_cam` is 0, 33_000_000, 67_000_000, 100_000_000, … and `t_cam(30) = 1_000_000_000`. Camera gaps are only 33_000_000 or 34_000_000 ns. Mean rate over each second is 30 Hz. When an IMU stamp equals a camera stamp, the stream order is IMU, then image, then projections.

Projection of landmark centers is section 7, phase 4. It uses C, not the z-buffer. Pixel noise is applied only to that channel:

```text
σ = pixel_noise_sigma          # 0.5 px, each axis
generator = Generator(PCG64(map_seed + 1))   # one generator for the run
```

Within a frame, visible landmarks in ascending id, draw `u` noise then `v` noise. Invisible landmarks consume no draws. Noise does not change visibility. The rendered image is independent of this generator.

Lighting does not move the projection-vs-render check: landmark material is emissive white (`emission="1 1 1"`, `specular="0"`, `rgba="1 1 1 1"`). Headlight is off. The sun does not cast shadows (`castshadow="false"`). Rendered images are not required to be byte-identical across GL backends. The centroid test is the image check. Replay byte-identity is about stored payloads, not about rendering again.

### 2.5 Ground truth

- `TruthRecord.pose` at every IMU stamp, including 0: `p_WB = (x, y, 0.05)`, `q_WB` from yaw. No noise.
- `TruthProjections` at every camera stamp: the noiseless observations, same ids and order as the noisy `Projections` of that `t_ns`.
- `TruthRecord.seq` is its own gapless counter. At a shared `t_ns`, the truth pose is published before the truth projections.
- The algorithm's consumer does not subscribe to `truth` or `scene`.

---

## 3. Architecture

```
 provider_mj (one Python process, main thread)
   glsetup → mujoco
   kinematics + follower + imu.py + Renderer + Projector
   optional launch_passive
        |  MQTT
   aMQTT broker 127.0.0.1:1883
        |-- algorithm (ee585_stream.consumer only)
        |-- recorder
        +-- evaluator (truth and scene)

 replay provider --MQTT--> broker          (no mujoco import)
```

MQTT topics (prefix `ee585/v1/`):

| Topic | Content | QoS | Retained | Readers |
|---|---|---|---|---|
| `status` | `Status` | 1 | yes | everyone |
| `calibration` | `Calibration` | 1 | yes | algorithm, evaluator |
| `scene` | `Scene` (true landmark positions) | 1 | yes | evaluator ONLY |
| `stream` | `Record` | 2 | no | algorithm, recorder |
| `truth` | `TruthRecord` | 2 | no | evaluator, recorder ONLY |

The algorithm must never subscribe to `scene` or `truth`.

Run log (written by the recorder, read by the replay provider): directory `runs/<run_name>/` containing `meta.json`, `calibration.bin`, `scene.bin`, `stream.bin`, `truth.bin`. Each `.bin` is a sequence of `[uint32 little-endian length][payload bytes]` where the payload is the exact MQTT payload. Replay republishes those bytes. `runs/` is git-ignored.

There is no `cmd_vel` topic, no `/clock`, and no `ros_gz` bridge. The follower is a function called inside `provider_mj`.

---

## 4. Repository layout to create

Paths are relative to the `EE585_PROJECT` repository root.

```
stream/                          pure Python package, no mujoco import
  pyproject.toml
  INDEX.md
  ee585_stream/
    INDEX.md
    stream.proto
    stream_pb2.py                generated; commit it; regenerate with scripts/gen_proto.py
    link.py                      topics, encode/decode, MQTT connect with will, bounded publisher
    consumer.py
    recorder.py                  CLI
    replay.py                    CLI
  scripts/gen_proto.py           Python, so Windows does not need a shell script
  tests/
    fake_provider.py
sim/
  pyproject.toml                 depends on ee585_stream, mujoco, numpy, pillow, pyyaml, paho-mqtt, amqtt
  INDEX.md
  ee585_sim/
    INDEX.md
    glsetup.py                   MUJOCO_GL before import mujoco
    broker.py                    python -m ee585_sim.broker
    maps.py                      map yaml, landmarks, MJCF, textures
    kinematics.py                section 2.3
    imu.py                       section 2.2
    projection.py                section 7 phase 4, no mujoco import
    follower.py                  pure pursuit, no ROS
    model.py                     compile MJCF, write qpos, mj_forward, render, sensor cross-check
    provider_mj.py               CLI: loop, merge, MQTT
  config/calibration.yaml
  maps/
  templates/world.xml.in
  tests/
docs/usage.md                    written in phase 7
docs/roadmap/261120_sim_environment.md    do not edit
runs/                            git-ignored
.gitignore
```

`stream/pyproject.toml` depends on `paho-mqtt>=2.0` and `protobuf>=5`. Dev dependencies: `pytest>=8`, `grpcio-tools>=1.60`. Sim dependencies also include `mujoco>=3.2`, `numpy>=1.26`, `pillow>=10`, `pyyaml>=6`, `amqtt>=0.11`.

Install:

```text
python3.12 -m venv .venv
# Windows: py -3.12 -m venv .venv
# then activate, and:
pip install -e stream -e sim
```

`.gitignore`: `venv`, `.venv`, `__pycache__`, `runs/`, `*.egg-info`, `build`, `.pytest_cache`.

`scripts/gen_proto.py` runs `python -m grpc_tools.protoc` with the proto's directory as `-I` and `--python_out` into `ee585_stream/`. It uses `sys.executable`.

---

## 5. Contract: `stream.proto`

Create exactly this schema. Comments are part of the contract.

```proto
syntax = "proto3";
package ee585.v1;

message Vec3 { double x = 1; double y = 2; double z = 3; }
// Hamilton unit quaternion.
message Quat { double w = 1; double x = 2; double y = 3; double z = 4; }

// ---- stream topic ----
message ImuSample {
  Vec3 gyro_B  = 1;   // rad/s, body frame FLU, includes noise
  Vec3 accel_B = 2;   // m/s^2, body frame, SPECIFIC FORCE (reads +g up at rest), includes noise
}
message ImageFrame {
  uint32 width  = 1;
  uint32 height = 2;
  bytes  mono8  = 3;  // width*height bytes, row-major, no padding, row 0 is the top
}
message Observation {
  uint32 landmark_id = 1;  // stable across the run
  double u = 2;            // pixels
  double v = 3;            // pixels
}
// Noisy synthetic projections for the image with the same t_ns. Always sent, possibly empty.
message Projections { repeated Observation obs = 1; }
message EndOfStream {}

message Record {
  uint64 seq  = 1;         // 0,1,2,... gapless over the stream topic
  int64  t_ns = 2;         // simulation time, non-decreasing over the stream
  oneof payload {
    ImuSample    imu         = 10;
    ImageFrame   image       = 11;
    Projections  projections = 12;
    EndOfStream  end         = 13;
  }
}
// Order at equal t_ns: IMU, then image, then projections.

// ---- calibration topic (retained) ----
message Calibration {
  uint32 schema_version = 1;     // 1
  string map_name = 2;
  double imu_rate_hz = 3;
  double gyro_noise_density = 4;   // rad/s/sqrt(Hz); per-sample sigma = density*sqrt(imu_rate_hz)
  double accel_noise_density = 5;  // m/s^2/sqrt(Hz)
  Vec3   gravity_W = 6;            // (0,0,-9.81)
  uint32 width = 7;
  uint32 height = 8;
  double camera_rate_hz = 9;
  double fx = 10; double fy = 11; double cx = 12; double cy = 13;
  double pixel_noise_sigma = 14;   // pixels, per axis, applied to Projections only
  Vec3   p_BC = 15;
  Quat   q_BC = 16;
  double initial_rest_s = 17;      // robot is stationary for this long at the start
}

// ---- scene topic (retained, evaluator only) ----
message Landmark { uint32 id = 1; Vec3 p_W = 2; }
message Scene { repeated Landmark landmarks = 1; }

// ---- truth topic (evaluator and recorder only) ----
message Pose { Vec3 p_WB = 1; Quat q_WB = 2; }
// Noiseless projections, same landmark ids and order as the noisy Projections of that frame.
message TruthProjections { repeated Observation obs = 1; }
message TruthRecord {
  uint64 seq  = 1;               // own counter, gapless
  int64  t_ns = 2;
  oneof payload { Pose pose = 10; TruthProjections projections = 11; }
}

// ---- status topic (retained) ----
message Status {
  enum State { UNKNOWN = 0; ONLINE = 1; COMPLETED = 2; ABORTED = 3; }
  State  state = 1;
  int64  last_seq  = 2;          // -1 if unknown (the Last Will cannot know it)
  int64  last_t_ns = 3;          // -1 if unknown
  string detail = 4;
}
```

`config/calibration.yaml` (the executing agent writes this file; these are the values):

```yaml
schema_version: 1
imu:
  rate_hz: 200
  gyro_noise_density: 1.6968e-4   # rad/s/sqrt(Hz); sigma = density*sqrt(rate_hz) ≈ 2.400e-3
  accel_noise_density: 2.0e-3     # m/s^2/sqrt(Hz); sigma ≈ 2.828e-2
  gravity_W: [0.0, 0.0, -9.81]
camera:
  width: 640
  height: 480
  rate_hz: 30
  hfov_rad: 1.5707963             # fx = fy = (width/2)/tan(hfov/2) ≈ 320, cx = width/2, cy = height/2
  pixel_noise_sigma: 0.5          # projections only; the rendered image has none
  p_BC: [0.15, 0.0, 0.10]
  R_BC: [[0, 0, 1], [-1, 0, 0], [0, -1, 0]]
  min_range_m: 0.2
  max_range_m: 20.0
initial_rest_s: 1.0
```

`min_range_m` and `max_range_m` stay in the yaml. They are not proto fields. Bias is not a yaml key: the bias is zero by section 2.2, and a key would imply a free parameter.

`camera_rate_hz` is published as `30`. Stamps follow section 2.4, whose mean rate is 30 Hz. The field is the nominal rate, not `1 / median_gap`.

---

## 6. Operating rules for providers and consumers

Provider (live and replay):

- MQTT client keepalive = 2 s. Network loop in a background thread (`loop_start`). MuJoCo stays on the main thread.
- On connect: register the Last Will on `status`: `Status{state: ABORTED, last_seq: -1, last_t_ns: -1, detail: "last will"}`, QoS 1, retained. Then publish retained `Status{ONLINE}`, then retained `calibration` and `scene`.
- `seq` starts at 0 and increases by 1 per `Record` on `stream`. `TruthRecord.seq` is a separate gapless counter.
- Publishing is bounded: count publishes not yet acknowledged and block while the count exceeds 100.
- Pacing: `--speed X` (default 1.0) publishes record `r` no earlier than `wall_start + (t_ns(r) - t0_ns) / X` seconds. `--speed 0` means unpaced, limited only by the unacknowledged bound. The viewer uses this same clock. It does not sleep on its own.
- Clean end: publish `Record{end: EndOfStream}` (it takes the next `seq`), wait until all publishes are acknowledged, publish retained `Status{COMPLETED, last_seq, last_t_ns}`, then `disconnect()` normally so the Last Will is not sent.
- Self-failure (`Link.abort(detail)`, retained `Status{ABORTED}`, exit non-zero): renderer exception or wrong image shape; chassis footprint intersects an obstacle; the unacknowledged bound blocks for more than 2 s (`detail: "publisher backlog"`); broker connection lost. In-process generation has no dropped-sensor gap to detect. Still abort if a produced IMU gap is not exactly 5_000_000 ns or a camera stamp does not match section 2.4.

Broker, `python -m ee585_sim.broker`:

- Bind `127.0.0.1:1883` only. Anonymous connections. No persistence. No message-size cap below 2_000_000 bytes (a 640×480 mono8 record is about 307 KB).
- Starting config shape (change key names if the installed aMQTT requires it; do not change the bind address, anonymity, or QoS the clients request):

```python
config = {
    "listeners": {"default": {"type": "tcp", "bind": "127.0.0.1:1883"}},
    "auth": {"allow-anonymous": True},
}
```

- The broker is a separate process from the provider, so killing the provider does not kill the broker.
- If port 1883 is taken, exit with a clear error. Do not pick another port.

Consumer (`ee585_stream.consumer`):

- Subscribes to `status`, `calibration`, `stream`. Never to `scene` or `truth` unless constructed as the evaluator variant.
- Yields events in order: `Calibration`, then `Record`s, then exactly one `Terminal`.
- Checks on every record: `seq == last_seq + 1`. A smaller `seq` is a duplicate and is dropped silently. A larger `seq` is a gap. `t_ns` must be non-decreasing. A gap or an out-of-order stamp ends the run with `Terminal(reason=DATA_ERROR)`. Fail loudly. Never repair.
- Terminal reasons:
  - `COMPLETED`: status `COMPLETED` received AND the record with `seq == status.last_seq` has been yielded.
  - `ABORTED`: status `ABORTED` received. Drain already-queued records first, then terminate.
  - `STALLED`: status is `ONLINE`, at least one record was received, and no record arrived for `stall_timeout_s` (default 5.0 wall-clock seconds). Before the first record, the wait limit is `startup_timeout_s` (default 60.0).
  - `DATA_ERROR`: as above, also an undecodable payload.
  - The `EndOfStream` record is yielded like any other record. It is not required for termination.
- `Terminal` carries `reason`, `last_seq`, `last_t_ns`. `write_run_status(path, terminal)` writes them to a small JSON file. Estimates already produced remain valid after any terminal reason.

Estimator-time semantics: the consumer hands records to the algorithm one at a time in `seq` order. The algorithm uses only `t_ns` differences, never wall-clock, so output must be identical at any `--speed`.

Live provider loop (one timestamp at a time, union of IMU and camera stamps, ascending):

1. Propagate the kinematic state to `t` (section 2.3). Write `qpos` and the matching `qvel` (`v cos yaw`, `v sin yaw`, `ω`). `mj_forward`.
2. If `t` is an IMU stamp: build the IMU sample (section 2.2), publish `ImuSample`, publish `TruthRecord.pose`.
3. If `t` is a camera stamp: render, convert to mono8, project, publish `ImageFrame` then `Projections` (empty if nothing is visible), publish `TruthProjections`.
4. `viewer.sync()` if the viewer is open. Pace the publishes.
5. On follower done (path finished, then `tail_s` of stationary streaming): flush in order and perform the clean end.

`qvel` is written so the gyro cross-check and the rendered camera see one state. The published IMU still comes from section 2.2.

---

## 7. Phases

Each phase lists deliverables and acceptance checks. All checks must pass before moving on. Record measured numbers in section 10.

### Phase 0: skeleton and environment

Deliverables:

- Layout of section 4, empty modules, `INDEX.md` stubs, `.gitignore`, both `pyproject.toml` files, `calibration.yaml`, `broker.py`, `glsetup.py`.
- Venv and `pip install -e stream -e sim`.
- `scripts/gen_proto.py`.
- GL smoke, in a subprocess so a failed `MUJOCO_GL` can be retried. On Linux try `egl`, then `osmesa`. On macOS and Windows try the default.

```python
import mujoco, numpy as np
xml = """<mujoco><worldbody>
  <light pos="0 0 3"/><geom type="plane" size="1 1 0.1"/>
  <geom type="sphere" size="0.1" pos="0 0 0.1" rgba="1 1 1 1"/>
</worldbody></mujoco>"""
model = mujoco.MjModel.from_xml_string(xml)
data = mujoco.MjData(model)
mujoco.mj_forward(model, data)
r = mujoco.Renderer(model, height=48, width=64)
r.update_scene(data)
img = r.render()
assert img.shape == (48, 64, 3) and img.dtype == np.uint8 and img.mean() > 0
```

Acceptance: `python -m ee585_sim.broker` accepts a connection and is killed afterward; `pytest stream/tests` runs (zero tests is fine); the smoke test passes; section 10 records OS, `platform.machine()`, Python version, `mujoco.__version__`, and the `MUJOCO_GL` value that worked. Apple Silicon must report `arm64`, not a Rosetta `x86_64`, when the machine is arm64.

### Phase 1: contract and link library

Deliverables:

- `stream.proto` exactly as in section 5, generated `stream_pb2.py`.
- `link.py`:
  - topic name constants with prefix `ee585/v1/`.
  - `encode(msg) -> bytes`, `decode(topic, bytes) -> message`.
  - `class Link`: paho v2 client. `connect(client_id, will=True)` sets keepalive 2, the Last Will, publishes `ONLINE`. `publish(topic, payload, qos, retain)` with the 100-unacknowledged bound. `finish(last_seq, last_t_ns)` implements the clean end. `abort(detail)`.
  - `class Pacer(speed, t0_ns)` with `wait_until(t_ns)`.
- Unit tests: encode/decode round-trip for every message type with exact equality, including doubles and a 307200-byte image; `Pacer` timing at speed 10 (tolerance 50 ms on a 0.5 s wait); `Link` against the live aMQTT broker publishes retained calibration and a status.

Acceptance: tests pass; a retained `Status{ONLINE}` is visible to a second client that connects after the publish. `rg "mujoco|rclpy|gazebo" stream/` returns nothing.

### Phase 2: consumer, recorder, replay, and a fake provider

Deliverables:

- `consumer.py` per section 6.
- `recorder.py` CLI: `python -m ee585_stream.recorder --run runs/<name>` subscribes to `status`, `calibration`, `scene`, `stream`, `truth` and writes the log of section 3 plus `meta.json` (schema version, map name, counts, final terminal reason, wall-clock creation time). It uses the consumer's checks on `stream`.
- `replay.py` CLI: `python -m ee585_stream.replay --run runs/<name> --speed 1.0`. Publishes calibration and scene (retained), then merges `stream.bin` and `truth.bin` by `(t_ns, stream-before-truth)` while pacing, then the clean end. `seq` values are the bytes already in the log. Do not renumber.
- `tests/fake_provider.py`: a synthetic run with no MuJoCo. 20 s. IMU 200 Hz from an analytic circle (exact accel and gyro, this fake path does not use section 2.2's unicycle). Camera 30 Hz, 64×48 images. Projections of 5 fixed landmarks. Truth poses at 200 Hz. It writes a run log directly. Use it for consumer and replay tests. Stamps follow section 2.4 so the gap checks in later phases agree; the 64×48 size is only for this fake log.

Acceptance (pytest, real local broker):

1. Round trip: fake log → replay at `--speed 0` → recorder → new log. `stream.bin`, `truth.bin`, `calibration.bin`, `scene.bin` are byte-identical.
2. Speed independence: consumer output `(seq, t_ns, sha256(payload))` is identical at `--speed 1` on a 3 s excerpt, `--speed 10`, and `--speed 0`.
3. Duplicate injection: consumer output unchanged.
4. Gap injection (skip one `seq`): `Terminal(DATA_ERROR)`.
5. Disorder injection (smaller `t_ns`): `Terminal(DATA_ERROR)`.
6. Abrupt halt: kill the replay process mid-run (`kill -9` on POSIX; terminate the process so it cannot send `DISCONNECT` on Windows). The consumer reaches `Terminal(ABORTED)` within 4 s and reports its own `last_seq`.
7. Stall: provider stays connected and stops publishing. `Terminal(STALLED)` after `stall_timeout_s` (test with 1.0 s).
8. Clean end: `Terminal(COMPLETED)` and the last yielded record is `EndOfStream`.
9. Consumer started before the provider: waits, then receives the retained calibration and the full run.

If check 6 fails because aMQTT does not deliver the Last Will on a dropped TCP connection, fix the broker configuration or pin a newer `amqtt` that does. Do not switch transport, do not drop QoS, and do not weaken the 4 s bound. If no aMQTT version passes check 6, stop and report.

This phase is a complete headless stream. The algorithm side can be developed against `fake_provider` logs with no MuJoCo.

### Phase 3: maps, MJCF, kinematics, IMU

Deliverables:

- `kinematics.py` and `imu.py` implementing sections 2.2 and 2.3, with no `mujoco` import.
- `maps.py`:
  - `load_map(path)` validates the yaml below.
  - `generate_landmarks(map)`: `Generator(PCG64(seed))`. Vertical faces only (no tops, no outer wall faces). Area-weighted face pick, then a uniform point on that face. Count = `round(landmark_density_per_m2 * total_face_area)` with half away from zero (use `floor(x + 0.5)`). Height uniform in `[0.3, face_height - 0.2]`. Reject the map if any face has `face_height - 0.2 <= 0.3`. Offset the point `0.02` m along the normal that points into free space (into the arena for perimeter walls, away from the obstacle center for obstacles). Ids `0 .. N-1`.
  - `render_world(map, calibration) -> str` from `templates/world.xml.in`.
  - `render_textures(out_dir)`: `checker.png`, `noise.png`, `plain.png`, each 512×512 grayscale. Checker is 64 px squares of values 32 and 220. Plain is 180. Noise is `Generator(PCG64(0))` uniform uint8, independent of the map seed. Deterministic.
  - CLI: `python -m ee585_sim.maps build maps/<name>.yaml --out build/<name>/` writes `world.xml`, `landmarks.json`, textures.
- Map yaml:

```yaml
name: demo_room
seed: 1
arena: {size: [12.0, 12.0], wall_height: 2.5}   # inner floor, centered on the origin
obstacles:
  - {center: [2.0, 1.0], size: [1.0, 1.0, 1.5], yaw: 0.3}   # full sizes, not half-extents
texture: {name: noise, tile_m: 1.0}             # checker | noise | plain
landmark_density_per_m2: 1.0
lighting: {ambient: 0.4, sun_intensity: 0.8, sun_direction: [-0.5, 0.3, -1.0]}
path:
  waypoints: [[-4, -4], [4, -4], [4, 4], [-4, 4], [-4, -4]]
  speed_mps: 0.5
  accel_max_mps2: 0.5
  lookahead_m: 0.6
  tail_s: 1.0
```

- Arena `size` is the inner floor. Walls are boxes of thickness 0.1 m sitting outside that rectangle, outer corners covered, height `wall_height`. Obstacle `size` is the full box. MuJoCo `geom size` for a box is the half-extents; the generator divides by 2. Landmark spheres: radius 0.03, emissive white as in section 2.4, `contype="0" conaffinity="0"`.
- MJCF content:
  - `<option timestep="0.001" gravity="0 0 -9.81"/>`. No contact is required for motion. Robot geoms use `contype="0" conaffinity="0"`.
  - `<visual>`: headlight `active="0"` and ambient/diffuse/specular all 0. `offwidth` and `offheight` at least the calibration size. Shadows off.
  - One directional light. `dir` is `sun_direction` normalized (rays travel that way). `ambient` and `diffuse` are the yaml scalars replicated on RGB. `specular="0 0 0"`. `castshadow="false"`.
  - Ground plane on `z = 0`, textured. `texuniform="true"` and `texrepeat="{1/tile_m} {1/tile_m}"` on floor, walls, and obstacles.
  - Robot, nested so slide axes stay in the world:

```xml
<body name="tx" pos="0 0 0.05">
  <joint name="jx" type="slide" axis="1 0 0" damping="0"/>
  <body name="ty">
    <joint name="jy" type="slide" axis="0 1 0" damping="0"/>
    <body name="base">
      <joint name="jyaw" type="hinge" axis="0 0 1" damping="0"/>
      <site name="imu" pos="0 0 0"/>
      <camera name="cam" pos="0.15 0 0.10" xyaxes="0 -1 0 0 0 1" fovy="FOVY_DEG" mode="fixed"/>
      <geom name="chassis" type="box" size="0.20 0.15 0.075" pos="0 0 0.05" mass="5" contype="0" conaffinity="0"/>
      <!-- visual wheels: radius 0.05, half-length 0.01, centers (0, ±0.15, 0), axis along body Y -->
      <!-- visual caster sphere radius 0.02 at (-0.15, 0, -0.03) -->
    </body>
  </body>
</body>
```

  `xyaxes="0 -1 0 0 0 1"` is camera +X (right) = body −Y and camera +Y (up) = body +Z, so the camera looks along body +X. Do not author the camera with `q_BC`; that quaternion is optical, and the MuJoCo camera frame is not C.
  - Sensors as in section 2.2.
- `model.py`: load the XML, set `qpos` from `(x, y, yaw)`, `mj_forward`, render through the named camera, and run the engine cross-check.

Acceptance, no follower yet:

1. `demo_room` XML compiles. `mj_forward` at `qpos = (1, 2, π/2)` gives base `xpos` within `1e-8` of `(1, 2, 0.05)`, and `R @ (1, 0, 0)` within `1e-8` of `(0, 1, 0)`. `xquat` matches `(cos(π/4), 0, 0, sin(π/4))` within `1e-8` (`w >= 0`).
2. Engine cross-check of section 2.2 (static accelerometer and gyro, and gyro at 0.3 rad/s).
3. Zero-noise IMU: at rest, every sample equals `(gyro 0, accel (0, 0, 9.81))` within `1e-12`. With `v` constant, `ω = 0.2`, `f_B = (0, 0.1, 9.81)` and `gyro = (0, 0, 0.2)`.
4. Straight noiseless profile: ω = 0, yaw = 0, command `v = 0.5 * t_end` on intervals ending at 0.005 … 1.000 s, then `v = 0.5` for 2 s. Dead-reckon as in phase 6 against the kinematic position at 3 s. Error below 0.01 m.
5. Noise statistics, rest, 10 s, 2000 samples, default densities, a fresh generator: mean accel within 0.05 of `(0, 0, 9.81)`; per-axis accel std within 10% of `2.828e-2`; per-axis gyro std within 10% of `2.400e-3`; gyro mean magnitude below `5e-4`.
6. Open-loop `v = 0.5`, `ω = 0` for 4 s from the origin: displacement `(2, 0)` within `1e-6` m. `z`, roll, and pitch unchanged.
7. IMU stamps over 10 s are exactly every 5_000_000 ns. Camera stamps over 1 s match `t_cam(k)` including `t_cam(30) = 1_000_000_000`.
8. One offscreen frame of `demo_room` at the start pose has shape `(480, 640, 3)` and mean greater than 0.

### Phase 4: projection library

`projection.py` is Python and numpy. It does not import `mujoco`.

```python
class Projector:
    def __init__(self, calib: dict, landmarks: np.ndarray, occluders: list, seed: int):
        ...
    def project(self, t_ns: int, p_WB: np.ndarray, q_WB: np.ndarray) -> tuple[list, list]:
        """Return (noisy, truth) observations, same ids and order, ids ascending."""
```

`seed` is `map_seed + 1`. One `Generator(PCG64(seed))` for the whole run.

For each landmark, double precision:

1. `T_WC = T_WB * T_BC`. `p_C = R_WC^T (p_W - p_WC)`.
2. Reject if `Z <= 0`, or range `|p_C|` is outside `[min_range_m, max_range_m]`.
3. `u = fx X/Z + cx`, `v = fy Y/Z + cy`. Reject if noiseless `u` is outside `[0, width)` or noiseless `v` is outside `[0, height)`.
4. Occlusion: each occluder is an oriented box (center, full size, yaw), including the four perimeter walls. Slab-test the segment from the camera origin to the landmark. Occluded if the segment hits a box at parameter `s` in `(0, 1 - 1e-6)`. The 0.02 m free-space offset keeps a landmark off its own face.
5. Noise per section 2.4.

Tests, no viewer:

1. A landmark on the optical axis projects to `(cx, cy)` exactly.
2. A landmark at `(1, 0, 4)` in C with the default intrinsics projects to `(fx * 0.25 + cx, cy)`.
3. Behind the camera, out of range, and out of the image are rejected.
4. A box between camera and landmark occludes; moving the box aside makes it visible.
5. Over 10,000 draws the noise std is within 3% of sigma and the mean error is below `0.05 * sigma`.
6. The same seed gives identical output. Truth observations have no noise.
7. Yawing the body by +90° (left turn) moves a landmark that was on the left to the image center.
8. Render orientation, three emissive white spheres of radius 0.05 m, no walls, sun off, ambient 1, robot at the origin with yaw 0. Place each sphere at optical coordinates `(0, 0, 4)`, `(1, 0, 4)`, and `(0, 1, 4)` via `T_WC`. Threshold mono `>= 200`. Centroid uses pixel centers `(col + 0.5, row + 0.5)`. The axial sphere's centroid is within 1.0 px of `(cx, cy)`. The right sphere has centroid `u > cx + 0.2 * fx`. The down sphere has centroid `v > cy + 0.2 * fy`.
9. Integration with the phase 3 world: robot parked, one frame, five visible landmarks. Centroid of pixels with mono `>= 200` versus the noiseless projection is below 1.0 px for each. Log the five residuals in section 10.

### Phase 5: path follower

`follower.py`, no ROS. Called at IMU stamps with the pose at that stamp. Returns `(v, ω)` for the next interval.

- For stamps with `t_ns < initial_rest_s * 1e9`, return `(0, 0)` and do not update the ramp.
- Pure pursuit on the polyline. Closest point: the minimum-distance projection onto any segment. Target: the point `lookahead_m` further along the polyline; if less than that remains, the target is the final waypoint.
- Heading error `α = wrap(atan2(ty - y, tx - x) - yaw)` into `(-π, π]`. `ω = clamp(1.5 * α, -1.0, 1.0)`.
- `d_stop = speed_mps² / (2 * accel_max_mps2)`. Remaining length is the polyline length from the closest point to the end. `v_target = speed_mps * min(1, remaining / d_stop)`, or `0` when remaining `< 1e-3`. Rate-limit: `|v - v_prev_command| <= accel_max_mps2 * 0.005`.
- After the path is complete and `v` has returned to 0, keep returning `(0, 0)` for `tail_s` seconds of simulation time, then set `done` exactly once.

Initial pose: `(x, y)` is the first waypoint, `yaw = atan2` toward the second waypoint (0 if the path has one point), `z = 0.05`.

Acceptance on `demo_room`, zero sensor noise, truth poses:

1. The robot finishes the path. Horizontal distance to the polyline stays below 0.3 m.
2. The chassis footprint (0.40 m × 0.30 m centered on the body origin, rotated by yaw) never intersects an obstacle rectangle. Check this in 2D on every IMU stamp.
3. `done` becomes true once.
4. Roll and pitch are 0 and `z` is 0.05 on every truth pose (tolerance `1e-9`).
5. The first 1.0 s is stationary: every truth pose in that window matches the start pose within `1e-9`.

### Phase 6: MuJoCo live provider

`provider_mj.py` CLI:

```text
python -m ee585_sim.provider_mj --map maps/demo_room.yaml --speed 1.0 [--viewer]
```

Implements section 6. `--viewer` calls `launch_passive` after the `Renderer` is created, syncs after `mj_forward`, and ignores window close. If `--viewer` was passed and the window cannot open, abort with `detail: "viewer failed"`. Headless is the default and must not try to open a window.

Acceptance, live provider, `demo_room`, recorder running, `--speed 0` allowed for the checks that are not about wall-clock pacing:

1. `meta.json` terminal reason is `COMPLETED`.
2. `stream.bin`: `seq` gapless, `t_ns` non-decreasing, IMU gaps exactly 5_000_000 ns, camera stamps equal `t_cam(k)`, every image immediately followed by one projections record with the same `t_ns`.
3. For 20 random frames, reproject the scene with the logged truth pose and `Projector` at zero noise. Match `TruthProjections` within `1e-6` px.
4. Noisy minus noiseless projection errors over the run have std within 5% of `pixel_noise_sigma` on each axis.
5. Rendered images: shape and length match the calibration; row 0 is the top (phase 4 test 8 already locks this). mono8 has no extra noise term (a repeated render of the same parked pose is byte-identical in one process).
6. Dead-reckon a 3 s excerpt that starts at rest. Use logged truth orientation, not an estimated one:

```text
a_W = R(q_truth) @ accel_B + g_W
v_W = v_W + a_W * 0.005
p_W = p_W + v_W * 0.005
```

   Position error versus truth at the end of the window is below 0.15 m. This checks the specific-force sign, the yaw, and the noise level together.
7. Killing the provider so it cannot `disconnect()` yields `ABORTED` at the consumer within 4 s.
8. Replay of the recorded log reproduces byte-identical `stream.bin` and `truth.bin`.
9. With `--viewer`, if the machine has a display (`DISPLAY` set, or a Windows desktop session): the window opens, the robot is visible, and images on the stream are not all zeros. If there is no display, log "viewer not run" and do not fail the phase.

### Phase 7: map set and reference logs

At least these maps:

- `demo_room`: phase 3 baseline.
- `sparse_texture`: same geometry, `plain` texture, `landmark_density_per_m2: 0.2`.
- `dense_features`: `noise` texture, `landmark_density_per_m2: 3.0`.
- `dim_lighting`: baseline geometry, ambient 0.1, sun intensity 0.2. Landmarks stay emissive, so the centroid check still applies. Walls and floor follow the dim sun.
- Optionally one map with a larger arena and a path of at least 60 s of motion.

For each map: build, run live, record `runs/<map>/`. Write `docs/usage.md`: start the broker, run headless, run with `--viewer`, record, replay, and a 15-line consumer example (section 8). State the pip install and that Ubuntu, macOS including Apple Silicon, and Windows are the target platforms. Keep `runs/` out of git. `docs/usage.md` stores the commands that regenerate the logs.

Acceptance: every map's run is `COMPLETED` and satisfies phase 6 checks 2–6. Mean visible landmarks per frame are ordered `dense_features` > `demo_room` > `sparse_texture`.

### Phase 8 (optional, only if the owner asks): moving occluders

A map may list boxes with a pose that is a function of `t_ns`. The projector and the renderer both use that pose at the camera stamp. The occluder list passed to `project` becomes time-dependent. Do not start this without an explicit request. Do not add a second process or a ROS node.

---

## 8. Interface summary for the algorithm side

```python
from ee585_stream.consumer import Consumer, Calibration, Record, Terminal

c = Consumer("localhost", 1883, stall_timeout_s=5.0)
for ev in c.events():
    if isinstance(ev, Calibration):
        init_filter(ev)
    elif isinstance(ev, Record):
        kind = ev.WhichOneof("payload")      # imu | image | projections | end
        filter.process(ev)                   # uses ev.t_ns differences only
    elif isinstance(ev, Terminal):
        write_run_status("out/status.json", ev)
        break
```

- The consumer is the only way the algorithm receives data.
- The evaluator variant additionally subscribes to `scene` and `truth`.
- Dropout is applied by the algorithm-side harness to records after they leave the consumer. The provider publishes every visible landmark.

---

## 9. Out of scope (do not implement)

The MSCKF, baselines, metrics, and the dropout harness. ROS 2, Gazebo, `ros_gz`, a required Mosquitto install, and any Ubuntu-only system package. IMU bias, bias random walk, scale factor, and misalignment. Lens distortion, rolling shutter, and a camera–IMU time offset (it is zero: stamps are the provider clock). Stereo and depth. Noise, blur, or exposure on the rendered image. Contact-driven driving and wheel-odometry output. Filament. Domain randomization. Real-dataset adapters (a future provider may publish a public dataset on this same contract). Loop closure. Moving occluders, unless phase 8 was explicitly requested.

---

## 10. Execution log (the executing agent appends here)

Format: `YYYY-MM-DD | phase | note`. Record OS, machine architecture, Python and MuJoCo and aMQTT versions, the `MUJOCO_GL` value that worked, the sensor-evaluation calls used for the cross-check, centroid residuals from phase 4, whether `--viewer` was run, measured acceptance numbers, and every deviation with its reason.

2026-11-22 | 0 | macOS 26.6.2, `platform.machine()` = `arm64`, `sysctl.proc_translated` = 0, CPython 3.12.4. mujoco 4.0.1, amqtt 0.12.2, paho-mqtt 2.2.0, numpy 2.5.5, pytest 9.2.0, protobuf 7.36.2, grpcio-tools 1.85.3. `MUJOCO_GL` unset (macOS default; Linux egl/osmesa was not tried). Smoke subprocess: shape `(48, 64, 3)`, dtype `uint8`, mean `135.88248697916666`. `python -m ee585_sim.broker` accepted a paho v2 connection (`CONNACK Success`) and was then terminated (exit `-15`). `pytest stream/tests` collected 0 tests and exited 5. `--viewer` was not run. No sensor cross-check and no centroid residuals yet.

Deviation: install command was `pip install -e "stream[dev]" -e sim`. The dev extra supplies pytest and grpcio-tools, which the phase 0 checks and `scripts/gen_proto.py` need. Runtime dependencies are unchanged.

Deviation: aMQTT 0.12.2 accepts the roadmap `auth` dict only by loading every entry-point plugin, and that loader warns it will be removed. It pulled in file auth (warning: no password file) and the `$SYS` plugin. `CONFIG` instead lists `plugins` → `amqtt.plugins.authentication.AnonymousAuthPlugin` with `allow_anonymous: True`. Bind remains `127.0.0.1:1883`. The broker sets no QoS and no packet-size cap. Retained state is memory only. No persistence. The MQTT remaining-length maximum is 268435455 bytes, above 2000000.

Note: Homebrew Mosquitto (`/opt/homebrew/opt/mosquitto/sbin/mosquitto`, config `/opt/homebrew/etc/mosquitto/mosquitto.conf`) was listening on `*:1883`. It was stopped for the connection check and started again afterward (launchd label `sh.brew.mosquitto`). The next broker run needs that port free.

2026-11-22 | 1 | `pytest stream/tests`: 3 passed in 1.17 s. Encode/decode round-trip matched every protobuf message, including the calibration doubles and a 307200-byte image. `Pacer(10, 0).wait_until(5_000_000_000)` finished within 50 ms of 0.5 s. A second client that connected after `Link.publish` received retained `Status{ONLINE, last_seq: -1, last_t_ns: -1}` and the retained `Calibration`. `rg "mujoco|rclpy|gazebo" stream/` returned nothing. `stream.proto` matches section 5. `stream_pb2.py` was generated with grpcio-tools 1.85.3. paho 2.2.0 used `CallbackAPIVersion.VERSION2`. Homebrew Mosquitto was stopped for the broker test and started again afterward.

2026-11-22 | 2 | Design choice: from this phase on, this machine uses the existing Homebrew Mosquitto at `127.0.0.1:1883` and paho-mqtt 2.2.0. Tests do not start `python -m ee585_sim.broker` and do not stop Mosquitto. Topics, QoS, payloads, keepalive, and the paho client are unchanged. Reason: aMQTT 0.12.2 did not deliver a QoS 2 burst reliably. Against that broker, `pytest stream/tests` was 5 failed, 7 passed in 37.63 s: the 20 s `--speed 0` replay exited 1, the speed-10 consumer stopped at `seq` 195, a 1 s speed-0 run did not yield `EndOfStream`, and the early consumer stalled. Checks 4, 5, 6, and 7 had already passed on aMQTT. Mosquitto binary `/opt/homebrew/opt/mosquitto/sbin/mosquitto`, config `/opt/homebrew/etc/mosquitto/mosquitto.conf` (`allow_anonymous true`, `listener 1883`, `persistence true`). `ee585_sim.broker` stays in the tree and is not the broker under test.

2026-11-22 | 2 | `pytest stream/tests`: 12 passed in 7.96 s against Mosquitto 2.1.2. Round trip at `--speed 0` made `stream.bin`, `truth.bin`, `calibration.bin`, and `scene.bin` byte-identical. Consumer `(seq, t_ns, sha256)` matched at speeds 1, 10, and 0 on the 3 s excerpt. Duplicate seq was dropped. A skipped seq and a decreased `t_ns` each ended in `DATA_ERROR`. `kill -9` on replay produced `Terminal(ABORTED)` within 4 s with the consumer's own `last_seq`. A connected silent provider produced `STALLED` at `stall_timeout_s=1.0`. Clean end yielded `EndOfStream` then `COMPLETED`. A consumer that subscribed first received retained calibration `fake_circle` and every stream frame. `rg "mujoco|rclpy|gazebo" stream/` returned nothing.

Deviation: `Link` keeps at most 20 unacknowledged QoS 1/2 publishes, and paho `max_inflight_messages` is 20. Mosquitto 2.1.2's default receive maximum is 20 (`max_inflight_messages`). Above that it discards the client's publishes and logs `exceeded its receive-maximum quota`. The abort ceiling is still 100 unacknowledged messages or a 2 s wait, whichever the publisher hits first. With the window at 20, a 20 s speed-0 log (5204 stream records, 4602 truth records) was delivered with no gaps.

2026-11-22 | 3 | `pytest sim/tests/test_phase3.py`: 9 passed in 0.43 s (with phase 4). `demo_room` compiles. At `qpos = (1, 2, π/2)`, base `xpos` is `(1, 2, 0.05)` and `xquat` is `(cos(π/4), 0, 0, sin(π/4))` within `1e-8`. Engine sensors use `mj_forward` only. On MuJoCo 4.0.1 that call already fills `sensordata`; `mj_fwdAcceleration` and `mj_sensorAcc` were tried and leave the same values. Static accelerometer is `(0, 0, 9.81)` and gyro is `(0, 0, 0)`. With joint yaw rate `0.3`, gyro is `(0, 0, 0.3)` and the accelerometer x component is `-2.68e-5`, inside `1e-4`. No sign flip. Offscreen frame at the path start is `(480, 640, 3)` uint8, mean `64.36492513020833`. `MUJOCO_GL` still unset. `--viewer` was not run.

Deviation: the literal `hfov_rad: 1.5707963` gives `fx = fy = 320.00000857436703`, which is `8.57e-6` from 320, outside the stated `1e-6`. `fovy_deg` is `73.73979381786295`. The yaml value was not changed. `q_BC` matches `(0.5, -0.5, 0.5, -0.5)`.

Deviation: MuJoCo 4.0.1 rejects the nested slide bodies unless `tx` and `ty` have positive inertia. Each has `<inertial mass="0.001" diaginertia="1e-6 1e-6 1e-6"/>`. Wheels and the caster use `mass="0.01"` so the default density does not add mass. Material `emission` is a scalar here; landmarks use `emission="1"` rather than `emission="1 1 1"`.

Deviation: landmark faces are the four inner arena faces of the inner rectangle, not the corner extensions of the wall boxes, plus the four vertical faces of each obstacle. `demo_room` has 126 landmarks.

2026-11-22 | 4 | `pytest sim/tests/test_phase4.py`: 9 passed. Optical-axis and `(1, 0, 4)` projections match the intrinsics. Behind, out-of-range, and out-of-image landmarks are dropped. A box on the ray occludes; the same box moved aside does not, and a face 0.02 m past the landmark does not. Over 10,000 frames the pixel-noise std is within 3% of 0.5 and the mean error is below `0.05 * 0.5`. The same seed repeats, and an invisible landmark consumes no draws. A +90° yaw puts the chosen left landmark on `(cx, cy)`. Three emissive spheres satisfy the right/down centroid directions, and the axial centroid is within 1 px of `(cx, cy)`. Five `demo_room` landmarks, robot parked, centroid residuals in pixels: `0.1317`, `0.0485`, `0.1468`, `0.0450`, `0.0150`.

2026-11-22 | 5 | `pytest sim/tests/test_phase5.py`: 1 passed in 0.19 s. On `demo_room` the pure-pursuit run reaches `done` once, stays within 0.3 m of the polyline, and the 0.40 m × 0.30 m chassis does not meet the obstacle rectangle. The first 1.0 s stays on the first waypoint. Published height stays 0.05 m and the quaternion stays a pure yaw. On a closed polyline, equal projection distances keep the along-track value nearer the previous progress, so the start is not confused with the final waypoint.
