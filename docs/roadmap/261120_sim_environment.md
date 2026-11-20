# Roadmap: Simulation Environment and Streaming Contract

Scope: everything on the data-provider side of the boundary, plus the Python library that the algorithm side uses to receive data. The MSCKF algorithm, its baselines, the dropout harness, and the evaluation metrics are NOT in this roadmap.

Audience: an executing agent with no prior context. Everything it needs is in this file. Do not re-open decisions in section 2.

---

## 0. Rules for the executing agent

- Follow `AGENTS.md` and the `folder-index` skill (`.cursor/skills/folder-index/SKILL.md`). Every folder that directly contains code gets an `INDEX.md` in the same change as the code. `tests/` folders and generated files are exempt.
- Simple and direct. Do not add abstractions, plugin systems, or configuration beyond what is written here.
- Do NOT commit or push. The owner decides when. When asked, use the `commit-helper` skill.
- Work phase by phase in the order of section 7. Do not start a phase before the previous phase's acceptance checks pass.
- Phases 1 and 2 need no ROS or Gazebo. Phases 3 onward need ROS 2 Jazzy and Gazebo Harmonic on Ubuntu 24.04. If those are not installed and cannot be installed, finish phases 1 and 2, then stop and report.
- If a Gazebo or ROS detail in this file does not match what you observe (topic name, plugin name, frame), trust the observation, fix the config, and record the deviation in section 10. Do not change a decision in section 2.
- Append to section 10 as you go. Do not edit other sections except to fix an error you can prove.

---

## 1. Goal

Provide an environment in which a visual-inertial algorithm can be run against:

1. a live Gazebo simulation, or
2. a replay of a recorded run (headless, no Gazebo),

through one identical interface: a stream of timestamped records on MQTT. The algorithm never imports Gazebo, ROS, or reads files. When the provider stops, the algorithm stops producing new estimates.

---

## 2. Locked decisions

| Topic | Decision |
|---|---|
| ROS / Gazebo | ROS 2 Jazzy, Gazebo Harmonic (`gz sim`), Ubuntu 24.04, `ros_gz` bridge |
| Role of Gazebo | World, rigid-body physics, IMU sensor, camera sensor, visualization. GUI is optional; the server alone must be able to generate data. |
| Robot | Wheeled differential-drive robot on flat ground. Motion is planar (x, y, yaw) by construction; ground truth is still the full 6-DoF pose Gazebo reports. |
| Scene | 3D (walls, obstacles, textured surfaces, landmarks) |
| Camera | ONE undistorted pinhole camera. No depth, no stereo. |
| Camera channels | Both, from the same camera pose and intrinsics: (a) the rendered mono8 image, (b) synthetic projections of known 3D landmarks with pixel noise |
| IMU | Additive white Gaussian noise only. Zero bias, zero bias random walk. Body frame FLU (x forward, y left, z up). World frame ENU, gravity_W = (0, 0, -9.81). |
| Maps | Variable between maps: geometry, path length, texture and feature density, lighting. Fixed: robot, intrinsics, IMU noise model. (Moving occluders: optional phase 8.) |
| Ground truth / dropout | Ground truth is logged on a separate topic. Feature dropout is NOT done in the simulator or provider; it is applied later by the estimator test harness. |
| Boundary | Provider publishes a stream; algorithm consumes it. Providers: Gazebo live provider, replay provider. |
| Transport | MQTT (Mosquitto broker, paho-mqtt client) |
| Stream layout | ONE ordered `stream` topic. Calibration, scene, status, truth on separate topics. |
| Payload | Protobuf. All numeric fields `double`. Timestamps `int64` simulation nanoseconds. |
| Image | mono8 raw bytes in the record, row-major, tightly packed |
| Delivery | QoS 2 on `stream` and `truth`. Gapless `seq` number in every record. Provider is paced. |
| Run ending | `end_of_stream` is metadata only; the algorithm never waits for it. Status in {completed, aborted, stalled}. Abort detected by MQTT Last Will. Stall timeout 5 s (wall clock, configurable). |
| Language | Python 3.12 for everything in this roadmap. The contract is language-neutral, so the algorithm side may differ. |

Frame and unit conventions (also written into `stream.proto` comments):

- World W: ENU, z up. Body B: FLU, origin at the IMU. Camera C: OPTICAL frame (x right, y down, z forward).
- Rotations are Hamilton unit quaternions (w, x, y, z). `q_WB` rotates body vectors into world: `v_W = R(q_WB) v_B`.
- `p_BC, q_BC` is the pose of C in B: `x_B = R(q_BC) x_C + p_BC`.
- IMU gyro: rad/s, body frame. IMU accel: m/s^2, body frame, SPECIFIC FORCE: `f_B = R_WB^T (a_W - g_W)`. At rest and level it reads (0, 0, +9.81).
- Pinhole projection of `p_C = (X, Y, Z)`, Z > 0: `u = fx X / Z + cx`, `v = fy Y / Z + cy`. No distortion.
- Time: `t_ns` = Gazebo simulation time in integer nanoseconds, 0 at world start. Wall-clock time is never used for data.
- Camera frame C relative to body B for a forward-facing camera (default): `R_BC = [[0,0,1],[-1,0,0],[0,-1,0]]` (columns are the optical x, y, z axes expressed in B). `p_BC = (0.15, 0, 0.10)`. Compute `q_BC` from this matrix.

---

## 3. Architecture

```
 Gazebo server (world, physics, imu, camera, diff-drive, pose publisher)
        |  gz-transport
   ros_gz_bridge  --ROS 2-->  provider_gz (ROS node)  --MQTT-->  broker  --MQTT-->  algorithm (consumer lib)
                               |  merge + project + seq                       |
                               +--follower node (cmd_vel)                     +--> recorder  (writes run log)
                                                                              +--> evaluator (truth topic)

 replay provider (reads run log) --MQTT--> broker     (same topics, same bytes, no Gazebo)
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

Run log (written by the recorder, read by the replay provider): directory `runs/<run_name>/` containing `meta.json`, `calibration.bin`, `scene.bin`, `stream.bin`, `truth.bin`. Each `.bin` is a sequence of `[uint32 little-endian length][payload bytes]` where payload is the exact MQTT payload. Replay republishes the same bytes. `runs/` is git-ignored.

---

## 4. Repository layout to create

```
stream/                          pure Python package, NO ROS/Gazebo imports
  pyproject.toml
  INDEX.md
  ee585_stream/
    INDEX.md
    stream.proto
    stream_pb2.py                generated (commit it; regenerate with scripts/gen_proto.sh)
    link.py                      topics, encode/decode, MQTT connect with will, bounded publisher
    consumer.py                  receive, order/seq checks, terminal status
    recorder.py                  subscribe and write run log (CLI)
    replay.py                    read run log and publish (CLI)
  scripts/gen_proto.sh
  tests/
sim/
  ee585_sim/                     ROS 2 package (ament_python)
    INDEX.md
    package.xml, setup.py, setup.cfg, resource/ee585_sim
    ee585_sim/
      INDEX.md
      maps.py                    load map yaml, generate landmarks, render world/robot SDF, textures
      projection.py              pure pinhole projection with occlusion and noise
      follower.py                ROS node: waypoint follower -> cmd_vel, /run/done
      provider_gz.py             ROS node: merge, project, publish to MQTT
    launch/
      INDEX.md
      sim.launch.py
    config/
      calibration.yaml           single source of truth for sensor parameters
      bridge.yaml                ros_gz_bridge config
    maps/                        map yaml files
    templates/                   robot.sdf.in, world.sdf.in
    tests/
broker/mosquitto.conf
docs/roadmap/                    this file
runs/                            git-ignored
.gitignore
```

Python dependencies (in a venv created with `--system-site-packages`, so `rclpy` stays visible): `paho-mqtt>=2.0`, `protobuf>=5`, `grpcio-tools` (dev, for `protoc`), `pyyaml`, `numpy`, `pillow`, `pytest`.

---

## 5. Contract: `stream.proto`

Create exactly this schema (comments are part of the contract; keep them).

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
  bytes  mono8  = 3;  // width*height bytes, row-major, no padding
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
  int64  t_ns = 2;        // simulation time, non-decreasing over the stream
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

Numeric defaults for `config/calibration.yaml` (the executing agent writes the YAML; values below are the defaults):

```yaml
schema_version: 1
imu:
  rate_hz: 200
  gyro_noise_density: 1.6968e-4     # rad/s/sqrt(Hz)  -> per-sample sigma 2.400e-3 rad/s at 200 Hz
  accel_noise_density: 2.0e-3       # m/s^2/sqrt(Hz)  -> per-sample sigma 2.828e-2 m/s^2 at 200 Hz
  gravity_W: [0.0, 0.0, -9.81]
camera:
  width: 640
  height: 480
  rate_hz: 30
  hfov_rad: 1.5707963               # fx = fy = (width/2)/tan(hfov/2) = 320.0, cx = width/2, cy = height/2
  pixel_noise_sigma: 0.5
  p_BC: [0.15, 0.0, 0.10]
  R_BC: [[0,0,1],[-1,0,0],[0,-1,0]]
  min_range_m: 0.2
  max_range_m: 20.0
initial_rest_s: 1.0
```

Gazebo applies `stddev` to every sample as given, so the SDF IMU noise `stddev` must equal `density * sqrt(rate_hz)`. `maps.py` computes it from the YAML; never hand-edit it in the SDF.

---

## 6. Operating rules for providers and consumers

Provider (both providers):

- MQTT client keepalive = 2 s. Network loop in a background thread (`loop_start`).
- On connect: register the Last Will on `status`: `Status{state: ABORTED, last_seq: -1, last_t_ns: -1, detail: "last will"}`, QoS 1, retained. Then publish retained `Status{ONLINE}`, then retained `calibration` and `scene`.
- `seq` starts at 0 and increases by 1 per `Record` on `stream`. `TruthRecord.seq` is a separate gapless counter.
- Publishing is bounded: count publishes not yet acknowledged and block while the count exceeds 100.
- Pacing: `--speed X` (default 1.0) publishes record `r` no earlier than `wall_start + (t_ns(r) - t0_ns)/X` seconds. `--speed 0` means unpaced, limited only by the unacknowledged bound.
- Clean end: publish `Record{end: EndOfStream}` (it takes the next `seq`), wait until all publishes are acknowledged, publish retained `Status{COMPLETED, last_seq, last_t_ns}`, then `disconnect()` normally (so the Last Will is not sent).
- If the provider detects its own failure (see section 7, phase 6), it publishes retained `Status{ABORTED, detail: <reason>}` and exits non-zero.

Broker (`broker/mosquitto.conf`):

```
listener 1883 127.0.0.1
allow_anonymous true
persistence false
max_queued_messages 0
max_inflight_messages 20
message_size_limit 0
```

Consumer (`ee585_stream.consumer`):

- Subscribes to `status`, `calibration`, `stream`. Never to `scene` or `truth` unless constructed as the evaluator variant.
- Yields events in order: `Calibration`, then `Record`s, then exactly one `Terminal`.
- Checks on every record: `seq == last_seq + 1`; a smaller `seq` is a duplicate and is dropped silently; a larger `seq` is a gap. `t_ns` non-decreasing, else out-of-order. A gap or out-of-order ends the run with `Terminal(reason=DATA_ERROR)`. Fail loudly; never repair.
- Terminal reasons:
  - `COMPLETED`: status `COMPLETED` received AND the record with `seq == status.last_seq` has been yielded.
  - `ABORTED`: status `ABORTED` received. Drain already-queued records first, then terminate.
  - `STALLED`: status is `ONLINE`, at least one record was received, and no record arrived for `stall_timeout_s` (default 5.0 wall-clock seconds). Before the first record, the wait limit is `startup_timeout_s` (default 60.0).
  - `DATA_ERROR`: as above, also an undecodable payload.
  - The `EndOfStream` record is yielded like any other record. It is not required for termination.
- `Terminal` carries `reason`, `last_seq`, `last_t_ns`. `write_run_status(path, terminal)` writes them to a small JSON file. Estimates already produced remain valid after any terminal reason.

Estimator-time semantics: the consumer hands records to the algorithm one at a time in `seq` order. The algorithm uses only `t_ns` differences, never wall-clock, so output must be identical at any `--speed`.

---

## 7. Phases

Each phase lists deliverables and acceptance checks. All checks must pass before moving on.

### Phase 0: skeleton and environment

Deliverables:
- Repository layout of section 4 (empty modules and `INDEX.md` stubs for folders that will hold code), `.gitignore` (`venv`, `__pycache__`, `runs/`, `*.egg-info`, `build`, `install`).
- Venv and dependencies. `broker/mosquitto.conf`. `stream/scripts/gen_proto.sh` (runs `python -m grpc_tools.protoc` into `ee585_stream/`).
- On Ubuntu 24.04 with ROS 2 Jazzy: `sudo apt install mosquitto ros-jazzy-ros-gz` (this pulls Gazebo Harmonic). Note the exact installed versions in section 10.

Acceptance: `mosquitto -c broker/mosquitto.conf` starts; `pytest stream/tests` runs (zero tests is fine); `gz sim --version` and `ros2 pkg list | grep ros_gz` work (only if installing ROS is possible in this environment).

### Phase 1: contract and link library

Deliverables:
- `stream.proto` exactly as in section 5, generated `stream_pb2.py`.
- `link.py`:
  - topic name constants.
  - `encode(msg) -> bytes`, `decode(topic, bytes) -> message`.
  - `class Link`: wraps a paho v2 client. `connect(client_id, will=True)` sets keepalive 2, the Last Will, publishes `ONLINE`. `publish(topic, payload, qos, retain)` with the 100-unacknowledged bound. `finish(last_seq, last_t_ns)` implements the clean end of section 6. `abort(detail)`.
  - `class Pacer(speed, t0_ns)` with `wait_until(t_ns)`.
- Unit tests: encode/decode round-trip for every message type with exact equality (including doubles and a 307200-byte image); `Pacer` timing at speed 10; `Link` against a live local broker publishes retained calibration and a status.

Acceptance: tests pass; a retained `Status{ONLINE}` is visible to a late-joining subscriber (`mosquitto_sub -t 'ee585/v1/#' -v -C 1`).

### Phase 2: consumer, recorder, replay, and a fake provider

Deliverables:
- `consumer.py` per section 6.
- `recorder.py` CLI: `python -m ee585_stream.recorder --run runs/<name>` subscribes to `status`, `calibration`, `scene`, `stream`, `truth` and writes the log of section 3 plus `meta.json` (schema version, map name from calibration, counts, final terminal reason, wall-clock creation time). It uses the consumer's checks on `stream`.
- `replay.py` CLI: `python -m ee585_stream.replay --run runs/<name> --speed 1.0`. Publishes calibration and scene (retained), then merges `stream.bin` and `truth.bin` by `t_ns` (stream first at ties) while pacing, then the clean end. `seq` values are taken from the bytes already in the log (do not renumber).
- `tests/fake_provider.py`: generates a synthetic run without Gazebo: 20 s, IMU 200 Hz computed analytically from a circular path (so accel and gyro are exact), camera 30 Hz with 64x48 test images, projections of 5 fixed landmarks, truth poses at 200 Hz. It writes a run log directly. Use it for all consumer and replay tests.

Acceptance (pytest, using a real local broker):
1. Round trip: fake run log -> replay at `--speed 0` -> recorder -> new log. The new `stream.bin`, `truth.bin`, `calibration.bin`, `scene.bin` are byte-identical to the originals.
2. Speed independence: consumer output (list of `(seq, t_ns, sha256(payload))`) is identical at `--speed 1` (use a 3 s excerpt), `--speed 10`, and `--speed 0`.
3. Duplicate injection (publish one record twice): consumer output unchanged.
4. Gap injection (skip one `seq`): `Terminal(DATA_ERROR)`.
5. Disorder injection (a record with smaller `t_ns`): `Terminal(DATA_ERROR)`.
6. Abrupt halt: `kill -9` the replay process mid-run. The consumer reaches `Terminal(ABORTED)` within 4 s of the kill and reports its own `last_seq`.
7. Stall: provider stays connected and stops publishing. `Terminal(STALLED)` after `stall_timeout_s` (test with 1.0 s, not 5.0).
8. Clean end: `Terminal(COMPLETED)` and the last yielded record is the `EndOfStream`.
9. Consumer started before the provider: waits (up to `startup_timeout_s`), then receives the retained calibration and the full run.

This phase delivers a complete headless pipeline. The algorithm side can already be developed against `fake_provider` logs.

### Phase 3: maps, generated SDF, and the Gazebo world

Deliverables:
- `maps.py`:
  - `load_map(path)`: validates the map yaml (schema below).
  - `generate_landmarks(map)`: deterministic from `seed`: sample points on the vertical faces of the perimeter walls and obstacles, area-weighted, count = `landmark_density_per_m2 * total_face_area`, height uniform in [0.3, wall_height - 0.2], each lifted 0.02 m along the face normal. Landmark ids are 0..N-1.
  - `render_world(map, calibration) -> str` and `render_robot(calibration) -> str` from the templates in `templates/`.
  - `render_textures(out_dir)`: write `checker.png`, `noise.png`, `plain.png` (512x512, grayscale) with numpy/pillow, deterministic.
  - CLI: `python -m ee585_sim.maps build maps/<name>.yaml --out build/<name>/` writes `world.sdf`, `robot.sdf`, `landmarks.json`, textures.
- Map yaml schema:

```yaml
name: demo_room
seed: 1
arena: {size: [12.0, 12.0], wall_height: 2.5}
obstacles:                       # vertical boxes
  - {center: [2.0, 1.0], size: [1.0, 1.0, 1.5], yaw: 0.3}
texture: {name: noise, tile_m: 1.0}       # checker | noise | plain
landmark_density_per_m2: 1.0
lighting: {ambient: 0.4, sun_intensity: 0.8, sun_direction: [-0.5, 0.3, -1.0]}
path:
  waypoints: [[-4,-4],[4,-4],[4,4],[-4,4],[-4,-4]]
  speed_mps: 0.5
  accel_max_mps2: 0.5
  lookahead_m: 0.6
  tail_s: 1.0
```

- World SDF content (generated):
  - Physics: `max_step_size` 0.001, `real_time_factor` 1.0. Gravity `0 0 -9.81`.
  - Systems: physics, user-commands, scene-broadcaster, sensors (`render_engine` ogre2), imu.
  - Ground plane (friction default), perimeter walls and obstacles with the chosen texture (PBR albedo map), light(s), landmarks as static spheres (radius 0.03 m, high-contrast flat color, one distinct color per landmark is not required).
- Robot SDF (generated): model `robot`, link `base_link` is frame B with its origin at the IMU. Chassis box 0.40 x 0.30 x 0.15 m, mass 5 kg. Two drive wheels (radius 0.05 m, separation 0.30 m) and a frictionless caster. Plugins: `gz-sim-diff-drive-system` (subscribes `/cmd_vel`), `gz-sim-pose-publisher-system` (publishes the pose of `base_link`).
  - IMU sensor on `base_link` at pose identity: `update_rate` from calibration, `topic` `imu`, noise `gaussian` for each of the 3 gyro and 3 accel axes with `mean 0`, `stddev` from the formula in section 5, `bias_mean 0`, `bias_stddev 0`, `dynamic_bias_stddev 0`, `dynamic_bias_correlation_time 0`.
  - Camera sensor on `base_link` at pose `p_BC` with zero rotation (Gazebo's camera looks along its own +x, which is body forward; the optical convention is applied in `projection.py` via `R_BC`, not in the SDF). `update_rate` 30, image `width`, `height`, `format` `L8` (if Gazebo rejects `L8`, use `R8G8B8` and convert to mono in the provider with weights 0.299, 0.587, 0.114 rounded to nearest, and record it in section 10), horizontal FOV from calibration, clip near 0.1 far 30, NO image noise, `topic` `camera/image`.
- `config/bridge.yaml` for `ros_gz_bridge` (verify actual gz topic names with `gz topic -l` and fix): `/clock` (Clock), `/imu` (gz.msgs.IMU -> sensor_msgs/Imu), `/camera/image` (gz.msgs.Image -> sensor_msgs/Image), the robot pose topic (gz.msgs.Pose_V -> tf2_msgs/TFMessage), `/cmd_vel` (ROS to gz Twist).
- `launch/sim.launch.py`: starts `gz sim -s -r --headless-rendering build/<name>/world.sdf` (add `-g` GUI optionally via an argument), the bridge with `use_sim_time`, and, in later phases, `follower` and `provider_gz`. Headless rendering on a machine without GPU may need `LIBGL_ALWAYS_SOFTWARE=1` or `xvfb-run`; record what worked in section 10.

Acceptance (manual or scripted, results recorded in section 10):
1. The world builds and runs for 10 s of sim time without errors; with the GUI, the robot, walls, textures and landmark spheres are visible.
2. Static IMU test (robot at rest, 10 s, 2000 samples): mean accel within 0.05 of (0, 0, 9.81); per-axis accel std within 10% of 0.0283; per-axis gyro std within 10% of 0.0024; gyro mean magnitude below 5e-4 rad/s. This also confirms the specific-force sign and the zero bias.
3. IMU stamps are multiples of 5 ms with no gaps (all gaps equal 5 ms); camera stamps have gaps within 33.3 ms +- 1.5 ms.
4. Publishing `/cmd_vel` linear.x = 0.5 for 4 s moves the robot 2.0 m +- 0.2 m along +x as reported by the truth pose; the pose is in the WORLD frame and body-forward.

### Phase 4: projection library

`projection.py` is pure Python/numpy with no ROS imports.

API:

```python
class Projector:
    def __init__(self, calib: dict, landmarks: np.ndarray, occluders: list, seed: int)
    def project(self, t_ns: int, p_WB: np.ndarray, q_WB: np.ndarray) -> tuple[list[Obs], list[Obs]]
        # returns (noisy, truth) observations, same ids and order, ids ascending
```

Algorithm for each landmark (all in double precision):
1. `T_WC = T_WB * T_BC`. Landmark in camera frame `p_C = R_WC^T (p_W - p_WC)`.
2. Reject if `Z <= 0`, or range `|p_C|` outside `[min_range_m, max_range_m]`.
3. `u = fx X/Z + cx`, `v = fy Y/Z + cy`. Reject if `u` not in `[0, width)` or `v` not in `[0, height)` (noiseless values decide visibility).
4. Occlusion: for each occluder (an oriented box: center, size, yaw, including the four perimeter walls as thin boxes), do a slab ray test of the segment from the camera center to the landmark. The landmark is occluded if the segment hits a box at parameter `s` in `(0, 1 - 1e-6)`. Landmarks lie 0.02 m outside their own face, so their own face does not occlude them when seen from outside.
5. Noisy value: `u + N(0, sigma^2)`, `v + N(0, sigma^2)` with one `numpy.random.Generator(PCG64(seed))` consumed in landmark-id order; independent draws per axis. Noise does not change visibility.

Tests (`sim/ee585_sim/tests/`, no Gazebo):
1. A landmark on the optical axis projects to `(cx, cy)` exactly.
2. A landmark at `(1, 0, 4)` in C with the default intrinsics projects to `(fx*0.25 + cx, cy)`.
3. Behind the camera, out of range, and out of FOV are rejected.
4. A box between camera and landmark occludes; moving the box aside makes the landmark visible.
5. Sampling: over 10,000 draws the noise std is within 3% of sigma and the mean error is below `0.05*sigma`.
6. Same seed gives identical output; truth observations never contain noise.
7. Pose consistency: yawing the body by +90 deg (left turn) moves a landmark that was on the left to the image center.

Integration check (needs Gazebo, from the phase 3 world): place the robot at a known pose with the follower not running, capture one rendered frame and the truth pose from the same timestamp, compute projections, and for 5 visible landmarks find the centroid of the sphere's bright pixels in the image. Pixel distance from the projected `(u, v)` must be below 1.0 px. Record the observed offset (whether pixel centers are at integer or half-integer coordinates) in section 10; if the offset is systematic (about 0.5 px), define the convention in `projection.py` and in the proto comment, and re-run.

### Phase 5: path follower and ground truth

`follower.py` (ROS node, `use_sim_time`):
- Reads the map's `path` parameters. For the first `initial_rest_s` of simulation time publish zero velocity.
- Pure pursuit: target point at `lookahead_m` along the polyline ahead of the closest point; angular velocity from the heading error with a gain of 1.5 and clamp to +- 1.0 rad/s; linear speed ramps toward `speed_mps` with slope limited by `accel_max_mps2`, and ramps down over the last `speed^2 / (2*accel_max)` metres before the final waypoint.
- At the end of the polyline publish zero velocity, wait `tail_s`, then publish `std_msgs/Empty` once on `/run/done`.

Acceptance:
1. On `demo_room`, the robot completes the path; position error to the polyline is below 0.3 m everywhere (from truth poses).
2. Robot never intersects an obstacle (obstacles are placed away from the path in `demo_room`).
3. `/run/done` is published exactly once.
4. Truth roll and pitch stay below 2 deg throughout.

### Phase 6: Gazebo live provider

`provider_gz.py` (ROS node, `use_sim_time`). Subscriptions use reliable QoS with depth 10000. Inputs: IMU, image, robot pose, `/run/done`.

Procedure:
1. Convert each sensor message into a `Record` payload (without `seq`). `t_ns` is the message header stamp in nanoseconds. Gyro and accel are copied from the ROS message as they are (body frame, specific force, noise already added by Gazebo).
2. Pose buffer holds the last 2000 poses. For each image at `t`, find the poses that bracket `t`, interpolate linearly in position and by slerp in orientation, and call `Projector.project`. If the newest pose is older than `t`, keep the image pending.
3. A completed camera frame yields two stream records at the same `t_ns`: image, then projections (empty if nothing is visible), and one `TruthProjections` truth record.
4. Merge buffer: items sorted by `(t_ns, kind)` with kind order IMU < image < projections. The watermark is `min(latest IMU t_ns, latest COMPLETED camera frame t_ns)`. Release, in order, every item with `t_ns <= watermark`; assign `seq` at release.
5. Truth poses: forward the bridged pose as `TruthRecord.pose`, throttled to at most one per 5 ms of sim time, on the `truth` topic.
6. Self-check: each source (IMU, image, pose) must have successive stamps no more than `1.5 * nominal period` apart (nominal period from calibration; use 1.5 * 1 ms for pose). A larger gap means a message was lost before reaching MQTT: abort the run (`Link.abort("source gap: <name>")`) and exit non-zero.
7. On `/run/done`: flush the merge buffer in order, then perform the clean end of section 6.
8. Pacing is implicit (Gazebo real_time_factor = 1.0). Publishing is bounded as in section 6; if the bound blocks for more than 2 s, abort with `detail: "publisher backlog"`.

Acceptance (live Gazebo, `demo_room`, with the recorder running):
1. Run completes: recorder log has `meta.json.terminal == COMPLETED`.
2. `stream.bin`: `seq` gapless, `t_ns` non-decreasing, IMU records spaced exactly 5,000,000 ns, camera frames within 33.3 ms +- 1.5 ms of each other, every image record immediately followed by one projections record with the same `t_ns`.
3. Ground-truth consistency: for 20 random frames, reproject the scene landmarks with the logged truth pose (interpolated) and `Projector` without noise; the result matches `TruthProjections` within 1e-6 px.
4. Noisy minus noiseless projection errors over the whole run have std within 5% of `pixel_noise_sigma`.
5. IMU consistency: dead-reckon the logged IMU (using the truth orientation to remove gravity and rotate to the world) over a 3 s excerpt starting at rest; position drift versus truth is below 0.15 m. (This validates sign conventions and noise level together; it is not a pass-or-fail of the algorithm.)
6. Killing the provider with `kill -9` during a run gives `ABORTED` at the consumer within 4 s.
7. Replay of the recorded log reproduces the byte-identical stream (as in phase 2 check 1).

### Phase 7: map set and reference logs

Deliverables: at least 4 map yaml files exercising the allowed variation:
- `demo_room`: the baseline from phase 3.
- `sparse_texture`: same geometry, `plain` texture, `landmark_density_per_m2: 0.2`.
- `dense_features`: `noise` texture, `landmark_density_per_m2: 3.0`.
- `dim_lighting`: baseline geometry, ambient 0.1, sun intensity 0.2.
- Optionally one with a different arena size and a longer path (at least 60 s of motion).

For each map: build, run live, record to `runs/<map>/`. Write `sim/README` content into a short `docs/usage.md` describing: how to start the broker, run the simulation, record, replay, and how an algorithm uses the consumer (a 15-line example). Keep `runs/` out of git; store only the commands that regenerate them.

Acceptance: every map's run is `COMPLETED`; each log satisfies phase 6 checks 2-5; mean visible landmarks per frame differ across maps in the expected order (`dense_features` > `demo_room` > `sparse_texture`).

### Phase 8 (optional, only if the owner asks): moving occluders

A map may list moving boxes with a time-parameterised path. The projector needs their poses at the image timestamp; the robot-side occluder list becomes time-dependent. The occluder poses can be published by a small ROS node on a known topic and merged with the same pose-buffer logic. Do not start this without an explicit request.

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
- Dropout is applied by the algorithm-side harness to records after they leave the consumer. The provider never drops records.

---

## 9. Out of scope (do not implement)

IMU bias, bias random walk, scale-factor or misalignment errors; lens distortion; rolling shutter; camera-IMU time offset (zero by construction); stereo and depth; image noise in the rendered image; motion blur; wheel-odometry output; real-dataset adapters (a future provider that publishes a public dataset in this same contract); loop closure; the MSCKF, baselines, metrics, and the dropout harness.

---

## 10. Execution log (the executing agent appends here)

Format: `YYYY-MM-DD | phase | note`. Record installed versions, Gazebo topic names that differ from section 7, the pixel-center convention found in phase 4, the headless rendering recipe that worked, any `L8` fallback, measured acceptance numbers, and every deviation with its reason.
