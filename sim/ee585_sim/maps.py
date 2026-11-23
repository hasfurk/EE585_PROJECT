"""Map yaml, landmark placement, MJCF, and textures."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from string import Template
from typing import Any

import numpy as np
from PIL import Image
import yaml

WALL_THICKNESS_M = 0.1
LANDMARK_OFFSET_M = 0.02
LANDMARK_Z_MIN = 0.3
LANDMARK_Z_MARGIN = 0.2
_TEXTURES = ("checker", "noise", "plain")
_TEMPLATE = Path(__file__).resolve().parents[1] / "templates" / "world.xml.in"
_CALIBRATION = Path(__file__).resolve().parents[1] / "config" / "calibration.yaml"


class _Face:
    def __init__(
        self,
        origin: tuple[float, float],
        axis: tuple[float, float],
        width: float,
        height: float,
        normal: tuple[float, float],
    ) -> None:
        self.origin = origin
        self.axis = axis
        self.width = width
        self.height = height
        self.normal = normal


def load_map(path: Path) -> dict[str, Any]:
    raw = yaml.safe_load(Path(path).read_text())
    if not isinstance(raw, dict):
        raise ValueError("map yaml must be a mapping")
    arena = raw["arena"]
    size = (_num(arena["size"][0], "arena.size[0]"), _num(arena["size"][1], "arena.size[1]"))
    if size[0] <= 0.0 or size[1] <= 0.0:
        raise ValueError("arena size must be positive")
    wall_height = _num(arena["wall_height"], "arena.wall_height")
    _require_face_height(wall_height, "arena.wall_height")
    obstacles = []
    for index, item in enumerate(raw["obstacles"]):
        center = (_num(item["center"][0], "center"), _num(item["center"][1], "center"))
        box = tuple(_num(item["size"][axis], "size") for axis in range(3))
        if min(box) <= 0.0:
            raise ValueError(f"obstacles[{index}].size must be positive")
        _require_face_height(box[2], f"obstacles[{index}].size[2]")
        obstacles.append(
            {
                "center": center,
                "size": box,
                "yaw": _num(item["yaw"], "yaw"),
            }
        )
    texture_name = str(raw["texture"]["name"])
    if texture_name not in _TEXTURES:
        raise ValueError(f"texture.name must be one of {_TEXTURES}")
    tile_m = _num(raw["texture"]["tile_m"], "texture.tile_m")
    if tile_m <= 0.0:
        raise ValueError("texture.tile_m must be positive")
    density = _num(raw["landmark_density_per_m2"], "landmark_density_per_m2")
    if density < 0.0:
        raise ValueError("landmark_density_per_m2 must be non-negative")
    lighting = raw["lighting"]
    direction = tuple(_num(lighting["sun_direction"][axis], "sun_direction") for axis in range(3))
    if math.sqrt(sum(component * component for component in direction)) == 0.0:
        raise ValueError("sun_direction must be non-zero")
    waypoints = []
    for point in raw["path"]["waypoints"]:
        waypoints.append((_num(point[0], "waypoint"), _num(point[1], "waypoint")))
    if not waypoints:
        raise ValueError("path.waypoints must not be empty")
    speed = _num(raw["path"]["speed_mps"], "speed_mps")
    accel = _num(raw["path"]["accel_max_mps2"], "accel_max_mps2")
    lookahead = _num(raw["path"]["lookahead_m"], "lookahead_m")
    tail = _num(raw["path"]["tail_s"], "tail_s")
    if speed <= 0.0 or accel <= 0.0 or lookahead <= 0.0 or tail < 0.0:
        raise ValueError("path speeds and lookahead must be positive and tail_s non-negative")
    return {
        "name": str(raw["name"]),
        "seed": int(raw["seed"]),
        "arena": {"size": size, "wall_height": wall_height},
        "obstacles": obstacles,
        "texture": {"name": texture_name, "tile_m": tile_m},
        "landmark_density_per_m2": density,
        "lighting": {
            "ambient": _num(lighting["ambient"], "ambient"),
            "sun_intensity": _num(lighting["sun_intensity"], "sun_intensity"),
            "sun_direction": direction,
        },
        "path": {
            "waypoints": waypoints,
            "speed_mps": speed,
            "accel_max_mps2": accel,
            "lookahead_m": lookahead,
            "tail_s": tail,
        },
    }


def generate_landmarks(map: dict[str, Any]) -> list[dict[str, Any]]:
    faces = _faces(map)
    total = sum(face.width * face.height for face in faces)
    count = math.floor(map["landmark_density_per_m2"] * total + 0.5)
    generator = np.random.Generator(np.random.PCG64(map["seed"]))
    landmarks: list[dict[str, Any]] = []
    for index in range(count):
        choice = float(generator.random()) * total
        cursor = 0.0
        chosen = faces[-1]
        for face in faces:
            cursor += face.width * face.height
            if choice < cursor:
                chosen = face
                break
        along = float(generator.random())
        z_hi = chosen.height - LANDMARK_Z_MARGIN
        z = LANDMARK_Z_MIN + float(generator.random()) * (z_hi - LANDMARK_Z_MIN)
        origin_x, origin_y = chosen.origin
        axis_x, axis_y = chosen.axis
        normal_x, normal_y = chosen.normal
        landmarks.append(
            {
                "id": index,
                "p_W": (
                    origin_x + along * chosen.width * axis_x + LANDMARK_OFFSET_M * normal_x,
                    origin_y + along * chosen.width * axis_y + LANDMARK_OFFSET_M * normal_y,
                    z,
                ),
            }
        )
    return landmarks


def render_world(map: dict[str, Any], calibration: dict[str, Any]) -> str:
    repeat = 1.0 / map["texture"]["tile_m"]
    direction = _unit(map["lighting"]["sun_direction"])
    ambient = map["lighting"]["ambient"]
    diffuse = map["lighting"]["sun_intensity"]
    lines = [_wall_xml(map), _obstacle_xml(map), _landmark_xml(generate_landmarks(map))]
    geoms = "\n".join(line for line in lines if line)
    return Template(_TEMPLATE.read_text()).substitute(
        NAME=map["name"],
        OFFWIDTH=str(int(calibration["width"])),
        OFFHEIGHT=str(int(calibration["height"])),
        TEXTURE=f"{map['texture']['name']}.png",
        TEXREPEAT=f"{_num_text(repeat)} {_num_text(repeat)}",
        LIGHT_DIR=" ".join(_num_text(component) for component in direction),
        LIGHT_AMBIENT=" ".join(_num_text(ambient) for _ in range(3)),
        LIGHT_DIFFUSE=" ".join(_num_text(diffuse) for _ in range(3)),
        FOVY=_num_text(calibration["fovy_deg"]),
        GEOMS=geoms,
    )


def occluders(map: dict[str, Any]) -> list[dict[str, Any]]:
    """Oriented boxes: four perimeter walls and every obstacle. Center, full size, yaw."""
    half_x = map["arena"]["size"][0] / 2.0
    half_y = map["arena"]["size"][1] / 2.0
    height = map["arena"]["wall_height"]
    thickness = WALL_THICKNESS_M
    boxes = [
        {
            "name": "wall_s",
            "center": (0.0, -half_y - thickness / 2.0, height / 2.0),
            "size": (map["arena"]["size"][0] + 2.0 * thickness, thickness, height),
            "yaw": 0.0,
        },
        {
            "name": "wall_n",
            "center": (0.0, half_y + thickness / 2.0, height / 2.0),
            "size": (map["arena"]["size"][0] + 2.0 * thickness, thickness, height),
            "yaw": 0.0,
        },
        {
            "name": "wall_e",
            "center": (half_x + thickness / 2.0, 0.0, height / 2.0),
            "size": (thickness, map["arena"]["size"][1], height),
            "yaw": 0.0,
        },
        {
            "name": "wall_w",
            "center": (-half_x - thickness / 2.0, 0.0, height / 2.0),
            "size": (thickness, map["arena"]["size"][1], height),
            "yaw": 0.0,
        },
    ]
    for index, obstacle in enumerate(map["obstacles"]):
        size_x, size_y, size_z = obstacle["size"]
        center_x, center_y = obstacle["center"]
        boxes.append(
            {
                "name": f"obstacle_{index}",
                "center": (center_x, center_y, size_z / 2.0),
                "size": (size_x, size_y, size_z),
                "yaw": obstacle["yaw"],
            }
        )
    return boxes


def render_textures(out_dir: Path) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    columns = np.arange(512) // 64
    rows = np.arange(512) // 64
    checker = np.where((rows[:, None] + columns[None, :]) % 2 == 0, 32, 220).astype(np.uint8)
    Image.fromarray(checker, mode="L").save(out_dir / "checker.png")
    Image.fromarray(np.full((512, 512), 180, dtype=np.uint8), mode="L").save(out_dir / "plain.png")
    noise = np.random.Generator(np.random.PCG64(0)).integers(0, 256, size=(512, 512), dtype=np.uint8)
    Image.fromarray(noise, mode="L").save(out_dir / "noise.png")


def build(map_path: Path, out_dir: Path, calibration_path: Path | None = None) -> None:
    from ee585_sim.model import load_calibration

    spec = load_map(map_path)
    calibration = load_calibration(calibration_path or _CALIBRATION)
    out_dir.mkdir(parents=True, exist_ok=True)
    render_textures(out_dir)
    landmarks = generate_landmarks(spec)
    (out_dir / "world.xml").write_text(render_world(spec, calibration))
    (out_dir / "landmarks.json").write_text(
        json.dumps(
            {"landmarks": [{"id": item["id"], "p_W": list(item["p_W"])} for item in landmarks]}
        )
        + "\n"
    )


def main(argv: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("command")
    parser.add_argument("map_yaml")
    parser.add_argument("--out", required=True)
    args = parser.parse_args(argv)
    if args.command != "build":
        raise SystemExit(f"unknown command {args.command}")
    build(Path(args.map_yaml), Path(args.out))


def _faces(map: dict[str, Any]) -> list[_Face]:
    width, depth = map["arena"]["size"]
    half_x = width / 2.0
    half_y = depth / 2.0
    height = map["arena"]["wall_height"]
    _require_face_height(height, "arena.wall_height")
    # Inner rectangle only. Corner caps of the wall boxes are not free space.
    faces = [
        _Face((-half_x, -half_y), (1.0, 0.0), width, height, (0.0, 1.0)),
        _Face((half_x, -half_y), (0.0, 1.0), depth, height, (-1.0, 0.0)),
        _Face((-half_x, half_y), (1.0, 0.0), width, height, (0.0, -1.0)),
        _Face((-half_x, -half_y), (0.0, 1.0), depth, height, (1.0, 0.0)),
    ]
    for index, obstacle in enumerate(map["obstacles"]):
        faces.extend(_obstacle_faces(obstacle, f"obstacles[{index}]"))
    return faces


def _obstacle_faces(obstacle: dict[str, Any], label: str) -> list[_Face]:
    center_x, center_y = obstacle["center"]
    size_x, size_y, size_z = obstacle["size"]
    _require_face_height(size_z, label)
    yaw = obstacle["yaw"]
    local = (
        ((size_x / 2.0, -size_y / 2.0), (0.0, 1.0), size_y, (1.0, 0.0)),
        ((-size_x / 2.0, -size_y / 2.0), (0.0, 1.0), size_y, (-1.0, 0.0)),
        ((-size_x / 2.0, size_y / 2.0), (1.0, 0.0), size_x, (0.0, 1.0)),
        ((-size_x / 2.0, -size_y / 2.0), (1.0, 0.0), size_x, (0.0, -1.0)),
    )
    faces = []
    for origin, axis, width, normal in local:
        rotated_origin = _rot(yaw, origin[0], origin[1])
        rotated_axis = _rot(yaw, axis[0], axis[1])
        rotated_normal = _rot(yaw, normal[0], normal[1])
        faces.append(
            _Face(
                (center_x + rotated_origin[0], center_y + rotated_origin[1]),
                rotated_axis,
                width,
                size_z,
                rotated_normal,
            )
        )
    return faces


def _wall_xml(map: dict[str, Any]) -> str:
    return "\n".join(_box_xml(box, "surf") for box in occluders(map) if box["name"].startswith("wall_"))


def _obstacle_xml(map: dict[str, Any]) -> str:
    return "\n".join(_box_xml(box, "surf") for box in occluders(map) if box["name"].startswith("obstacle_"))


def _box_xml(box: dict[str, Any], material: str) -> str:
    center = box["center"]
    size = box["size"]
    return "    " + _geom(
        box["name"],
        "box",
        " ".join(_num_text(component / 2.0) for component in size),
        " ".join(_num_text(component) for component in center),
        material=material,
        euler=f"0 0 {_num_text(box['yaw'])}",
    )


def _landmark_xml(landmarks: list[dict[str, Any]]) -> str:
    lines = []
    for landmark in landmarks:
        x, y, z = landmark["p_W"]
        lines.append(
            "    "
            + _geom(
                f"lm{landmark['id']}",
                "sphere",
                "0.03",
                f"{_num_text(x)} {_num_text(y)} {_num_text(z)}",
                material="mark",
                contype="0",
                conaffinity="0",
            )
        )
    return "\n".join(lines)


def _geom(
    name: str,
    geom_type: str,
    size: str,
    pos: str,
    *,
    material: str,
    euler: str | None = None,
    contype: str | None = None,
    conaffinity: str | None = None,
) -> str:
    extra = ""
    if euler is not None:
        extra += f' euler="{euler}"'
    if contype is not None:
        extra += f' contype="{contype}" conaffinity="{conaffinity}"'
    return f'<geom name="{name}" type="{geom_type}" size="{size}" pos="{pos}" material="{material}"{extra}/>'


def _require_face_height(height: float, label: str) -> None:
    if height - LANDMARK_Z_MARGIN <= LANDMARK_Z_MIN:
        raise ValueError(f"{label} is too short for a landmark")


def _rot(yaw: float, x: float, y: float) -> tuple[float, float]:
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    return (cosine * x - sine * y, sine * x + cosine * y)


def _unit(vector: tuple[float, float, float]) -> tuple[float, float, float]:
    norm = math.sqrt(sum(component * component for component in vector))
    return (vector[0] / norm, vector[1] / norm, vector[2] / norm)


def _num(value: object, label: str) -> float:
    try:
        number = float(value)  # type: ignore[arg-type]
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{label} must be a number") from exc
    if not math.isfinite(number):
        raise ValueError(f"{label} must be finite")
    return number


def _num_text(value: float) -> str:
    return format(float(value), ".17g")


if __name__ == "__main__":
    main()
