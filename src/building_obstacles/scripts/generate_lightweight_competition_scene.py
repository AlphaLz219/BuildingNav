#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import random
import sys
from typing import Any


def _ensure_generator_on_path() -> None:
    try:
        import building_generator_core  # noqa: F401
        return
    except ImportError:
        pass

    scripts_dir = Path(__file__).resolve().parent
    src_dir = scripts_dir.parent.parent
    candidate = src_dir / "building_generator_core"
    if candidate.exists():
        sys.path.insert(0, str(candidate))


_ensure_generator_on_path()

from building_generator_core.layout import (  # noqa: E402
    BuildingLayout,
    DoorSpec,
    ElevatorSpec,
    FloorLayout,
    FurnitureSpec,
    Rect2D,
    RoomSpec,
)
from building_generator_core.lightweight_exporter import export_lightweight_sdf  # noqa: E402

import generate_competition_scene as full_scene  # noqa: E402


DEFAULT_WIDTH = 10.0
DEFAULT_LENGTH = 14.0
DEFAULT_FLOOR_HEIGHT = 2.6
DEFAULT_WALL_HEIGHT = 2.35


def main(argv: list[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    seed = args.seed if args.seed is not None else random.SystemRandom().randint(1, 2**31 - 1)
    rng = random.Random(seed)
    output_dir = Path(args.output_dir).resolve()
    results_dir = Path(args.results_dir).resolve() if args.results_dir else output_dir.parent / "results"
    output_dir.mkdir(parents=True, exist_ok=True)
    results_dir.mkdir(parents=True, exist_ok=True)

    floor_count = _sample_count(args.floor_count, rng)
    rooms_per_floor = _sample_count(args.rooms_per_floor, rng)
    include_elevator = args.include_elevator
    include_stairs = args.include_stairs
    if floor_count > 1 and not include_elevator and not include_stairs:
        raise ValueError("multi-floor lightweight buildings require --include-elevator or --include-stairs")

    layout = generate_lightweight_layout(
        seed=seed,
        floor_count=floor_count,
        rooms_per_floor=rooms_per_floor,
        width=args.width,
        length=args.length,
        include_elevator=include_elevator,
        include_stairs=include_stairs,
        include_furniture=args.include_furniture,
        floor_height=args.floor_height,
        wall_height=args.wall_height,
        corridor_width=args.corridor_width,
        lobby_depth=args.lobby_depth,
    )
    artifact_paths = export_lightweight_sdf(
        layout,
        output_dir,
        include_roof=args.include_roof,
        include_elevator=include_elevator,
        include_stairs=include_stairs,
        include_furniture=args.include_furniture,
    )

    obstacle_rng = random.Random(seed ^ 0x5EED5EED)
    danger_count = _sample_count(args.danger_count, obstacle_rng)
    distractor_count = _sample_count(args.distractor_count, obstacle_rng)
    sources = full_scene._place_sources(layout, obstacle_rng, danger_count, distractor_count)

    world_path = output_dir / "competition_scene.world"
    full_scene._write_world_with_sources(Path(artifact_paths.world_sdf), world_path, sources)
    Path(artifact_paths.world_sdf).write_text(world_path.read_text(encoding="utf-8"), encoding="utf-8")

    truth_data = full_scene._build_truth_data(layout, seed, sources)
    truth_path = results_dir / "danger_truth.json"
    output_truth_path = output_dir / "danger_truth.json"
    truth_json = json.dumps(truth_data, indent=2, ensure_ascii=False) + "\n"
    truth_path.write_text(truth_json, encoding="utf-8")
    output_truth_path.write_text(truth_json, encoding="utf-8")

    building_config_path = output_dir / "building_config.json"
    building_config_path.write_text(
        json.dumps(_build_building_config(layout, seed, world_path, sources, args), indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )

    manifest = {
        "schema": "lightweight_competition_scene_manifest_v1",
        "seed": seed,
        "world_file": str(world_path),
        "building_config": str(building_config_path),
        "layout_metadata": artifact_paths.layout_metadata,
        "door_config": artifact_paths.door_config,
        "elevator_config": artifact_paths.elevator_config,
        "validation_report": artifact_paths.validation_report,
        "truth_file": str(truth_path),
        "output_truth_file": str(output_truth_path),
        "danger_count": danger_count,
        "distractor_count": distractor_count,
        "source_count": len(sources),
        "lightweight_options": {
            "include_roof": args.include_roof,
            "include_elevator": include_elevator,
            "include_stairs": include_stairs,
            "include_furniture": args.include_furniture,
            "width": args.width,
            "length": args.length,
            "floor_height": args.floor_height,
            "wall_height": args.wall_height,
            "corridor_width": args.corridor_width,
            "lobby_depth": args.lobby_depth,
        },
        "robot_start": {
            "x": args.robot_x,
            "y": args.robot_y,
            "z": args.robot_z,
            "yaw": args.robot_yaw,
        },
        "competition_interfaces": {
            "velocity_command_topic": "/cmd_vel",
            "odometry_topic": "/Odometry_gazebo",
            "door_service": "/set_door_state",
            "elevator_service": "/call_elevator" if include_elevator else None,
            "truth_file_for_referee": str(truth_path),
            "team_detection_file": str(results_dir / "detected_danger.json"),
        },
    }
    manifest_path = output_dir / "scene_manifest.json"
    manifest_path.write_text(json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    print(json.dumps(manifest, indent=2, sort_keys=True))
    return 0


def generate_lightweight_layout(
    *,
    seed: int,
    floor_count: int,
    rooms_per_floor: int,
    width: float,
    length: float,
    include_elevator: bool,
    include_stairs: bool,
    include_furniture: bool,
    floor_height: float,
    wall_height: float,
    corridor_width: float,
    lobby_depth: float,
) -> BuildingLayout:
    if width < 6.0:
        raise ValueError("lightweight building width must be >= 6.0")
    if length < 8.0:
        raise ValueError("lightweight building length must be >= 8.0")
    if rooms_per_floor < 1:
        raise ValueError("rooms_per_floor must be >= 1")

    half_width = width / 2.0
    room_depth = half_width - corridor_width / 2.0 - 0.35
    if room_depth < 1.7:
        raise ValueError("building is too narrow for rooms around the corridor")

    core_width = min(1.8, max(1.2, room_depth - 0.2))
    core_length = min(2.4, max(1.5, lobby_depth - 0.8))
    stair_bounds = Rect2D(
        x_min=-corridor_width / 2.0 - 0.35 - core_width,
        x_max=-corridor_width / 2.0 - 0.35,
        y_min=0.7,
        y_max=0.7 + core_length,
    )
    elevator_bounds = Rect2D(
        x_min=corridor_width / 2.0 + 0.35,
        x_max=corridor_width / 2.0 + 0.35 + core_width,
        y_min=0.7,
        y_max=0.7 + core_length,
    )
    if include_stairs and stair_bounds.x_min < -half_width + 0.25:
        raise ValueError("building is too narrow for lightweight stair core")
    if include_elevator and elevator_bounds.x_max > half_width - 0.25:
        raise ValueError("building is too narrow for lightweight elevator core")

    corridor_start = max(lobby_depth, stair_bounds.y_max + 0.6 if include_stairs else lobby_depth)
    if include_elevator:
        corridor_start = max(corridor_start, elevator_bounds.y_max + 0.6)
    corridor_end = length - 0.2
    usable_span = corridor_end - corridor_start
    segment_count = math.ceil(rooms_per_floor / 2.0)
    if usable_span / max(segment_count, 1) < 1.6:
        raise ValueError("building is too short for the requested room count")

    floors: list[FloorLayout] = []
    doors: list[DoorSpec] = [
        DoorSpec(
            id="main_entrance",
            floor_index=0,
            kind="main_entrance",
            pose=(0.0, 0.0, 1.1, 0.0, 0.0, math.pi / 2.0),
            width=1.6,
            height=2.2,
            initial_open=True,
            dynamic=True,
        )
    ]
    elevator_lobbies: dict[int, tuple[float, float, float, float, float, float]] = {}
    target_points: dict[str, Any] = {
        "main_entrance": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        "rooms": {},
        "stairs": {},
        "elevators": {},
    }

    for floor_index in range(floor_count):
        elevation = floor_index * floor_height
        rooms = _build_lightweight_rooms(
            floor_index=floor_index,
            elevation=elevation,
            room_count=rooms_per_floor,
            room_depth=room_depth,
            corridor_width=corridor_width,
            corridor_start=corridor_start,
            corridor_end=corridor_end,
            include_furniture=include_furniture,
        )
        if include_elevator:
            elevator_door_pose = (
                elevator_bounds.x_min,
                elevator_bounds.center[1],
                elevation + 1.1,
                0.0,
                0.0,
                math.pi,
            )
            doors.append(
                DoorSpec(
                    id=f"elevator_floor_{floor_index}",
                    floor_index=floor_index,
                    kind="elevator",
                    pose=elevator_door_pose,
                    width=1.2,
                    height=2.1,
                    initial_open=(floor_index == 0),
                    dynamic=True,
                )
            )
            elevator_lobbies[floor_index] = elevator_door_pose
            target_points["elevators"][str(floor_index)] = list(elevator_door_pose)
        if include_stairs:
            target_points["stairs"][str(floor_index)] = [
                stair_bounds.x_max - 0.45,
                stair_bounds.center[1],
                elevation,
                0.0,
                0.0,
                0.0,
            ]
        for room in rooms:
            target_points["rooms"][room.id] = list(room.goal_pose)

        floors.append(
            FloorLayout(
                floor_index=floor_index,
                elevation=elevation,
                lobby_bounds=Rect2D(-half_width, half_width, 0.0, corridor_start),
                corridor_bounds=Rect2D(-corridor_width / 2.0, corridor_width / 2.0, corridor_start, corridor_end),
                stair_bounds=stair_bounds,
                elevator_bounds=elevator_bounds,
                rooms=rooms,
                reachability={
                    "stair": include_stairs,
                    "elevator": include_elevator,
                    "rooms": {room.id: True for room in rooms},
                },
            )
        )

    elevator_specs = []
    if include_elevator:
        elevator_specs.append(
            ElevatorSpec(
                id="elevator_main",
                shaft_bounds=elevator_bounds,
                served_floors=list(range(floor_count)),
                current_floor=0,
                car_size=(max(elevator_bounds.width - 0.35, 1.0), max(elevator_bounds.length - 0.35, 1.2), 2.15),
                lobby_positions=elevator_lobbies,
            )
        )

    metadata = {
        "seed": seed,
        "generator": "lightweight_competition_scene",
        "include_roof": False,
        "include_elevator": include_elevator,
        "include_stairs": include_stairs,
        "include_furniture": include_furniture,
        "room_counts": [rooms_per_floor for _ in range(floor_count)],
        "connectivity": _connectivity_label(include_elevator, include_stairs),
    }
    signature = _build_signature(
        footprint={"width": width, "length": length},
        floors=floors,
        doors=doors,
        elevators=elevator_specs,
        metadata=metadata,
    )
    return BuildingLayout(
        model_name="generated_lightweight_building",
        footprint={"width": width, "length": length},
        floor_height=floor_height,
        wall_height=wall_height,
        entrance_pose=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        floors=floors,
        door_specs=doors,
        elevator_specs=elevator_specs,
        signature=signature,
        target_points=target_points,
        metadata=metadata,
    )


def _build_lightweight_rooms(
    *,
    floor_index: int,
    elevation: float,
    room_count: int,
    room_depth: float,
    corridor_width: float,
    corridor_start: float,
    corridor_end: float,
    include_furniture: bool,
) -> list[RoomSpec]:
    rooms: list[RoomSpec] = []
    segment_count = math.ceil(room_count / 2.0)
    segment_length = (corridor_end - corridor_start) / max(segment_count, 1)
    for room_index in range(room_count):
        side = "left" if room_index % 2 == 0 else "right"
        segment_index = room_index // 2
        y_min = corridor_start + segment_index * segment_length
        y_max = y_min + segment_length
        if side == "left":
            bounds = Rect2D(-corridor_width / 2.0 - room_depth, -corridor_width / 2.0, y_min, y_max)
            door_pose = (bounds.x_max, bounds.center[1], elevation + 1.1, 0.0, 0.0, 0.0)
            goal_pose = (bounds.center[0] + 0.35, bounds.center[1], elevation, 0.0, 0.0, 0.0)
        else:
            bounds = Rect2D(corridor_width / 2.0, corridor_width / 2.0 + room_depth, y_min, y_max)
            door_pose = (bounds.x_min, bounds.center[1], elevation + 1.1, 0.0, 0.0, math.pi)
            goal_pose = (bounds.center[0] - 0.35, bounds.center[1], elevation, 0.0, 0.0, 0.0)
        room_id = f"light_floor_{floor_index}_room_{room_index}"
        rooms.append(
            RoomSpec(
                id=room_id,
                floor_index=floor_index,
                room_type="lightweight_room",
                bounds=bounds,
                side=side,
                door_pose=door_pose,
                goal_pose=goal_pose,
                furniture=_build_lightweight_furniture(room_id, bounds, elevation) if include_furniture else [],
            )
        )
    return rooms


def _build_lightweight_furniture(room_id: str, bounds: Rect2D, elevation: float) -> list[FurnitureSpec]:
    return [
        FurnitureSpec(
            id=f"{room_id}_simple_table",
            kind="simple_table",
            pose=(bounds.center[0], bounds.center[1], elevation + 0.35, 0.0, 0.0, 0.0),
            size=(0.8, 0.5, 0.7),
        )
    ]


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Generate a small, low-link-count competition scene compatible with the original door/elevator control files."
    )
    parser.add_argument("--output-dir", required=True, help="Directory for generated world and metadata.")
    parser.add_argument("--results-dir", help="Directory for referee truth and team result files.")
    parser.add_argument("--seed", type=int, help="Scene seed. If omitted, a random seed is selected and recorded.")
    parser.add_argument("--floor-count", default="1", help="Exact value or min:max range.")
    parser.add_argument("--rooms-per-floor", default="2", help="Exact value or min:max range.")
    parser.add_argument("--width", type=float, default=DEFAULT_WIDTH)
    parser.add_argument("--length", type=float, default=DEFAULT_LENGTH)
    parser.add_argument("--floor-height", type=float, default=DEFAULT_FLOOR_HEIGHT)
    parser.add_argument("--wall-height", type=float, default=DEFAULT_WALL_HEIGHT)
    parser.add_argument("--corridor-width", type=float, default=1.6)
    parser.add_argument("--lobby-depth", type=float, default=4.2)
    parser.add_argument("--danger-count", default="1:2", help="Exact value or min:max range.")
    parser.add_argument("--distractor-count", default="0:2", help="Exact value or min:max range.")
    parser.add_argument("--include-roof", dest="include_roof", action="store_true", default=False)
    parser.add_argument("--no-roof", dest="include_roof", action="store_false")
    parser.add_argument("--include-elevator", dest="include_elevator", action="store_true", default=True)
    parser.add_argument("--no-elevator", dest="include_elevator", action="store_false")
    parser.add_argument("--include-stairs", dest="include_stairs", action="store_true", default=True)
    parser.add_argument("--no-stairs", dest="include_stairs", action="store_false")
    parser.add_argument("--include-furniture", dest="include_furniture", action="store_true", default=False)
    parser.add_argument("--no-furniture", dest="include_furniture", action="store_false")
    parser.add_argument("--robot-x", type=float, default=0.0)
    parser.add_argument("--robot-y", type=float, default=-1.2)
    parser.add_argument("--robot-z", type=float, default=0.6)
    parser.add_argument("--robot-yaw", type=float, default=1.5708)
    return parser


def _sample_count(raw_value: str, rng: random.Random) -> int:
    spec = str(raw_value)
    if ":" in spec:
        min_value, max_value = spec.split(":", 1)
        return rng.randint(int(min_value), int(max_value))
    return int(spec)


def _build_signature(*, footprint: dict[str, float], floors, doors, elevators, metadata: dict[str, Any]) -> str:
    payload = {
        "footprint": footprint,
        "floors": [floor.as_dict() for floor in floors],
        "doors": [door.as_dict() for door in doors],
        "elevators": [elevator.as_dict() for elevator in elevators],
        "metadata": metadata,
    }
    return hashlib.sha256(json.dumps(payload, sort_keys=True).encode("utf-8")).hexdigest()[:16]


def _connectivity_label(include_elevator: bool, include_stairs: bool) -> str:
    if include_elevator and include_stairs:
        return "stairs_and_elevator"
    if include_elevator:
        return "elevator_only"
    if include_stairs:
        return "stairs_only"
    return "single_floor"


def _build_building_config(layout: BuildingLayout, seed: int, world_path: Path, sources, args) -> dict[str, object]:
    return {
        "schema": "lightweight_competition_building_config_v1",
        "seed": seed,
        "model_name": layout.model_name,
        "world_file": str(world_path),
        "num_floors": len(layout.floors),
        "floor_heights": [floor.elevation for floor in layout.floors],
        "building_width": layout.footprint["width"],
        "building_depth": layout.footprint["length"],
        "room_count": sum(len(floor.rooms) for floor in layout.floors),
        "danger_count": sum(1 for source in sources if source.source_kind.is_danger),
        "distractor_count": sum(1 for source in sources if not source.source_kind.is_danger),
        "entrance_pose": list(layout.entrance_pose),
        "target_points": layout.target_points,
        "lightweight_options": {
            "include_roof": args.include_roof,
            "include_elevator": args.include_elevator,
            "include_stairs": args.include_stairs,
            "include_furniture": args.include_furniture,
        },
    }


if __name__ == "__main__":
    raise SystemExit(main())
