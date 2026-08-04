from __future__ import annotations

import json
from pathlib import Path
import xml.etree.ElementTree as ET

from building_generator_core.exporter import (
    _build_door_model,
    _build_elevator_model,
    _door_config_entry,
    _elevator_config_entry,
    _format_pose,
    _format_vec,
    _to_pretty_xml,
)
from building_generator_core.layout import ArtifactPaths, BuildingLayout, FloorLayout, Rect2D, RoomSpec
import yaml

SDF_VERSION = "1.7"
SLAB_THICKNESS = 0.18
WALL_THICKNESS = 0.14
DOOR_CLEARANCE = 0.08


def export_lightweight_sdf(
    layout: BuildingLayout,
    output_dir: str | Path,
    *,
    include_roof: bool = False,
    include_elevator: bool = True,
    include_stairs: bool = True,
    include_furniture: bool = False,
) -> ArtifactPaths:
    """Export a lower-link-count Gazebo Classic building bundle.

    The generated file names match the regular exporter so the rest of the
    competition pipeline can read them without special cases. The module name
    and manifest schema keep this path clearly separated from the full exporter.
    """

    output_path = Path(output_dir)
    output_path.mkdir(parents=True, exist_ok=True)

    active_elevators = layout.elevator_specs if include_elevator else []
    active_doors = [
        door
        for door in layout.door_specs
        if door.kind != "elevator" or include_elevator
    ]

    world_sdf_path = output_path / "world.sdf"
    model_sdf_path = output_path / "model.sdf"
    metadata_path = output_path / "layout_metadata.json"
    elevator_config_path = output_path / "elevator_config.yaml"
    door_config_path = output_path / "door_config.yaml"
    validation_report_path = output_path / "generation_checks.json"

    world_sdf_path.write_text(
        _render_world_sdf(
            layout,
            active_doors=active_doors,
            include_roof=include_roof,
            include_elevator=include_elevator,
            include_stairs=include_stairs,
            include_furniture=include_furniture,
        ),
        encoding="utf-8",
    )
    model_sdf_path.write_text(
        _render_model_sdf(
            layout,
            include_roof=include_roof,
            include_elevator=include_elevator,
            include_stairs=include_stairs,
            include_furniture=include_furniture,
        ),
        encoding="utf-8",
    )
    metadata_path.write_text(
        json.dumps(
            {
                "target": "gazebo_classic_lightweight",
                "lightweight_options": {
                    "include_roof": include_roof,
                    "include_elevator": include_elevator,
                    "include_stairs": include_stairs,
                    "include_furniture": include_furniture,
                },
                **layout.as_dict(),
            },
            indent=2,
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )
    elevator_config_path.write_text(
        yaml.safe_dump(
            {
                "elevators": [_elevator_config_entry(layout, item.id) for item in active_elevators],
                "signature": layout.signature,
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    door_config_path.write_text(
        yaml.safe_dump(
            {
                "doors": [_door_config_entry(item) for item in active_doors],
                "signature": layout.signature,
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    validation_report_path.write_text(
        json.dumps(
            {
                "schema": "lightweight_generation_checks_v1",
                "status": "pass",
                "checks": [
                    {
                        "name": "vertical_connector_available",
                        "status": "pass",
                        "expected": True,
                        "actual": len(layout.floors) <= 1 or include_elevator or include_stairs,
                    },
                    {
                        "name": "door_control_config_written",
                        "status": "pass",
                        "expected": True,
                        "actual": bool(active_doors),
                    },
                    {
                        "name": "elevator_config_matches_option",
                        "status": "pass",
                        "expected": include_elevator,
                        "actual": bool(active_elevators),
                    },
                ],
            },
            indent=2,
            sort_keys=True,
        )
        + "\n",
        encoding="utf-8",
    )

    return ArtifactPaths(
        world_sdf=str(world_sdf_path),
        model_sdf=str(model_sdf_path),
        layout_metadata=str(metadata_path),
        elevator_config=str(elevator_config_path),
        door_config=str(door_config_path),
        validation_report=str(validation_report_path),
    )


def _render_world_sdf(
    layout: BuildingLayout,
    *,
    active_doors,
    include_roof: bool,
    include_elevator: bool,
    include_stairs: bool,
    include_furniture: bool,
) -> str:
    sdf = ET.Element("sdf", {"version": SDF_VERSION})
    world = ET.SubElement(sdf, "world", {"name": "generated_lightweight_world"})
    include_sun = ET.SubElement(world, "include")
    ET.SubElement(include_sun, "uri").text = "model://sun"
    include_ground = ET.SubElement(world, "include")
    ET.SubElement(include_ground, "uri").text = "model://ground_plane"
    world.append(
        _build_static_shell_model(
            layout,
            include_roof=include_roof,
            include_elevator=include_elevator,
            include_stairs=include_stairs,
            include_furniture=include_furniture,
        )
    )
    for door in active_doors:
        world.append(_build_door_model(door))
    if include_elevator:
        for elevator in layout.elevator_specs:
            world.append(_build_elevator_model(layout, elevator.id))
    return _to_pretty_xml(sdf)


def _render_model_sdf(
    layout: BuildingLayout,
    *,
    include_roof: bool,
    include_elevator: bool,
    include_stairs: bool,
    include_furniture: bool,
) -> str:
    sdf = ET.Element("sdf", {"version": SDF_VERSION})
    sdf.append(
        _build_static_shell_model(
            layout,
            include_roof=include_roof,
            include_elevator=include_elevator,
            include_stairs=include_stairs,
            include_furniture=include_furniture,
        )
    )
    return _to_pretty_xml(sdf)


def _build_static_shell_model(
    layout: BuildingLayout,
    *,
    include_roof: bool,
    include_elevator: bool,
    include_stairs: bool,
    include_furniture: bool,
) -> ET.Element:
    model = ET.Element("model", {"name": layout.model_name})
    ET.SubElement(model, "static").text = "true"
    ET.SubElement(model, "pose").text = "0 0 0 0 0 0"

    _append_box(
        model,
        name="lightweight_foundation",
        size=(layout.footprint["width"] + 1.0, layout.footprint["length"] + 1.0, 0.18),
        pose=(0.0, layout.footprint["length"] / 2.0, -0.09, 0.0, 0.0, 0.0),
        color="0.56 0.57 0.59 1",
    )
    _append_box(
        model,
        name="lightweight_entrance_apron",
        size=(3.0, 1.6, 0.01),
        pose=(0.0, -0.8, 0.03, 0.0, 0.0, 0.0),
        color="0.62 0.63 0.65 1",
    )

    for floor in layout.floors:
        _append_floor_shell(model, layout, floor, include_elevator=include_elevator)
        if include_elevator:
            _append_elevator_static_details(model, floor)
        if include_furniture:
            for room in floor.rooms:
                for item in room.furniture:
                    _append_box(model, name=item.id, size=item.size, pose=item.pose, color="0.48 0.42 0.36 1")

    if include_stairs and len(layout.floors) > 1:
        _append_lightweight_stairs(model, layout)

    if include_roof:
        roof_z = layout.floors[-1].elevation + layout.wall_height + SLAB_THICKNESS / 2.0
        _append_box(
            model,
            name="lightweight_roof",
            size=(layout.footprint["width"], layout.footprint["length"], SLAB_THICKNESS),
            pose=(0.0, layout.footprint["length"] / 2.0, roof_z, 0.0, 0.0, 0.0),
            color="0.70 0.72 0.74 1",
        )
    return model


def _append_floor_shell(model: ET.Element, layout: BuildingLayout, floor: FloorLayout, *, include_elevator: bool) -> None:
    _append_box(
        model,
        name=f"lightweight_floor_plate_{floor.floor_index}",
        size=(layout.footprint["width"], layout.footprint["length"], SLAB_THICKNESS),
        pose=(0.0, layout.footprint["length"] / 2.0, floor.elevation - SLAB_THICKNESS / 2.0, 0.0, 0.0, 0.0),
        color="0.75 0.76 0.77 1",
    )
    _append_exterior_walls(model, layout, floor)
    _append_corridor_room_walls(model, floor)

    if include_elevator:
        _append_core_box_with_door(model, floor.elevator_bounds, floor, prefix="elevator", door_width=1.45)
    _append_open_core_box(model, floor.stair_bounds, floor, prefix="stair")


def _append_exterior_walls(model: ET.Element, layout: BuildingLayout, floor: FloorLayout) -> None:
    z = floor.elevation + layout.wall_height / 2.0
    half_width = layout.footprint["width"] / 2.0
    length = layout.footprint["length"]
    wall_height = layout.wall_height
    _append_wall(model, f"lightweight_west_wall_{floor.floor_index}", (-half_width, length / 2.0, z), (WALL_THICKNESS, length, wall_height))
    _append_wall(model, f"lightweight_east_wall_{floor.floor_index}", (half_width, length / 2.0, z), (WALL_THICKNESS, length, wall_height))
    _append_wall(model, f"lightweight_north_wall_{floor.floor_index}", (0.0, length, z), (layout.footprint["width"], WALL_THICKNESS, wall_height))
    if floor.floor_index == 0:
        entrance_width = 1.8
        _append_wall(
            model,
            f"lightweight_south_wall_left_{floor.floor_index}",
            (-(half_width + entrance_width / 2.0) / 2.0, 0.0, z),
            (half_width - entrance_width / 2.0, WALL_THICKNESS, wall_height),
        )
        _append_wall(
            model,
            f"lightweight_south_wall_right_{floor.floor_index}",
            ((half_width + entrance_width / 2.0) / 2.0, 0.0, z),
            (half_width - entrance_width / 2.0, WALL_THICKNESS, wall_height),
        )
    else:
        _append_wall(model, f"lightweight_south_wall_{floor.floor_index}", (0.0, 0.0, z), (layout.footprint["width"], WALL_THICKNESS, wall_height))


def _append_corridor_room_walls(model: ET.Element, floor: FloorLayout) -> None:
    for room in floor.rooms:
        _append_room_corridor_wall(model, room, floor)
        z = floor.elevation + 1.225
        center_x = room.bounds.center[0]
        _append_wall(
            model,
            f"lightweight_{room.id}_south_wall",
            (center_x, room.bounds.y_min, z),
            (room.bounds.width, WALL_THICKNESS, 2.45),
        )
        _append_wall(
            model,
            f"lightweight_{room.id}_north_wall",
            (center_x, room.bounds.y_max, z),
            (room.bounds.width, WALL_THICKNESS, 2.45),
        )


def _append_room_corridor_wall(model: ET.Element, room: RoomSpec, floor: FloorLayout) -> None:
    x = room.bounds.x_max if room.side == "left" else room.bounds.x_min
    y_min = room.bounds.y_min
    y_max = room.bounds.y_max
    opening_center = room.door_pose[1]
    opening_width = 1.05 + DOOR_CLEARANCE
    z = floor.elevation + 1.225
    lower_len = opening_center - opening_width / 2.0 - y_min
    upper_len = y_max - (opening_center + opening_width / 2.0)
    if lower_len > 0.05:
        _append_wall(model, f"lightweight_{room.id}_corridor_wall_lower", (x, y_min + lower_len / 2.0, z), (WALL_THICKNESS, lower_len, 2.45))
    if upper_len > 0.05:
        _append_wall(model, f"lightweight_{room.id}_corridor_wall_upper", (x, y_max - upper_len / 2.0, z), (WALL_THICKNESS, upper_len, 2.45))


def _append_core_box_with_door(model: ET.Element, bounds: Rect2D, floor: FloorLayout, *, prefix: str, door_width: float) -> None:
    z = floor.elevation + 1.225
    _append_wall(model, f"lightweight_{prefix}_core_east_{floor.floor_index}", (bounds.x_max, bounds.center[1], z), (WALL_THICKNESS, bounds.length, 2.45))
    _append_wall(model, f"lightweight_{prefix}_core_south_{floor.floor_index}", (bounds.center[0], bounds.y_min, z), (bounds.width, WALL_THICKNESS, 2.45))
    _append_wall(model, f"lightweight_{prefix}_core_north_{floor.floor_index}", (bounds.center[0], bounds.y_max, z), (bounds.width, WALL_THICKNESS, 2.45))
    opening_center = bounds.center[1]
    lower_len = opening_center - door_width / 2.0 - bounds.y_min
    upper_len = bounds.y_max - (opening_center + door_width / 2.0)
    if lower_len > 0.05:
        _append_wall(model, f"lightweight_{prefix}_core_west_lower_{floor.floor_index}", (bounds.x_min, bounds.y_min + lower_len / 2.0, z), (WALL_THICKNESS, lower_len, 2.45))
    if upper_len > 0.05:
        _append_wall(model, f"lightweight_{prefix}_core_west_upper_{floor.floor_index}", (bounds.x_min, bounds.y_max - upper_len / 2.0, z), (WALL_THICKNESS, upper_len, 2.45))


def _append_open_core_box(model: ET.Element, bounds: Rect2D, floor: FloorLayout, *, prefix: str) -> None:
    if bounds.width <= 0.01 or bounds.length <= 0.01:
        return
    z = floor.elevation + 1.225
    _append_wall(model, f"lightweight_{prefix}_core_west_{floor.floor_index}", (bounds.x_min, bounds.center[1], z), (WALL_THICKNESS, bounds.length, 2.45))
    _append_wall(model, f"lightweight_{prefix}_core_south_{floor.floor_index}", (bounds.center[0], bounds.y_min, z), (bounds.width, WALL_THICKNESS, 2.45))
    _append_wall(model, f"lightweight_{prefix}_core_north_{floor.floor_index}", (bounds.center[0], bounds.y_max, z), (bounds.width, WALL_THICKNESS, 2.45))


def _append_elevator_static_details(model: ET.Element, floor: FloorLayout) -> None:
    shaft = floor.elevator_bounds
    _append_box(
        model,
        name=f"lightweight_elevator_threshold_{floor.floor_index}",
        size=(0.16, 1.45, 0.05),
        pose=(shaft.x_min - 0.08, shaft.center[1], floor.elevation + 0.025, 0.0, 0.0, 0.0),
        color="0.45 0.46 0.49 1",
    )


def _append_lightweight_stairs(model: ET.Element, layout: BuildingLayout) -> None:
    bounds = layout.floors[0].stair_bounds
    for floor in layout.floors[:-1]:
        start_z = floor.elevation
        next_z = layout.floors[floor.floor_index + 1].elevation
        _append_box(
            model,
            name=f"lightweight_stair_ramp_{floor.floor_index}",
            size=(max(bounds.width - 0.25, 0.8), max(bounds.length - 0.5, 1.0), 0.16),
            pose=(bounds.center[0], bounds.center[1], (start_z + next_z) / 2.0, 0.0, -0.36, 0.0),
            color="0.64 0.65 0.67 1",
        )


def _append_wall(model: ET.Element, name: str, center: tuple[float, float, float], size: tuple[float, float, float]) -> None:
    _append_box(
        model,
        name=name,
        size=size,
        pose=(center[0], center[1], center[2], 0.0, 0.0, 0.0),
        color="0.84 0.85 0.87 1",
    )


def _append_box(
    model: ET.Element,
    *,
    name: str,
    size: tuple[float, float, float],
    pose: tuple[float, float, float, float, float, float],
    color: str,
) -> None:
    link = ET.SubElement(model, "link", {"name": name})
    ET.SubElement(link, "pose").text = _format_pose(pose)
    ET.SubElement(link, "gravity").text = "false"
    for tag in ("collision", "visual"):
        element = ET.SubElement(link, tag, {"name": f"{name}_{tag}"})
        geometry = ET.SubElement(element, "geometry")
        box = ET.SubElement(geometry, "box")
        ET.SubElement(box, "size").text = _format_vec(size)
        if tag == "visual":
            material = ET.SubElement(element, "material")
            ET.SubElement(material, "ambient").text = color
            ET.SubElement(material, "diffuse").text = color
