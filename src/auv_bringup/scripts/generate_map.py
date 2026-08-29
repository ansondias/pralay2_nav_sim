#!/usr/bin/env python3
"""Generate ocean_arena.pgm/.yaml to match auv_gazebo/worlds/ocean_arena.sdf.

Obstacle coordinates below are kept in sync by hand with the <model> poses
in the world file. Re-run this after editing the world's obstacle layout:

    python3 generate_map.py --out ../maps/ocean_arena
"""
import argparse
import math

ARENA_HALF_SIZE = 20.0  # world spans [-20, 20] x [-20, 20]
RESOLUTION = 0.2

FREE = 254
OCCUPIED = 0

# type, center_x, center_y, a, b, yaw_rad  (a/b: box half-extents, or (radius, radius) for a circle)
OBSTACLES = [
    ("box", 8.0, 5.0, 1.5, 1.5, 0.0),      # reef_1
    ("circle", -6.0, 10.0, 2.0, 2.0, 0.0),  # reef_2
    ("box", 0.0, -8.0, 2.0, 1.0, 0.6),      # reef_3
    ("circle", 12.0, -10.0, 1.5, 1.5, 0.0),  # reef_4
    ("box", -12.0, -4.0, 1.0, 3.0, 0.3),    # reef_5
    ("box", 0.0, 19.5, 20.0, 0.5, 0.0),     # wall_north
    ("box", 0.0, -19.5, 20.0, 0.5, 0.0),    # wall_south
    ("box", 19.5, 0.0, 0.5, 20.0, 0.0),     # wall_east
    ("box", -19.5, 0.0, 0.5, 20.0, 0.0),    # wall_west
]


def occupied(wx, wy):
    for kind, cx, cy, a, b, yaw in OBSTACLES:
        dx, dy = wx - cx, wy - cy
        if kind == "circle":
            if dx * dx + dy * dy <= a * a:
                return True
        else:  # box, possibly rotated
            c, s = math.cos(-yaw), math.sin(-yaw)
            lx = c * dx - s * dy
            ly = s * dx + c * dy
            if abs(lx) <= a and abs(ly) <= b:
                return True
    return False


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", default="../maps/ocean_arena")
    args = parser.parse_args()

    size = int(2 * ARENA_HALF_SIZE / RESOLUTION)  # 200

    rows = []
    for row in range(size):
        world_y = ARENA_HALF_SIZE - (row + 0.5) * RESOLUTION
        line = bytearray(size)
        for col in range(size):
            world_x = -ARENA_HALF_SIZE + (col + 0.5) * RESOLUTION
            line[col] = OCCUPIED if occupied(world_x, world_y) else FREE
        rows.append(bytes(line))

    pgm_path = args.out + ".pgm"
    with open(pgm_path, "wb") as f:
        f.write(f"P5\n{size} {size}\n255\n".encode("ascii"))
        for row in rows:
            f.write(row)

    yaml_path = args.out + ".yaml"
    with open(yaml_path, "w") as f:
        f.write(
            "image: ocean_arena.pgm\n"
            f"resolution: {RESOLUTION}\n"
            f"origin: [{-ARENA_HALF_SIZE}, {-ARENA_HALF_SIZE}, 0.0]\n"
            "negate: 0\n"
            "occupied_thresh: 0.65\n"
            "free_thresh: 0.196\n"
        )

    print(f"wrote {pgm_path} and {yaml_path} ({size}x{size} px)")


if __name__ == "__main__":
    main()
