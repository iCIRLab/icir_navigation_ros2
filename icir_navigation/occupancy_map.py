import math
import os

__all__ = ['write_box_map', 'cache_dir']

FREE = 254
OCCUPIED = 0


def cache_dir():
    root = os.environ.get('ROS_HOME', os.path.join(os.path.expanduser('~'), '.ros'))
    path = os.path.join(root, 'icir_navigation')
    os.makedirs(path, exist_ok=True)
    return path


def write_box_map(obstacles, out_prefix, resolution=0.05, padding=1.0):
    if not obstacles:
        raise ValueError('no obstacles: cannot build a map (need obs_* boxes in the scene)')

    xmin = min(cx - hx for cx, cy, hx, hy in obstacles) - padding
    xmax = max(cx + hx for cx, cy, hx, hy in obstacles) + padding
    ymin = min(cy - hy for cx, cy, hx, hy in obstacles) - padding
    ymax = max(cy + hy for cx, cy, hx, hy in obstacles) + padding

    width = int(math.ceil((xmax - xmin) / resolution))
    height = int(math.ceil((ymax - ymin) / resolution))

    grid = [bytearray([FREE]) * width for _ in range(height)]
    for cx, cy, hx, hy in obstacles:
        c0 = max(0, int(math.floor((cx - hx - xmin) / resolution)))
        c1 = min(width, max(c0 + 1, int(math.ceil((cx + hx - xmin) / resolution))))
        r0 = max(0, int(math.floor((cy - hy - ymin) / resolution)))
        r1 = min(height, max(r0 + 1, int(math.ceil((cy + hy - ymin) / resolution))))
        for r in range(r0, r1):
            grid[r][c0:c1] = bytes([OCCUPIED]) * (c1 - c0)

    out_dir = os.path.dirname(out_prefix)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)

    pgm = out_prefix + '.pgm'
    with open(pgm, 'wb') as f:
        f.write(b'P5\n%d %d\n255\n' % (width, height))
        for row in reversed(grid):
            f.write(row)

    yaml_path = out_prefix + '.yaml'
    with open(yaml_path, 'w') as f:
        f.write('image: %s\n' % os.path.basename(pgm))
        f.write('mode: trinary\n')
        f.write('resolution: %g\n' % resolution)
        f.write('origin: [%.4f, %.4f, 0.0]\n' % (xmin, ymin))
        f.write('negate: 0\n')
        f.write('occupied_thresh: 0.65\n')
        f.write('free_thresh: 0.25\n')

    print('[occupancy_map] %d box(es) -> %s (%dx%d @ %g m, origin %.2f, %.2f)'
          % (len(obstacles), yaml_path, width, height, resolution, xmin, ymin))
    return yaml_path
