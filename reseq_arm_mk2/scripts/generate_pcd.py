import sys

import numpy as np


def create_pcd(filename):
    points = []

    # Updated Grid: Coarser grid (0.1m spacing) to prevent crashes
    x_range = np.arange(-0.4, 0.41, 0.1)  # -0.4m to 0.4m
    y_range = np.arange(-0.4, 0.41, 0.1)  # -0.4m to 0.4m
    z_range = np.arange(0.0, 0.61, 0.1)  # 0.0m to 0.6m

    # Dummy normal
    nx, ny, nz = 1.0, 0.0, 0.0

    for x in x_range:
        for y in y_range:
            for z in z_range:
                # Exclude self-collision zone
                if np.sqrt(x**2 + y**2 + z**2) < 0.15:
                    continue

                points.append(f'{x} {y} {z} {nx} {ny} {nz} 0')

    header = f"""# .PCD v.7 - Point Cloud Data file format
VERSION .7
FIELDS x y z normal_x normal_y normal_z curvature
SIZE 4 4 4 4 4 4 4
TYPE F F F F F F F
COUNT 1 1 1 1 1 1 1
WIDTH {len(points)}
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS {len(points)}
DATA ascii
"""

    with open(filename, 'w') as f:
        f.write(header)
        f.write('\n'.join(points))

    print(f'Generated {len(points)} points in {filename}')


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print('Usage: python3 generate_pcd.py <output_file>')
        sys.exit(1)
    create_pcd(sys.argv[1])
