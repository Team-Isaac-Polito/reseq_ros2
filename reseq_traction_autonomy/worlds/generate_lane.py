"""Generate lane.world: one RoboCup Rescue mobility test lane.

Sizes follow the league's lane overview (rules 2026D, page 6):
a 2.4 m wide, 4.8 m long lane with a 1.2 m deep end bay at each end.
The robot drives down range from the start bay to the end bay and back.
A QR code on the end wall marks the end zone; the start has none.

The robot spawns at the origin facing +x, so world and SLAM map frames match.
This first lane is flat. Terrain (K-rails, 15 degree ramps) comes later.

Run it from this directory after changing the layout:  python3 generate_lane.py
"""

import cv2
from PIL import Image
from world_parts import (
    TEXTURES,
    WALL_THICKNESS,
    box_visual,
    make_base_textures,
    material,
    overview_camera,
    static_model,
    wall_model,
    write_world,
    zone_marking,
)

LENGTH = 4.8
WIDTH = 2.4
BAY_DEPTH = 1.2
# The back wall sits just behind the robot's tail (about 1.1 m behind base_link).
START_X = -1.15
END_X = START_X + LENGTH
HALF = WIDTH / 2

QR_TEXT = 'RESEQ END ZONE'
QR_SIZE = 0.24  # metres
# Centre above the floor. The arm camera is about 0.27 m up and tilts
# down 11 degrees, so a higher code is cut off when the robot stops close.
QR_HEIGHT = 0.22


def make_textures():
    make_base_textures()
    # Bay markings are twice as wide (y) as deep (x).
    zone_marking('start_bay', (40, 170, 70), 'START', size=(512, 1024))
    zone_marking('end_bay', (235, 190, 30), 'END', size=(512, 1024))

    # OpenCV draws one pixel per QR module; scale it up and add a white margin.
    modules = cv2.QRCodeEncoder.create().encode(QR_TEXT)
    qr = Image.fromarray(modules).resize((400, 400), Image.NEAREST)
    board = Image.new('L', (512, 512), 255)
    board.paste(qr, (56, 56))
    board.save(TEXTURES / 'qr_end.png')


def walls():
    corners = [(START_X, -HALF), (END_X, -HALF), (END_X, HALF), (START_X, HALF)]
    return [
        wall_model(i, *corners[i], *corners[(i + 1) % 4])
        for i in range(4)
    ]


def floor_model():
    body = ''
    # 1.2 m OSB tiles over the whole lane.
    for i in range(4):
        for j in range(2):
            x = START_X + 0.6 + 1.2 * i
            y = -HALF + 0.6 + 1.2 * j
            body += box_visual(f'tile_{i}_{j}', f'{x:.2f} {y:.2f} 0.001 0 0 0',
                               '1.2 1.2 0.002', material('osb_floor.jpg'))
    bay = f'{BAY_DEPTH} {WIDTH} 0.002'
    start = START_X + BAY_DEPTH / 2
    end = END_X - BAY_DEPTH / 2
    body += box_visual('start_bay', f'{start:.2f} 0 0.003 0 0 0', bay, material('start_bay.png'))
    body += box_visual('end_bay', f'{end:.2f} 0 0.003 0 0 0', bay, material('end_bay.png'))
    return static_model('lane_floor', '0 0 0 0 0 0', body)


def qr_model():
    # On the inner face of the end wall, facing back down the lane.
    x = END_X - WALL_THICKNESS / 2 - 0.005
    size = f'0.008 {QR_SIZE} {QR_SIZE}'
    body = box_visual('board', '0 0 0 0 0 0', size, material('qr_end.png'))
    return static_model('end_qr', f'{x:.3f} 0 {QR_HEIGHT} 0 0 3.1416', body)


def main():
    make_textures()
    models = [floor_model(), qr_model(), *walls()]
    # Above the lane centre; the lane length runs across the image width.
    models.append(overview_camera((START_X + END_X) / 2, 0, 4.2))
    write_world('lane', models, 'generate_lane.py')


if __name__ == '__main__':
    main()
