"""Generate rescue_maze.world and its textures.

A dwelling-search maze in the style of the RoboCup Rescue Robot League arena,
sized for Rese.Q: a 1.3 m three-module robot driven by a 2D Nav2 stack that
does not reverse. The corridor is 2 m wide and the floor is flat.

Run it from this directory after changing the layout:  python3 generate_rescue_maze.py
"""

import math

from PIL import Image, ImageDraw, ImageFont
from world_parts import (
    FONT,
    TEXTURES,
    box_visual,
    cylinder,
    tint,
    make_base_textures,
    material,
    overview_camera,
    solid,
    static_model,
    wall_model,
    write_world,
)

# Walls as (x1, y1, x2, y2, finish). Gaps between segments are the doorways.
WALLS = [
    # Outer walls of the 12 x 8 m arena.
    (-2, 4, 10, 4, 'plywood'),
    (-2, -4, 10, -4, 'plywood'),
    (-2, -4, -2, 4, 'plywood'),
    (10, -4, 10, 4, 'plywood'),
    # Central corridor (2 m wide), doors to the north rooms at x 1.2-2.6 and 4.6-6.0.
    (-2, 1, 1.2, 1, 'plywood'),
    (2.6, 1, 4.6, 1, 'plywood'),
    (6.0, 1, 7, 1, 'plywood'),
    # Doors to the south rooms at x 2.0-3.4 and 5.0-6.4.
    (-2, -1, 2.0, -1, 'plywood'),
    (3.4, -1, 5.0, -1, 'plywood'),
    (6.4, -1, 7, -1, 'plywood'),
    # Painted dividers between the rooms, as landmarks.
    (3.5, 1, 3.5, 4, 'blue'),
    (3.5, -4, 3.5, -1, 'yellow'),
    # East rooms open to the east hall through 1.4 m doors.
    (7, 1, 7, 2.2, 'plywood'),
    (7, 3.6, 7, 4, 'plywood'),
    (7, -4, 7, -3.6, 'plywood'),
    (7, -2.2, 7, -1, 'plywood'),
]

# Placards as (texture, x, y, facing yaw), centred 0.6 m high like the league's wall tasks.
SIGNS = [
    ('hazmat_flammable_gas', 3.6, 0.94, -math.pi / 2),
    ('hazmat_corrosive', 4.2, -0.94, math.pi / 2),
    ('hazmat_poison', 5.2, 3.94, -math.pi / 2),
    ('hazmat_oxidizer', -1.94, -2.5, 0.0),
    ('hazmat_explosive', 9.94, 2.0, math.pi),
    ('hazmat_dangerous_when_wet', 4.5, -3.94, math.pi / 2),
    ('hazmat_non_flammable_gas', 0.5, 3.94, -math.pi / 2),
    ('end_sign', 9.94, 0.0, math.pi),
]


def make_textures():
    make_base_textures()

    # Cardboard with a strip of tape.
    noise = tint(Image.effect_noise((256, 256), 18), 0, 0.25)
    red = tint(noise, 160, 1)
    green = tint(noise, 118, 1)
    blue = tint(noise, 74, 1)
    cardboard = Image.merge('RGB', [red, green, blue])
    ImageDraw.Draw(cardboard).rectangle([0, 110, 256, 146], fill=(200, 180, 140))
    cardboard.save(TEXTURES / 'cardboard.jpg', quality=88)

    placard('hazmat_flammable_gas', 'FLAMMABLE\nGAS', '2', (205, 30, 35), 'white')
    placard('hazmat_non_flammable_gas', 'NON-FLAMMABLE\nGAS', '2', (20, 140, 70), 'white')
    placard('hazmat_oxidizer', 'OXIDIZER', '5.1', (240, 200, 30), 'black')
    placard('hazmat_poison', 'POISON', '6', (250, 250, 250), 'black')
    placard('hazmat_explosive', 'EXPLOSIVE', '1.4', (245, 130, 30), 'black')
    placard('hazmat_dangerous_when_wet', 'DANGEROUS\nWHEN WET', '4', (30, 90, 190), 'white')
    placard('hazmat_corrosive', 'CORROSIVE', '8', None, 'white')

    end = Image.new('RGB', (512, 512), (235, 190, 30))
    draw = ImageDraw.Draw(end)
    for offset in range(-512, 512, 64):
        draw.polygon([(offset, 0), (offset + 32, 0), (offset + 544, 512), (offset + 512, 512)],
                     fill=(30, 30, 30))
    draw.rectangle([60, 170, 452, 342], fill=(250, 250, 250))
    font = ImageFont.truetype(FONT, 120)
    draw.text((256, 256), 'END', fill=(20, 20, 20), anchor='mm', font=font)
    end.save(TEXTURES / 'end_sign.png')


def placard(name, label, number, color, text_color):
    """A diamond hazmat-style placard on a white board. color None draws the corrosive split."""
    size = 512
    board = Image.new('RGB', (size, size), (250, 250, 250))
    draw = ImageDraw.Draw(board)
    diamond = [(size / 2, 16), (size - 16, size / 2), (size / 2, size - 16), (16, size / 2)]
    if color is None:
        draw.polygon(diamond, fill=(250, 250, 250))
        lower_half = [(16, size / 2), (size - 16, size / 2), (size / 2, size - 16)]
        draw.polygon(lower_half, fill=(20, 20, 20))
    else:
        draw.polygon(diamond, fill=color)
    draw.polygon(diamond, outline=(20, 20, 20), width=6)
    inner = [(size / 2, 40), (size - 40, size / 2), (size / 2, size - 40), (40, size / 2)]
    # The corrosive placard has a fixed black and white style.
    if color is None:
        outline_color = (20, 20, 20)
        label_color = (20, 20, 20)
        number_color = 'white'
    else:
        outline_color = text_color
        label_color = text_color
        number_color = text_color
    draw.polygon(inner, outline=outline_color, width=3)
    # Smaller letters for long or two-line labels.
    font_size = 54
    if '\n' in label or len(label) > 9:
        font_size = 46
    font = ImageFont.truetype(FONT, font_size)
    draw.multiline_text((size / 2, size / 2 - 20), label, fill=label_color, anchor='mm',
                        align='center', font=font)
    draw.text((size / 2, size - 95), number, fill=number_color, anchor='mm',
              font=ImageFont.truetype(FONT, 64))
    board.save(TEXTURES / f'{name}.png')


def floor_model():
    body = ''
    for i, x in enumerate(range(-2, 10, 2)):
        for j, y in enumerate(range(-4, 4, 2)):
            body += box_visual(f'tile_{i}_{j}', f'{x + 1} {y + 1} 0.001 0 0 0', '2 2 0.002',
                               material('osb_floor.jpg'))
    body += box_visual('start_zone', '-0.4 0 0.003 0 0 0', '1.6 1.0 0.002',
                       material('start_zone.png'))
    body += box_visual('end_zone', '8.5 0 0.003 0 0 0', '1.4 1.4 0.002', material('end_zone.png'))
    return static_model('arena_floor', '0 0 0 0 0 0', body)


def sign_model(index, texture, x, y, yaw):
    size = '0.008 0.32 0.32'
    if texture == 'end_sign':
        size = '0.008 0.6 0.6'
    body = box_visual('board', '0 0 0 0 0 0', size, material(f'{texture}.png'))
    return static_model(f'sign_{index}_{texture}', f'{x} {y} 0.6 0 0 {yaw:.4f}', body)


def objects():
    skin = '0.85 0.65 0.5 1'
    shirt = '0.95 0.45 0.6 1'
    trousers = '0.2 0.35 0.7 1'
    # Victim: a doll-sized figure lying on its back (about 0.8 m long).
    victim = (
        solid('torso', '0 0 0.07 0 0 0', '<box><size>0.32 0.2 0.12</size></box>', shirt)
        + solid('head', '0.24 0 0.08 0 0 0', '<sphere><radius>0.08</radius></sphere>', skin)
        + solid('leg_left', '-0.3 0.05 0.04 0 1.5708 0', cylinder(0.04, 0.3), trousers)
        + solid('leg_right', '-0.3 -0.05 0.04 0 1.5708 0', cylinder(0.04, 0.3), trousers)
        + solid('arm_left', '0.02 0.15 0.04 0 1.5708 0.3', cylinder(0.03, 0.26), skin)
        + solid('arm_right', '0.02 -0.15 0.04 0 1.5708 -0.3', cylinder(0.03, 0.26), skin)
    )
    backpack = (
        solid('bag', '0 0 0.22 0 0 0', '<box><size>0.2 0.34 0.44</size></box>', '0.15 0.3 0.75 1')
        + solid('pocket', '0.12 0 0.15 0 0 0', '<box><size>0.06 0.26 0.2</size></box>',
                '0.1 0.2 0.5 1')
    )
    hard_hat = (
        solid('dome', '0 0 0 0 0 0', '<sphere><radius>0.13</radius></sphere>', '0.95 0.8 0.1 1')
        + solid('brim', '0.03 0 0.005 0 0 0', cylinder(0.17, 0.01), '0.95 0.8 0.1 1')
    )
    extinguisher = (
        solid('body', '0 0 0.25 0 0 0', cylinder(0.08, 0.5), '0.8 0.08 0.08 1')
        + solid('head', '0 0 0.54 0 0 0', cylinder(0.03, 0.08), '0.2 0.2 0.2 1')
        + solid('hose', '0.07 0 0.45 0 0.6 0', cylinder(0.012, 0.2), '0.1 0.1 0.1 1')
    )
    propane = (
        solid('tank', '0 0 0.25 0 0 0', cylinder(0.15, 0.4), '0.92 0.92 0.92 1')
        + solid('top', '0 0 0.45 0 0 0', '<sphere><radius>0.15</radius></sphere>',
                '0.92 0.92 0.92 1')
        + solid('valve', '0 0 0.62 0 0 0', cylinder(0.04, 0.08), '0.75 0.65 0.2 1')
    )
    barrel = solid('barrel', '0 0 0.44 0 0 0', cylinder(0.29, 0.88), '0.15 0.35 0.65 1')
    # Cardboard boxes as (pose, size).
    boxes = [
        ('0 0 0.2 0 0 0.3', '0.5 0.4 0.4'),
        ('0.45 0.3 0.15 0 0 -0.5', '0.4 0.4 0.3'),
        ('-0.2 0.45 0.25 0 0 0.9', '0.35 0.6 0.5'),
        ('0.2 -0.35 0.12 0.2 0 0.1', '0.6 0.3 0.24'),
    ]
    debris = ''
    for i, (pose, size) in enumerate(boxes):
        geometry = f'<box><size>{size}</size></box>'
        debris += (f'<collision name="box_{i}_collision"><pose>{pose}</pose>'
                   f'<geometry>{geometry}</geometry></collision>')
        debris += box_visual(f'box_{i}', pose, size, material('cardboard.jpg'))
    return [
        static_model('victim_doll', '5.2 3.0 0 0 0 0.4', victim),
        static_model('backpack', '-0.5 3.2 0 0 0 0.6', backpack),
        static_model('hard_hat', '0.6 -3.0 0 0 0 0', hard_hat),
        static_model('fire_extinguisher', '9.5 3.4 0 0 0 0', extinguisher),
        static_model('propane_tank', '5.9 -3.3 0 0 0 0', propane),
        static_model('barrel_1', '-1.4 3.4 0 0 0 0', barrel),
        static_model('barrel_2', '-0.8 3.5 0 0 0 0', barrel),
        static_model('debris', '8.6 -2.7 0 0 0 0', debris),
    ]


def main():
    make_textures()
    models = [floor_model()]
    for i, wall in enumerate(WALLS):
        models.append(wall_model(i, *wall))
    for i, sign in enumerate(SIGNS):
        models.append(sign_model(i, *sign))
    models += objects()
    # Above the arena centre; the 12 m length runs across the image width.
    models.append(overview_camera(4, 0, 9.5))
    write_world('rescue_maze', models, 'generate_rescue_maze.py')


if __name__ == '__main__':
    main()
