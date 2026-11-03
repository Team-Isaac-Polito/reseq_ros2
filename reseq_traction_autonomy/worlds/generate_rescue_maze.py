"""Generate rescue_maze.world and its textures.

A dwelling-search maze in the style of the RoboCup Rescue Robot League arena,
sized for Rese.Q: a 1.3 m tracked robot driven by a 2D Nav2 stack that does
not reverse. Passages are wider than the league's 1.2 m lanes and the floor is
flat; terrain (K-rails, ramps, stepfields) is left for a later world.

Run it from this directory after changing the layout:  python3 generate_rescue_maze.py
"""

import math
import random
from pathlib import Path

from PIL import Image, ImageDraw, ImageFilter, ImageFont

HERE = Path(__file__).resolve().parent
TEXTURES = HERE / 'textures'
FONT = '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'
WALL_HEIGHT = 1.2
WALL_THICKNESS = 0.1
PANEL_WIDTH = 1.22  # one plywood sheet

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

PAINT = {'blue': '0.25 0.45 0.75 1', 'yellow': '0.9 0.75 0.2 1'}

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
    TEXTURES.mkdir(exist_ok=True)
    rng = random.Random(7)

    # Plywood: warm base with streaky grain from stretched noise.
    grain = Image.effect_noise((32, 512), 40).resize((512, 512))
    grain = grain.filter(ImageFilter.GaussianBlur(2))
    plywood = Image.merge('RGB', [
        grain.point(lambda v: int(150 + v * 0.35)),
        grain.point(lambda v: int(112 + v * 0.30)),
        grain.point(lambda v: int(70 + v * 0.20)),
    ])
    plywood.save(TEXTURES / 'plywood.jpg', quality=88)

    # OSB floor: overlapping wood flakes in several tones.
    osb = Image.new('RGB', (512, 512), (176, 140, 92))
    draw = ImageDraw.Draw(osb)
    for _ in range(900):
        x, y = rng.randint(-20, 512), rng.randint(-20, 512)
        w, h = rng.randint(20, 70), rng.randint(6, 18)
        tone = rng.randint(-35, 35)
        color = (176 + tone, 140 + tone, 92 + tone // 2)
        if rng.random() < 0.5:
            draw.rectangle([x, y, x + w, y + h], fill=color)
        else:
            draw.rectangle([x, y, x + h, y + w], fill=color)
    osb.filter(ImageFilter.GaussianBlur(0.8)).save(TEXTURES / 'osb_floor.jpg', quality=88)

    # Cardboard with a strip of tape.
    cardboard = Image.effect_noise((256, 256), 18).point(lambda v: int(v * 0.25))
    cardboard = Image.merge('RGB', [
        cardboard.point(lambda v: 160 + v), cardboard.point(lambda v: 118 + v),
        cardboard.point(lambda v: 74 + v),
    ])
    ImageDraw.Draw(cardboard).rectangle([0, 110, 256, 146], fill=(200, 180, 140))
    cardboard.save(TEXTURES / 'cardboard.jpg', quality=88)

    placard('hazmat_flammable_gas', 'FLAMMABLE\nGAS', '2', (205, 30, 35), 'white')
    placard('hazmat_non_flammable_gas', 'NON-FLAMMABLE\nGAS', '2', (20, 140, 70), 'white')
    placard('hazmat_oxidizer', 'OXIDIZER', '5.1', (240, 200, 30), 'black')
    placard('hazmat_poison', 'POISON', '6', (250, 250, 250), 'black')
    placard('hazmat_explosive', 'EXPLOSIVE', '1.4', (245, 130, 30), 'black')
    placard('hazmat_dangerous_when_wet', 'DANGEROUS\nWHEN WET', '4', (30, 90, 190), 'white')
    placard('hazmat_corrosive', 'CORROSIVE', '8', None, 'white')
    zone_marking('start_zone', (40, 170, 70), 'START')
    zone_marking('end_zone', (235, 190, 30), 'END')

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
    draw.polygon(inner, outline=text_color if color is not None else (20, 20, 20), width=3)
    font = ImageFont.truetype(FONT, 46 if '\n' in label or len(label) > 9 else 54)
    label_color = (20, 20, 20) if color is None else text_color
    draw.multiline_text((size / 2, size / 2 - 20), label, fill=label_color, anchor='mm',
                        align='center', font=font)
    number_color = 'white' if color is None else text_color
    draw.text((size / 2, size - 95), number, fill=number_color, anchor='mm',
              font=ImageFont.truetype(FONT, 64))
    board.save(TEXTURES / f'{name}.png')


def zone_marking(name, color, text):
    mark = Image.new('RGB', (512, 512), (176, 140, 92))
    draw = ImageDraw.Draw(mark)
    draw.rectangle([8, 8, 504, 504], outline=color, width=36)
    draw.text((256, 256), text, fill=color, anchor='mm', font=ImageFont.truetype(FONT, 110))
    mark.save(TEXTURES / f'{name}.png')


def material(texture=None, color='0.8 0.8 0.8 1'):
    if texture is None:
        return (f'<material><ambient>{color}</ambient><diffuse>{color}</diffuse>'
                '<specular>0.1 0.1 0.1 1</specular></material>')
    return ('<material><diffuse>1 1 1 1</diffuse><specular>0.1 0.1 0.1 1</specular>'
            f'<pbr><metal><albedo_map>textures/{texture}</albedo_map>'
            '<roughness>0.9</roughness><metalness>0</metalness></metal></pbr></material>')


def box_visual(name, pose, size, mat):
    return (f'<visual name="{name}"><pose>{pose}</pose><geometry><box><size>{size}</size></box>'
            f'</geometry>{mat}</visual>')


def static_model(name, pose, body):
    return (f'<model name="{name}"><static>true</static><pose>{pose}</pose>'
            f'<link name="link">{body}</link></model>')


def wall_model(index, x1, y1, x2, y2, finish):
    length = math.hypot(x2 - x1, y2 - y1)
    yaw = math.atan2(y2 - y1, x2 - x1)
    cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
    z = WALL_HEIGHT / 2
    body = (f'<collision name="collision"><geometry><box><size>{length:.3f} {WALL_THICKNESS} '
            f'{WALL_HEIGHT}</size></box></geometry></collision>')
    # One visual per plywood sheet so the grain is not stretched along long walls.
    panels = max(1, math.ceil(length / PANEL_WIDTH))
    width = length / panels
    mat = material('plywood.jpg') if finish == 'plywood' else material(color=PAINT[finish])
    for i in range(panels):
        offset = -length / 2 + width * (i + 0.5)
        body += box_visual(f'panel_{i}', f'{offset:.3f} 0 0 0 0 0',
                           f'{width - 0.01:.3f} {WALL_THICKNESS} {WALL_HEIGHT}', mat)
    return static_model(f'wall_{index}', f'{cx:.3f} {cy:.3f} {z} 0 0 {yaw:.4f}', body)


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
    size = '0.008 0.32 0.32' if texture != 'end_sign' else '0.008 0.6 0.6'
    body = box_visual('board', '0 0 0 0 0 0', size, material(f'{texture}.png'))
    return static_model(f'sign_{index}_{texture}', f'{x} {y} 0.6 0 0 {yaw:.4f}', body)


def solid(name, pose, geometry, color):
    """A static object whose collision matches its visual."""
    return (f'<collision name="{name}_collision"><pose>{pose}</pose><geometry>{geometry}'
            f'</geometry></collision><visual name="{name}"><pose>{pose}</pose><geometry>'
            f'{geometry}</geometry>{material(color=color)}</visual>')


def cylinder(radius, length):
    return f'<cylinder><radius>{radius}</radius><length>{length}</length></cylinder>'


def objects():
    skin, shirt, trousers = '0.85 0.65 0.5 1', '0.95 0.45 0.6 1', '0.2 0.35 0.7 1'
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
    box = '<box><size>{}</size></box>'
    debris = ''
    for i, (pose, size) in enumerate([
        ('0 0 0.2 0 0 0.3', '0.5 0.4 0.4'),
        ('0.45 0.3 0.15 0 0 -0.5', '0.4 0.4 0.3'),
        ('-0.2 0.45 0.25 0 0 0.9', '0.35 0.6 0.5'),
        ('0.2 -0.35 0.12 0.2 0 0.1', '0.6 0.3 0.24'),
    ]):
        debris += (f'<collision name="box_{i}_collision"><pose>{pose}</pose><geometry>'
                   f'{box.format(size)}</geometry></collision>'
                   + box_visual(f'box_{i}', pose, size, material('cardboard.jpg')))
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


def overview_camera():
    """A fixed camera above the arena for recordings; it never feeds the robot."""
    sensor = ('<sensor name="overview" type="camera"><update_rate>2</update_rate>'
              '<topic>overview/image</topic><camera><horizontal_fov>1.25</horizontal_fov>'
              '<image><width>960</width><height>640</height></image>'
              '<clip><near>0.5</near><far>30</far></clip></camera></sensor>')
    # Looking straight down, turned so the 12 m arena length runs across the 960 px width.
    return static_model('overview_camera', '4 0 9.5 0 1.5708 1.5708', sensor)


PLUGINS = """
    <physics name="1ms" type="ignored">
      <max_step_size>0.01</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>
    <scene>
      <ambient>0.55 0.55 0.55 1</ambient>
      <background>0.7 0.82 0.95 1</background>
      <shadows>true</shadows>
    </scene>
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>4 0 10 0 0 0</pose>
      <diffuse>0.85 0.85 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.4 0.2 -0.9</direction>
    </light>
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>40 40</size></plane></geometry>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>40 40</size></plane></geometry>
          <material><ambient>0.5 0.55 0.5 1</ambient><diffuse>0.5 0.55 0.5 1</diffuse></material>
        </visual>
      </link>
    </model>
"""


def main():
    make_textures()
    models = [floor_model()]
    models += [wall_model(i, *wall) for i, wall in enumerate(WALLS)]
    models += [sign_model(i, *sign) for i, sign in enumerate(SIGNS)]
    models += objects()
    models.append(overview_camera())
    world = (
        '<?xml version="1.0"?>\n'
        '<!-- Generated by generate_rescue_maze.py; edit the script, not this file. -->\n'
        '<sdf version="1.9">\n  <world name="rescue_maze">'
        + PLUGINS + '\n    '.join([''] + models) + '\n  </world>\n</sdf>\n'
    )
    (HERE / 'rescue_maze.world').write_text(world)
    print(f'wrote {HERE / "rescue_maze.world"} with {len(models)} models')


if __name__ == '__main__':
    main()
