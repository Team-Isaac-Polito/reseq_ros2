"""SDF pieces and textures shared by the world generators in this folder."""

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
PAINT = {'blue': '0.25 0.45 0.75 1', 'yellow': '0.9 0.75 0.2 1'}


def make_base_textures():
    """Plywood walls, OSB floor and the START/END floor markings."""
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

    zone_marking('start_zone', (40, 170, 70), 'START')
    zone_marking('end_zone', (235, 190, 30), 'END')


def zone_marking(name, color, text, size=(512, 512)):
    """A painted floor outline with a label. size is in pixels, (along x, along y)."""
    w, h = size
    mark = Image.new('RGB', size, (176, 140, 92))
    draw = ImageDraw.Draw(mark)
    draw.rectangle([8, 8, w - 8, h - 8], outline=color, width=36)
    font = ImageFont.truetype(FONT, 110)
    draw.text((w / 2, h / 2), text, fill=color, anchor='mm', font=font)
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


def wall_model(index, x1, y1, x2, y2, finish='plywood', height=WALL_HEIGHT):
    length = math.hypot(x2 - x1, y2 - y1)
    yaw = math.atan2(y2 - y1, x2 - x1)
    cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
    body = (f'<collision name="collision"><geometry><box><size>{length:.3f} {WALL_THICKNESS} '
            f'{height}</size></box></geometry></collision>')
    # One visual per plywood sheet so the grain is not stretched along long walls.
    panels = max(1, math.ceil(length / PANEL_WIDTH))
    width = length / panels
    mat = material('plywood.jpg') if finish == 'plywood' else material(color=PAINT[finish])
    for i in range(panels):
        offset = -length / 2 + width * (i + 0.5)
        body += box_visual(f'panel_{i}', f'{offset:.3f} 0 0 0 0 0',
                           f'{width - 0.01:.3f} {WALL_THICKNESS} {height}', mat)
    return static_model(f'wall_{index}', f'{cx:.3f} {cy:.3f} {height / 2} 0 0 {yaw:.4f}', body)


def solid(name, pose, geometry, color):
    """A static object whose collision matches its visual."""
    return (f'<collision name="{name}_collision"><pose>{pose}</pose><geometry>{geometry}'
            f'</geometry></collision><visual name="{name}"><pose>{pose}</pose><geometry>'
            f'{geometry}</geometry>{material(color=color)}</visual>')


def cylinder(radius, length):
    return f'<cylinder><radius>{radius}</radius><length>{length}</length></cylinder>'


def overview_camera(x, y, height, fov=1.25):
    """A fixed camera looking straight down, for recordings only. It never feeds the robot."""
    sensor = (f'<sensor name="overview" type="camera"><update_rate>2</update_rate>'
              f'<topic>overview/image</topic><camera><horizontal_fov>{fov}</horizontal_fov>'
              '<image><width>960</width><height>640</height></image>'
              '<clip><near>0.5</near><far>30</far></clip></camera></sensor>')
    # Turned so the world x axis runs across the 960 px image width.
    return static_model('overview_camera', f'{x} {y} {height} 0 1.5708 1.5708', sensor)


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


def write_world(name, models, generator):
    """Write <name>.world next to this file with the shared plugins, light and ground."""
    world = (
        '<?xml version="1.0"?>\n'
        f'<!-- Generated by {generator}; edit the script, not this file. -->\n'
        f'<sdf version="1.9">\n  <world name="{name}">'
        + PLUGINS + '\n    '.join([''] + models) + '\n  </world>\n</sdf>\n'
    )
    path = HERE / f'{name}.world'
    path.write_text(world)
    print(f'wrote {path} with {len(models)} models')
