from reseq_traction_autonomy.parameters import DEFAULTS
from reseq_traction_autonomy.replay import replay, straight_footprint, world_grid

# A 6 x 4 m room around the origin, walls 0.1 m thick.
WORLD = """<sdf version="1.9"><world name="room">
  <model name="ground"><link name="link"><collision name="c">
    <geometry><plane><normal>0 0 1</normal></plane></geometry></collision></link></model>
  <model name="north"><pose>0 2 0 0 0 0</pose><link name="link"><collision name="c">
    <geometry><box><size>6 0.1 1</size></box></geometry></collision></link></model>
  <model name="south"><pose>0 -2 0 0 0 0</pose><link name="link"><collision name="c">
    <geometry><box><size>6 0.1 1</size></box></geometry></collision></link></model>
  <model name="west"><pose>-3 0 0 0 0 1.5708</pose><link name="link"><collision name="c">
    <geometry><box><size>4 0.1 1</size></box></geometry></collision></link></model>
  <model name="east"><pose>3 0 0 0 0 1.5708</pose><link name="link"><collision name="c">
    <geometry><box><size>4 0.1 1</size></box></geometry></collision></link></model>
  <model name="barrel"><pose>1.5 1 0 0 0 0</pose><link name="link"><collision name="c">
    <geometry><cylinder><radius>0.3</radius><length>1</length></cylinder></geometry>
  </collision></link></model>
</world></sdf>"""


def grid(tmp_path):
    path = tmp_path / 'room.world'
    path.write_text(WORLD)
    return world_grid(path)


def event(kind, data):
    return {'event': kind, 'data': data}


def state(x, y, yaw):
    return event('state', {'pose_frame': 'map', 'pose': {'x': x, 'y': y, 'yaw': yaw}})


def prediction(rows, stop=False):
    return event('prediction', {'frame': 'base_link', 'waypoints': rows, 'stop': stop})


def test_world_walls_and_cylinders_become_obstacles(tmp_path):
    room = grid(tmp_path)
    assert room.traversable(0.0, 0.0)
    assert not room.traversable(0.0, 2.0)
    assert not room.traversable(3.0, 0.5)
    assert not room.traversable(1.5, 1.0)
    # Outside the walls is off the map.
    assert not room.traversable(5.0, 0.0)


def test_replay_counts_what_each_prediction_becomes(tmp_path):
    room = grid(tmp_path)
    footprint = straight_footprint(DEFAULTS)
    events = [
        state(0.5, -1.0, 0.0),
        event('observation', {}),
        prediction([[0.5, 0.0, 0.0]]),
        event('observation', {}),
        prediction([[0.0, 0.0, 0.6]]),
        prediction([], stop=True),
    ]
    with_arcs = replay(events, room, footprint, DEFAULTS)
    assert with_arcs['waypoint'] == 1
    assert with_arcs['forward_arc'] == 1
    assert with_arcs['stop'] == 1
    without = replay(events, room, footprint, dict(DEFAULTS, turn_arcs=False))
    assert without['none'] == 1
