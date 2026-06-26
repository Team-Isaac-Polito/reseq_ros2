from __future__ import annotations

import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

try:
    import PyKDL as kdl
except ImportError as exc:  # pragma: no cover - runtime dependency guard
    raise ImportError(f'PyKDL import failed: {exc}') from exc


@dataclass
class _Origin:
    xyz: list[float]
    rpy: list[float]


@dataclass
class _Joint:
    name: str
    parent: str
    child: str
    joint_type: str
    origin: _Origin
    axis: list[float]


@dataclass
class _Link:
    name: str
    inertial: None = None


class _URDFModel:
    def __init__(self, root_link, joint_map, link_map, children_by_parent):
        self._root_link = root_link
        self.joint_map = joint_map
        self.link_map = link_map
        self._children_by_parent = children_by_parent

    def get_root(self):
        return self._root_link

    def get_chain(self, root, tip, joints=True, links=False, fixed=True):
        def _search(current_link, visited):
            if current_link == tip:
                return []

            for joint_name in self._children_by_parent.get(current_link, []):
                joint = self.joint_map[joint_name]
                if joint.child in visited:
                    continue
                path = _search(joint.child, visited | {joint.child})
                if path is not None:
                    return [joint_name] + path
            return None

        path = _search(root, {root})
        if path is None:
            return []

        if joints:
            return path
        if links:
            links_path = [root]
            for joint_name in path:
                links_path.append(self.joint_map[joint_name].child)
            return links_path
        return []


def _to_vector(values):
    return kdl.Vector(values[0], values[1], values[2])


def _to_rotation(values):
    return kdl.Rotation.RPY(values[0], values[1], values[2])


def _to_frame(origin):
    return kdl.Frame(_to_rotation(origin.rpy), _to_vector(origin.xyz))


def _joint_to_kdl(joint):
    parent_frame = _to_frame(joint.origin)
    axis = _to_vector(joint.axis)

    if joint.joint_type in ('revolute', 'continuous'):
        return kdl.Joint(joint.name, parent_frame.p, parent_frame.M * axis, kdl.Joint.RotAxis)
    if joint.joint_type == 'prismatic':
        return kdl.Joint(joint.name, parent_frame.p, parent_frame.M * axis, kdl.Joint.TransAxis)

    return kdl.Joint(joint.name, kdl.Joint.Fixed)


def _segment_from_joint_and_link(joint, link):
    return kdl.Segment(link.name, _joint_to_kdl(joint), _to_frame(joint.origin))


def _parse_origin(element):
    if element is None:
        return _Origin([0.0, 0.0, 0.0], [0.0, 0.0, 0.0])

    xyz = [0.0, 0.0, 0.0]
    rpy = [0.0, 0.0, 0.0]
    if element.get('xyz'):
        xyz = [float(value) for value in element.get('xyz').split()]
    if element.get('rpy'):
        rpy = [float(value) for value in element.get('rpy').split()]
    return _Origin(xyz, rpy)


def _parse_axis(element):
    if element is None or not element.get('xyz'):
        return [0.0, 0.0, 0.0]
    return [float(value) for value in element.get('xyz').split()]


def _parse_urdf(xml_text):
    root = ET.fromstring(xml_text)

    links = {}
    joint_map = {}
    children_by_parent = {}
    child_links = set()

    for link_el in root.findall('link'):
        name = link_el.get('name')
        if not name:
            continue
        links[name] = _Link(name=name)

    for joint_el in root.findall('joint'):
        name = joint_el.get('name')
        parent_el = joint_el.find('parent')
        child_el = joint_el.find('child')
        if not name or parent_el is None or child_el is None:
            continue
        parent = parent_el.get('link')
        child = child_el.get('link')
        if not parent or not child:
            continue

        joint = _Joint(
            name=name,
            parent=parent,
            child=child,
            joint_type=joint_el.get('type', 'fixed'),
            origin=_parse_origin(joint_el.find('origin')),
            axis=_parse_axis(joint_el.find('axis')),
        )
        joint_map[name] = joint
        children_by_parent.setdefault(parent, []).append(name)
        child_links.add(child)

    root_links = [name for name in links if name not in child_links]
    root_link = root_links[0] if root_links else None
    if root_link is None:
        return None

    return _URDFModel(root_link, joint_map, links, children_by_parent)


class _TreeShim:
    def __init__(self, robot_model):
        self._robot_model = robot_model

    def getChain(self, root, tip, chain):
        joint_names = self._robot_model.get_chain(root, tip, joints=True, links=False, fixed=True)
        if not joint_names:
            return False

        for joint_name in joint_names:
            joint = self._robot_model.joint_map[joint_name]
            child_link = self._robot_model.link_map[joint.child]
            chain.addSegment(_segment_from_joint_and_link(joint, child_link))

        return True


def treeFromUrdfModel(robot_model, tree=None):
    root_name = robot_model.get_root()
    if not root_name:
        return False, None

    return True, _TreeShim(robot_model)


def treeFromString(xml, tree=None):
    try:
        robot_model = _parse_urdf(xml)
    except Exception:
        return False, None

    if robot_model is None:
        return False, None

    return treeFromUrdfModel(robot_model, tree)


def treeFromFile(file_path, tree=None):
    xml = Path(file_path).read_text(encoding='utf-8')
    return treeFromString(xml, tree)
