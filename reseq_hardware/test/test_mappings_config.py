from pathlib import Path

import yaml


def _mapping_by_msg_id(msg_id: int) -> dict:
    mappings_path = Path(__file__).resolve().parents[1] / 'config' / 'mappings.yaml'
    with mappings_path.open() as stream:
        mappings = yaml.safe_load(stream)

    for mapping in mappings:
        if mapping.get('msg_id') == msg_id:
            return mapping
    raise AssertionError(f'Missing CAN mapping for msg_id 0x{msg_id:02X}')


def test_inter_module_joint_feedback_skips_module_1():
    for msg_id in (0x32, 0x34, 0x36):
        assert _mapping_by_msg_id(msg_id)['modules'] == 'skip_first'


def test_inter_module_joint_commands_skip_module_1():
    for msg_id in (0x61, 0x62, 0x63, 0x64):
        assert _mapping_by_msg_id(msg_id)['modules'] == 'skip_first'
