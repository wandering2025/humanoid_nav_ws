from pathlib import Path
import xml.etree.ElementTree as ET
import pytest
from t1_bringup.description import robot_description


def test_wrapper_preserves_joint_tree_and_resolves_assets(tmp_path):
    path = tmp_path / 'robots/T1'
    path.mkdir(parents=True)
    (path / 'mesh.stl').write_text('test')
    source = '<robot name="T1"><link name="trunk"><visual><geometry><mesh filename="mesh.stl"/></geometry></visual></link><link name="leg"/><joint name="hip" type="revolute"><parent link="trunk"/><child link="leg"/></joint></robot>'
    (path / 'T1_23dof.urdf').write_text(source)
    root = ET.fromstring(robot_description(tmp_path))
    assert {x.attrib['name'] for x in root.findall('link')} == {'base_link','trunk','leg','lidar_link'}
    assert root.find("joint[@name='hip']/parent").attrib['link'] == 'trunk'
    joint = root.find("joint[@name='base_link_to_trunk']")
    assert joint.attrib['type'] == 'fixed'
    assert joint.find('parent').attrib['link'] == 'base_link'
    assert joint.find('child').attrib['link'] == 'trunk'
    assert root.find('.//mesh').attrib['filename'] == (path / 'mesh.stl').as_uri()
    assert (path / 'T1_23dof.urdf').read_text() == source


def test_duplicate_alias_is_rejected(tmp_path):
    path = tmp_path / 'robots/T1'
    path.mkdir(parents=True)
    (path / 'T1_23dof.urdf').write_text('<robot><link name="trunk"/><link name="base_link"/></robot>')
    with pytest.raises(ValueError, match='sole root trunk'):
        robot_description(tmp_path)
