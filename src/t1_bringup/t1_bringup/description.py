"""W06/W07 visualization wrapper of the frozen external URDF; no asset copies."""
from pathlib import Path
import xml.etree.ElementTree as ET


def robot_description(assets_path):
    path = Path(assets_path) / 'robots/T1/T1_23dof.urdf'
    root = ET.parse(path).getroot()
    links = {link.attrib['name'] for link in root.findall('link')}
    children = {joint.find('child').attrib['link'] for joint in root.findall('joint')}
    if links - children != {'trunk'} or 'base_link' in links:
        raise ValueError('frozen URDF must have sole root trunk, without base_link')
    # Relative meshes are invalid for RViz robot_description; resolve to assets.
    for mesh in root.iter('mesh'):
        filename = mesh.attrib['filename']
        if '://' not in filename:
            resolved = (path.parent / filename).resolve()
            if not resolved.is_file():
                raise ValueError(f'URDF mesh is missing: {resolved}')
            mesh.set('filename', resolved.as_uri())
    # Upstream CAD materials are translucent; make only the ROS visualization
    # opaque for inspection. Geometry, transforms and external files stay intact.
    for color in root.findall('.//visual/material/color'):
        rgba = color.attrib['rgba'].split()
        color.set('rgba', ' '.join(rgba[:3] + ['1']))
    ET.SubElement(root, 'link', name='base_link')
    joint = ET.SubElement(root, 'joint', name='base_link_to_trunk', type='fixed')
    ET.SubElement(joint, 'parent', link='base_link')
    ET.SubElement(joint, 'child', link='trunk')
    ET.SubElement(joint, 'origin', xyz='0 0 0', rpy='0 0 0')
    from t1_mujoco.lidar_config import MOUNT, RAW_FRAME
    if RAW_FRAME in links:
        raise ValueError('frozen URDF must not already contain lidar_link')
    ET.SubElement(root, 'link', name=RAW_FRAME)
    mount = ET.SubElement(root, 'joint', name='trunk_to_lidar', type='fixed')
    ET.SubElement(mount, 'parent', link='trunk')
    ET.SubElement(mount, 'child', link=RAW_FRAME)
    ET.SubElement(mount, 'origin', xyz=' '.join(map(str, MOUNT)), rpy='0 0 0')
    return ET.tostring(root, encoding='unicode')
