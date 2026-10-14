"""Minimal W06 wrapper of the frozen external URDF; no asset copies."""
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
    ET.SubElement(root, 'link', name='base_link')
    joint = ET.SubElement(root, 'joint', name='base_link_to_trunk', type='fixed')
    ET.SubElement(joint, 'parent', link='base_link')
    ET.SubElement(joint, 'child', link='trunk')
    ET.SubElement(joint, 'origin', xyz='0 0 0', rpy='0 0 0')
    return ET.tostring(root, encoding='unicode')
