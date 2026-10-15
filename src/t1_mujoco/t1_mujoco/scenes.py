"""Compose original layouts around a frozen robot without changing its physics."""
import hashlib
import json
import math
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

import yaml

ASSETS_SHA = '12a97516f57469078707afb587a5f84c1cb110d1'
SCENES = ('calibration_world', 'static_world', 'dynamic_crossing_world', 'stress_test_world', 'advanced_world')


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def vector(value, length, label):
    if not isinstance(value, list) or len(value) != length or any(
            type(v) not in (int, float) or not math.isfinite(v) for v in value):
        raise ValueError(f'{label} must have {length} finite numbers')
    return value


def load_layout(path):
    return validate_layout(yaml.safe_load(Path(path).read_text()))


def validate_layout(data):
    if not isinstance(data, dict) or data.get('schema') != 1 or data.get('name') not in SCENES:
        raise ValueError('unknown scene/schema')
    if data.get('source_model') != 'robots/T1/T1_23dof.xml':
        raise ValueError('only the frozen 23DOF source model is supported')
    b = vector(data['bounds'], 4, 'bounds')
    if not (b[0] < b[1] and b[2] < b[3]):
        raise ValueError('bounds must be ordered')
    for label in ('initial_pose', 'goal'):
        x, y, _ = vector(data[label], 3, label)
        if not (b[0] < x < b[1] and b[2] < y < b[3]):
            raise ValueError(f'{label} outside inner walls')
    if not (math.isfinite(data['minimum_passage_m']) and data['minimum_passage_m'] > 0):
        raise ValueError('minimum passage must be positive')
    names = set()
    for box in data['boxes']:
        name = box['name']
        if not isinstance(name, str) or not name.isidentifier() or name in names:
            raise ValueError('box names must be unique identifiers')
        names.add(name)
        x, y, z = vector(box['center'], 3, 'center')
        sx, sy, sz = vector(box['size'], 3, 'size')
        if min(sx, sy, sz) <= 0 or abs(z-sz/2) > 1e-9:
            raise ValueError('boxes must have positive full size and stand on z=0')
        if not (b[0] <= x-sx/2 < x+sx/2 <= b[1] and b[2] <= y-sy/2 < y+sy/2 <= b[3]):
            raise ValueError('box outside bounds')
        for label in ('initial_pose', 'goal'):
            px, py, _ = data[label]
            if abs(px-x) <= sx/2 and abs(py-y) <= sy/2:
                raise ValueError(f'{label} inside obstacle')
    regions = set()
    for region in data['activity_regions']:
        name = region['name']
        rb = vector(region['bounds'], 4, 'region bounds')
        if name in regions or not (b[0] <= rb[0] < rb[1] <= b[1] and b[2] <= rb[2] < rb[3] <= b[3]):
            raise ValueError('duplicate or out-of-bounds activity region')
        regions.add(name)
    slots = set()
    for slot in data['obstacle_slots']:
        if (not isinstance(slot['name'], str) or not slot['name'].isidentifier()
                or slot['name'] in slots or slot['region'] not in regions or slot['shape'] != 'box'
                or type(slot['motion_implemented']) is not bool):
            raise ValueError('invalid obstacle slot metadata')
        if min(vector(slot['size'], 3, 'slot size')) <= 0:
            raise ValueError('slot dimensions must be positive')
        slots.add(slot['name'])
    return data


def layout_path(selection, worlds_dir):
    if selection in SCENES:
        return Path(worlds_dir) / (selection + '.yaml')
    path = Path(selection)
    if not path.is_absolute() or not path.is_file():
        raise ValueError('scene must be a known layout name or an existing absolute YAML/MJCF path')
    return path


def assemble(layout_file, assets_path, cache_dir, active_slot=None):
    """Mapping/default builds contain static geometry only; events opt in one mocap."""
    layout_file, assets = Path(layout_file).resolve(), Path(assets_path).resolve()
    data = load_layout(layout_file)
    actual = subprocess.check_output(['git', '-C', str(assets), 'rev-parse', 'HEAD'], text=True).strip()
    dirty = subprocess.check_output(['git', '-C', str(assets), 'status', '--porcelain'], text=True).strip()
    if actual != ASSETS_SHA or dirty:
        raise ValueError('assets must match the clean W01 frozen commit')
    source = assets / data['source_model']
    root = ET.parse(source).getroot()
    compiler = root.find('compiler')
    compiler.set('meshdir', str((source.parent / compiler.get('meshdir', '')).resolve()))
    meshes = {}
    for mesh in root.findall('asset/mesh'):
        path = Path(compiler.get('meshdir')) / mesh.attrib['file']
        meshes[mesh.attrib['name']] = dict(path=str(path), sha256=digest(path))
    # Names change no contact/visual property, but allow stable event attribution.
    for tag in ('geom', 'light'):
        occupied = {element.get('name') for element in root.iter(tag) if element.get('name')}
        for index, element in enumerate(root.iter(tag)):
            if not element.get('name'):
                name = f't1_{tag}_{index:03d}'
                if name in occupied:
                    raise ValueError('generated name conflicts with upstream')
                element.set('name', name)
                occupied.add(name)
    world = root.find('worldbody')
    xmin, xmax, ymin, ymax = data['bounds']
    h, thickness = data['obstacle_height_m'], 0.2
    if not math.isfinite(h) or h <= 0:
        raise ValueError('obstacle height must be positive')
    boxes = [
        ('wall_west', [xmin-thickness/2, (ymin+ymax)/2, h/2], [thickness, ymax-ymin+2*thickness, h]),
        ('wall_east', [xmax+thickness/2, (ymin+ymax)/2, h/2], [thickness, ymax-ymin+2*thickness, h]),
        ('wall_south', [(xmin+xmax)/2, ymin-thickness/2, h/2], [xmax-xmin, thickness, h]),
        ('wall_north', [(xmin+xmax)/2, ymax+thickness/2, h/2], [xmax-xmin, thickness, h]),
    ] + [(box['name'], box['center'], box['size']) for box in data['boxes']]
    for name, center, size in boxes:
        ET.SubElement(world, 'geom', name='env_'+name, type='box',
                      pos=' '.join(map(str, center)), size=' '.join(str(s/2) for s in size),
                      group='2', contype='1', conaffinity='1', rgba='0.65 0.55 0.35 1')
    if active_slot is not None:
        slot = next((s for s in data['obstacle_slots'] if s['name'] == active_slot), None)
        if slot is None or not slot['motion_implemented']:
            raise ValueError('events require an implemented obstacle slot')
        body = ET.SubElement(world, 'body', name='obstacle_'+active_slot,
                             mocap='true', pos='0 0 -10')
        ET.SubElement(body, 'geom', name='obstacle_'+active_slot, type='box',
                      size=' '.join(str(s/2) for s in slot['size']), group='2',
                      contype='1', conaffinity='1', rgba='0.9 0.2 0.15 1')
    provenance = dict(schema=1, name=data['name'], assets_sha=actual,
                      source_xml=dict(path=str(source), sha256=digest(source)),
                      layout=dict(path=str(layout_file), sha256=digest(layout_file)),
                      license=dict(path=str(assets/'LICENSE'), sha256=digest(assets/'LICENSE')),
                      meshes=meshes, scene_metadata=data,
                      dynamic_motion_implemented=active_slot is not None, active_slot=active_slot)
    content = ET.tostring(root, encoding='utf-8', xml_declaration=True)
    key = hashlib.sha256(content + json.dumps(provenance, sort_keys=True).encode()).hexdigest()
    directory = Path(cache_dir) / key
    directory.mkdir(parents=True, exist_ok=True)
    path = directory / (data['name']+'.xml')
    if path.exists() and path.read_bytes() != content:
        raise ValueError('cache content mismatch')
    path.write_bytes(content)
    provenance.update(expanded_xml=dict(path=str(path), sha256=digest(path)))
    (directory/'provenance.json').write_text(json.dumps(provenance, indent=2)+'\n')
    return path, provenance


def parse_start_pose(text):
    """'' keeps the scene initial pose; otherwise 'x y yaw' (m, m, rad in world)."""
    if not text.strip():
        return None
    values = [float(v) for v in text.replace(',', ' ').split()]
    if len(values) != 3 or not all(math.isfinite(v) for v in values):
        raise ValueError('start_pose must be three finite numbers: x y yaw')
    return values


def configure_scene(cfg, selection, assets_path, cache_dir, worlds_dir, start_pose=None, active_slot=None):
    """Select before constructing runner; initial pose is never a motion method.

    start_pose overrides only the pre-construction task initial value of a YAML
    layout (W11 predefined cases). It is validated like the scene's own pose and
    does not change the expanded MJCF or its checksum.
    """
    if not selection:
        if start_pose is not None:
            raise ValueError('start_pose requires a YAML scene layout')
        return None
    path = layout_path(selection, worlds_dir)
    if path.suffix in ('.yaml', '.yml'):
        path, provenance = assemble(path, assets_path, cache_dir, active_slot)
        if start_pose is not None:
            checked = dict(provenance['scene_metadata'], initial_pose=list(start_pose))
            validate_layout(checked)
            provenance['start_pose_override'] = list(start_pose)
        x, y, yaw = start_pose or provenance['scene_metadata']['initial_pose']
        cfg.mujoco.init_pos = [x, y, cfg.mujoco.init_pos[2]]
        cfg.mujoco.init_quat = [math.cos(yaw/2), 0.0, 0.0, math.sin(yaw/2)]
    else:
        if start_pose is not None:
            raise ValueError('start_pose requires a YAML scene layout')
        provenance = dict(expanded_xml=dict(path=str(path), sha256=digest(path)),
                          scene_metadata=None, dynamic_motion_implemented=False)
    cfg.robot = cfg.robot.replace(mjcf_path=str(path))
    return provenance
