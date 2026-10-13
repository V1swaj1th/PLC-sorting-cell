"""ROS-free helpers: read the scene/config message and build the vision/detections message."""
import json
import math

import yaml


def parse_scene(text):
    """Returns (revision, [(color, x, y, yaw), ...]) from the YAML published on scene/config."""
    try:
        data = yaml.safe_load(text)
    except yaml.YAMLError as e:
        raise ValueError(f'scene/config is not valid YAML: {e}')
    if not isinstance(data, dict) or not isinstance(data.get('objects'), list):
        raise ValueError("scene/config needs a list called 'objects'")
    revision = data.get('revision')
    if not isinstance(revision, int):
        raise ValueError("scene/config needs an integer 'revision'")
    objects = []
    for i, o in enumerate(data['objects']):
        try:
            color = str(o['color'])
            x, y, yaw = float(o['x']), float(o['y']), float(o.get('yaw', 0.0))
        except (KeyError, TypeError, ValueError):
            raise ValueError(f'objects[{i}] needs color, x, y (and optional yaw)')
        if not all(math.isfinite(v) for v in (x, y, yaw)):
            raise ValueError(f'objects[{i}] has a non-finite number')
        objects.append((color, x, y, yaw))
    return revision, objects


def detections_message(revision, detections):
    """One JSON message with the scene revision and all detected cubes, so a consumer never mixes
    cubes of one scene with containers of another."""
    return json.dumps({
        'rev': revision,
        'cubes': [{'color': d['color'], 'x': round(d['x'], 5), 'y': round(d['y'], 5),
                   'yaw': round(d['yaw'], 5)} for d in detections]})
