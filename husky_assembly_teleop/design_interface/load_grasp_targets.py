import os
import json
from compas.data import Data
from compas.geometry import Transformation

class GraspTarget(Data):
    def __init__(self, target_type, **kwargs):
        super(GraspTarget, self).__init__()
        self.type = target_type
        for k, v in kwargs.items():
            setattr(self, k, v)

    @property
    def __data__(self):
        data = {'type': self.type}
        for k, v in self.__dict__.items():
            if k not in ['type', '_guid', '_name']:
                data[k] = v
        return data

    @classmethod
    def __from_data__(cls, data):
        target_type = data.pop('type')
        return cls(target_type, **data)

def parse_transformation(data):
    """Parse a transformation from a dict with a 'matrix' key."""
    if isinstance(data, dict) and 'matrix' in data:
        return Transformation(data['matrix'])
    return data

def parse_grasp_target_dict(d):
    """Parse a dict (from JSON) into a GraspTarget, converting transformations."""
    target_type = d.get('type')
    kwargs = {}
    for k, v in d.items():
        if k == 'type':
            continue
        # Handle nested 'data' for transformations
        if isinstance(v, dict) and v.get('dtype', '').endswith('Transformation'):
            kwargs[k] = parse_transformation(v['data'])
        else:
            kwargs[k] = v
    return GraspTarget(target_type, **kwargs)

def load_grasp_targets(file_path, state_name):
    in_path = os.path.join(file_path, 'RobotCellStates', state_name + '_GraspTargets.json')
    with open(in_path, 'r') as f:
        raw = json.load(f)
    # raw is a list of objects, each with a 'data' key
    targets = []
    for item in raw:
        data = item['data'] if 'data' in item else item
        targets.append(parse_grasp_target_dict(data))
    return targets


# Example parsed grasped targets:

# target = GraspTarget(
#     "direct",
#     world_from_bar=GHPlane_to_tf(world_from_bar),
#     world_from_tool0=GHPlane_to_tf(world_from_tool0)
# )