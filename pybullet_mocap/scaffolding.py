import os, json
import numpy as np
import pybullet_planning as pp
from typing import Dict
from collections import defaultdict
from pybullet_mocap import DATA_DIRECTORY

COUPLER_DIR = os.path.join(DATA_DIRECTORY, 'coupler')
SCAFFOLDING_DIR = os.path.join(DATA_DIRECTORY, 'scaffolding_design')

BUFF_COLOR = (219/256, 153/256, 90/256, 1)
COUPLER_BODY_COLORS = [pp.apply_alpha(pp.BROWN, 1.0), pp.apply_alpha(BUFF_COLOR, 1.0), pp.apply_alpha(pp.BROWN, 0.5)]

def flatten_list(nested):
    inds = []
    for p in nested:
        inds.extend(p)
    return inds

def list_to_pairs(inds):
    assert len(inds) % 2 == 0
    return [(inds[i], inds[i+1]) for i in range(0,len(inds),2)]

def convex_combination(x1, x2, w=0.5):
    assert 0 <= w and w <= 1
    return (1 - w) * np.array(x1) + (w * np.array(x2))

##################################

class HalfCoupler:
    def __init__(self, bodies, pose, at_element, to_element):
        self.bodies = bodies
        self.pose = pose
        self.at_element = at_element
        self.to_element = to_element
    
    def update_pose(self, pose=None):
        pose = pose or self.pose
        set_pose_batch(self.bodies, pose)

    def flip_pose_z_axis(self, pose=None):
        pose = pose or self.pose
        self.update_pose(flip_pose_z_axis(pose))

    def set_color(self, color):
        for b in self.bodies:
            pp.set_color(b, color)

    def reset_color(self):
        for i, b in enumerate(self.bodies):
            pp.set_color(b, COUPLER_BODY_COLORS[i])

    def focus_camera(self):
        pp.camera_focus_on_body(body=self.bodies[0])

    def is_dual_coupler(self, coupler):
        return self.at_element == coupler.to_element and self.to_element == coupler.at_element

    @classmethod
    def from_data(cls, data: Dict):
        if pp.is_connected():
            bodies = load_half_coupler_parts()
        else:
            bodies = [None, None, None]
        pose = pp.pose_from_tform(data['pose'])
        return cls(bodies, pose, data['at_element'], data['to_element'])

    def to_data(self):
        return {
            'pose': pp.tform_from_pose(self.pose).tolist(), # list per row
            'at_element': self.at_element,
            'to_element': self.to_element
        }

def pose_from_yz_frame(origin, y_axis, z_axis):
    tform = np.eye(4)
    tform[:3,3] = origin
    tform[:3,0] = np.cross(y_axis, z_axis)
    tform[:3,1] = y_axis
    tform[:3,2] = z_axis
    return pp.pose_from_tform(tform)

def load_half_coupler_parts():
    cbs = []
    for i in range(3):
        cb = pp.create_obj(os.path.join(COUPLER_DIR,f'half_coupler_{i}.obj'), color=COUPLER_BODY_COLORS[i], mass=pp.STATIC_MASS)
        cbs.append(cb)
    return cbs

def set_pose_batch(bodies, pose):
    for b in bodies:
        pp.set_pose(b, pose)

def flip_pose_z_axis(pose):
    tform = pp.tform_from_pose(pose)
    tform[:3,2] *= -1
    # recompute x axis to keep right-hand coordinate system
    tform[:3,0] = np.cross(tform[:3,1], tform[:3,2])
    return pp.pose_from_tform(tform)

def debug_coupler_pose(contact_point1, contact_point2, bar1_dir, bar2_dir, bar1_nodes, bar2_nodes):
    pp.draw_pose(pp.unit_pose(), length=0.1)
    contact_dir = pp.get_difference(contact_point1, contact_point2)
    pp.add_line(contact_point1, contact_point2, color=pp.apply_alpha(pp.BLACK, 1), width=2)
    pp.add_line(bar1_nodes[1], bar1_nodes[0], color=pp.apply_alpha(pp.BLACK, 1), width=2)
    pp.add_line(bar2_nodes[1], bar2_nodes[0], color=pp.apply_alpha(pp.BLACK, 1), width=2)
    pp.draw_pose(pose_from_yz_frame(contact_point1, -contact_dir, bar1_dir))

# distance between a point and a line segment
def compute_closest_point_to_line(point, line_point_1, line_point_2, opt_tol=1e-8):
    p1 = line_point_1
    p2 = line_point_2
    d = p2 - p1
    assert np.linalg.norm(d) > opt_tol, 'degenerate line segment'

    t = -(p1 - point).dot(d) / d.dot(d)
    return np.clip(t, 0, 1)

def compute_closest_t_between_lines(line_point_1_1, line_point_1_2, line_point_2_1, line_point_2_2, opt_tol=1e-8):
    p1 = line_point_1_1
    p2 = line_point_1_2
    p3 = line_point_2_1
    p4 = line_point_2_2
    d1 = p2 - p1
    d2 = p4 - p3
    assert np.linalg.norm(d1) > opt_tol and np.linalg.norm(d2) > opt_tol, 'degenerate line segment'

    A = np.array([[d1.dot(d1), -d1.dot(d2)], 
                  [d2.dot(d1), -d2.dot(d2)]])
    denominator = np.linalg.det(A)
    b = np.array([(p3 - p1).dot(d1), (p3 - p1).dot(d2)])

    # if two lines are not parallel
    if abs(denominator) > opt_tol:
        t = np.linalg.inv(A) @ b
        # intersection happens inbetween the two line segments
        if t[0] >= 0 and t[0] <= 1 and t[1] >=0 and t[1] <= 1:
            return t

    #the closest points must include a end point of the two line segments
    ts = [[0, None], [1, None], [None, 0], [None, 1]]
    min_dist = np.inf
    min_t = []
    for tc in ts:
        t = []
        if tc[0] == None:
            pt = convex_combination(p3, p4, tc[1])
            t0 = compute_closest_point_to_line(pt, p1, p2, opt_tol)
            t = [t0, tc[1]]
        elif tc[1] == None:
            pt = convex_combination(p1, p2, tc[0])
            t1 = compute_closest_point_to_line(pt, p3, p4, opt_tol)
            t = [tc[0], t1]

        dist = np.linalg.norm(convex_combination(p1, p2, t[0]) - convex_combination(p3, p4, t[1]))
        #print(t, dist)
        if min_dist > dist:
            min_t = t
            min_dist = dist

    return min_t

def create_couplers(line_pts_flattened, contact_id_pairs):
    contact_ts = []
    for ei, ej in contact_id_pairs:
        t1, t2 = compute_closest_t_between_lines(line_pts_flattened[ei*2], line_pts_flattened[ei*2+1], line_pts_flattened[ej*2], line_pts_flattened[ej*2+1])
        contact_ts.extend([t1,t2])

    node_pairs = list_to_pairs(line_pts_flattened)
    contact_t_pairs = list_to_pairs(contact_ts)
    half_couplers = defaultdict(list)
    with pp.LockRenderer():
        for contact_idp, contact_tp in zip(contact_id_pairs, contact_t_pairs):
            e0, e1 = contact_idp
            # collision checking between clamps and bars performed inside
            coupler_pair = create_swivel_coupler(node_pairs, e0, e1, *contact_tp)
            half_couplers[frozenset([e0, e1])] = coupler_pair
    return half_couplers

# load half_coupler.obj from data path, create a collision body, and set the pose to the contact point
def create_swivel_coupler(node_pairs, e0, e1, t1, t2):
    if not pp.is_connected():
        return None
    bar1_nodes = node_pairs[e0]
    bar2_nodes = node_pairs[e1]
 
    contact_point1 = convex_combination(*bar1_nodes, t1)
    contact_point2 = convex_combination(*bar2_nodes, t2)
    contact_dir = pp.get_unit_vector(contact_point2 - contact_point1)
    bar1_dir = pp.get_unit_vector(bar1_nodes[1] - bar1_nodes[0])
    bar2_dir = pp.get_unit_vector(bar2_nodes[1] - bar2_nodes[0])

    half_coupler_bodies = load_half_coupler_parts()
    pose1 = pose_from_yz_frame(contact_point1, -contact_dir, bar1_dir)
    set_pose_batch(half_coupler_bodies, pose1)

    half_coupler_bodies2 = load_half_coupler_parts()
    pose2 = pose_from_yz_frame(contact_point2, contact_dir, bar2_dir)
    set_pose_batch(half_coupler_bodies2, pose2)

    return HalfCoupler(half_coupler_bodies, pose1, e0, e1), HalfCoupler(half_coupler_bodies2, pose2, e1, e0)

#############################################

def parse_mt_geometric(mt_json_file_name):
    file_path = os.path.join(SCAFFOLDING_DIR, mt_json_file_name)
    with open(file_path, 'r') as f:
        json_data = json.load(f)

    line_pt_pairs = json_data['line_pt_pairs']
    contact_id_pairs = json_data['contact_id_pairs']

    if 'opt_parameters' in json_data:
        bar_radius = json_data['opt_parameters'].get('bar_radius', 0.01)
    else:
        bar_radius = 0.01

    return line_pt_pairs, contact_id_pairs, bar_radius

def create_bar_body(_p1, _p2, bar_radius, scale=1.0, use_box=False, color=pp.apply_alpha(pp.RED, 1), shrink_radius=0.0):
    """create bar's collision body in pybullet
    """
    if not pp.is_connected():
        return None
    p1 = np.array(_p1) * scale
    p2 = np.array(_p2) * scale
    # height = max(np.linalg.norm(p2 - p1) - 2*shrink, 0)
    height = max(np.linalg.norm(p2 - p1), 0)
    center = (p1 + p2) / 2

    delta = p2 - p1
    x, y, z = delta
    phi = np.math.atan2(y, x)
    theta = np.math.acos(z / np.linalg.norm(delta))
    quat = pp.quat_from_euler(pp.Euler(pitch=theta, yaw=phi))
    # p1 is z=-height/2, p2 is z=+height/2
    diameter = 2*(bar_radius*scale - shrink_radius)

    if use_box:
        # Much smaller than cylinder
        # use inscribed square
        h = diameter / np.sqrt(2)
        body = pp.create_box(h, h, height, color=color, mass=pp.STATIC_MASS)
    else:
        # Visually, smallest diameter is 2e-3
        # The geometries and bounding boxes seem correct though
        body = pp.create_cylinder(diameter/2, height, color=color, mass=pp.STATIC_MASS)
        # print('Diameter={:.5f} | Height={:.5f}'.format(diameter/2., height))
        # print(get_aabb_extent(get_aabb(body)).round(6).tolist())
        # print(get_visual_data(body))
        # print(get_collision_data(body))

    pp.set_point(body, center)
    pp.set_quat(body, quat)
    pp.set_color(body, color)
    # draw_aabb(get_aabb(body))
    # draw_pose(get_pose(body), length=5e-3)
    return body

def create_collision_bodies(lines, radius_per_edge, viewer=False, **kwargs):
    # input: (num_linex2) x 3 line nodal position np array
    if not pp.is_connected():
        pp.connect(use_gui=viewer)
    bodies = []
    centroid = np.mean(lines, axis=0)
    camera_offset = 3 * np.array([1, 1, 1])
    pp.set_camera_pose(camera_point=centroid + camera_offset, target_point=centroid)
    assert len(lines) / 2 == len(radius_per_edge)
    with pp.LockRenderer():
        for i, r in zip(range(0,len(lines),2), radius_per_edge):
            body = create_bar_body(lines[i], lines[i+1], r, **kwargs)
            bodies.append(body)
    return bodies