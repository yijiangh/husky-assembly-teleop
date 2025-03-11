# https://github.com/CristianoPizzamiglio/circle-fitting-3d
# https://leomariga.github.io/pyRANSAC-3D/

import json, os
from circle_fitting_3d import Circle3D

HERE = os.path.dirname(os.path.abspath(__file__))

# load each json file start with the name "calibration_" in the data folder
json_files = [f for f in os.listdir(HERE) if f.startswith('calibration_') and f.endswith('.json')]

for file_name in json_files:
    print('Working on file: ', file_name)
    file_path = os.path.join(HERE, file_name)

    # Load the JSON file
    with open(file_path, 'r') as file:
        data = json.load(file)

    # Parse the origin of mocap_pose data
    origins = []
    for entry in data:
        mocap_pose = entry.get("mocap_pose", [])
        if mocap_pose:
            origin = mocap_pose[0]  # Assuming the first element is the origin
            origins.append(origin)

    circle_3d = Circle3D(origins)
    print('center: ', circle_3d.center)
    print('radius: ', circle_3d.radius)
    print('normal: ', circle_3d.normal)

    # save these results back to the same json file
    new_data = {'raw_data': data}
    new_data["center"] = list(circle_3d.center)
    new_data["radius"] = circle_3d.radius
    new_data["normal"] = list(circle_3d.normal)

    with open(file_path, 'w') as file:
        json.dump(new_data, file, indent=4)