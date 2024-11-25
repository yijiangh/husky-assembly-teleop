def plan():
    huskyModel = self.huskyModels[0]
    joint_state_slider_values = np.array([p.readUserDebugParameter(ps) for ps in self.joint_state_sliders])
    self.planned_arm_trajectory = plan_transit_motion(
                huskyModel.robot,
                joint_state_slider_values,
                [huskyModel.ee_attachment],
                [],
                debug=True,
                disabled_collisions=False,
            )
    
    print(self.planned_arm_trajectory)
    
    x_range = (-3, 3)
    y_range = (-3, 3)
    
    ob_x_list = [np.inf] # what is this?
    ob_y_list = [np.inf]
    
    rrt_star = RRTStar(
                0.2, *x_range, *y_range, robot_size=0.1, avoid_dist=0.25
            )
    start_point, start_ori = pp.get_pose(huskyModel.robot)
    start_pose = (
        start_point[0],
        start_point[1],
        R.from_quat(start_ori).as_euler("zyx")[0],
    )
    goal_point, goal_ori = pp.get_pose(self.goalModel.robot)
    goal_pose = (
        goal_point[0],
        goal_point[1],
        R.from_quat(goal_ori).as_euler("zyx")[0],
    )
    x_list, y_list = rrt_star.plan(
                ob_x_list, ob_y_list, *(start_pose[:2]), *(goal_pose[:2])
            )
    yaw_list = fill_yaw_angle(start_pose[-1], goal_pose[-1], x_list, y_list)
    targets = [
                State(x, y, yaw)
                for x, y, yaw in zip(x_list, y_list, yaw_list)
            ]
    
    points = [(x, y, 0.0) for x, y in zip(x_list, y_list)]
    with pp.LockRenderer():
        pp.add_segments(points)
        
    controller = Stanley(
            targets[0],
            targets,
            dt=0.1,
            max_steps=2000,
            switch_distance=0.2,
            max_velocity=0.2,
            max_angle_velocity=np.pi,
            stanley_gain=0.75,
            position_tolerance=0.05,
            yaw_tolerance=0.1,
        )
    
    x_list_ctrl, y_list_ctrl, yaw_list_ctrl = controller.run()
    self.planned_base_trajectory = [
            State(x, y, yaw)
            for x, y, yaw in zip(x_list_ctrl, y_list_ctrl, yaw_list_ctrl)
        ]
    
    points = [(x, y, 0.0) for x, y in zip(x_list_ctrl, y_list_ctrl)]
    with pp.LockRenderer():
        pp.add_segments(points, color=RED)