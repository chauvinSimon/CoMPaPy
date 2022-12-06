import copy

from compapy.scripts.CoMPaPy import CoMPaPy


def execute_ref_actions():
    # todo: test the end state after each execution as a regression test

    compapy = CoMPaPy()

    # gripper
    compapy.open_gripper()
    compapy.close_gripper()
    compapy.open_gripper()
    compapy.close_gripper()

    # reach a ref position, stretching up as a giraffe
    compapy.go_to_joint_state()

    # move inside a virtual cube
    cube_size = 0.05
    init_pose = compapy.move_group.get_current_pose().pose
    targets = []
    # 1- along the internal "long" diagonal
    target_1 = copy.deepcopy(init_pose)
    target_1.position.x += cube_size
    target_1.position.y += cube_size
    target_1.position.z += cube_size
    targets.append(target_1)

    # 2- along the external diagonal of one face
    target_2 = copy.deepcopy(init_pose)
    target_2.position.y += cube_size
    targets.append(target_2)

    # 3- along the edge of one face, vertically
    target_3 = copy.deepcopy(init_pose)
    target_3.position.y += cube_size
    target_3.position.z += cube_size
    targets.append(target_3)

    # 4- along the external diagonal of one face, back to the init
    targets.append(init_pose)

    for target_pose in targets:
        compapy.move_l(target_pose)

    # some "not too stretched" pose
    target_pose = compapy.move_group.get_current_pose().pose
    target_pose.position.x += 0.3
    target_pose.position.z -= 0.3
    compapy.move_l(target_pose)

    # rotations of the gripper
    init_pose = compapy.move_group.get_current_pose().pose
    for q in [
        # quaternion in scalar-last
        # target is not the eef. Rather the link-8. Therefore, 45° correction
        [0.92, -0.38, -0.0, 0.0],  # ref-pose: gripper points down: euler.xyz = [180.0, 0.0, -45.0]

        [1.0, 0.0, 0.0, 0.0],  # 45° around z: [180.0, 0.0, 0.0]
        [0.71, -0.71, -0.0, 0.0],  # -45° around z: [180.0, 0.0, -90.0]

        [0.92, -0.38, -0.0, 0.0],  # ref-pose

        [0.85, -0.35, -0.35, -0.15],  # 45° around y: [180.0, 45.0, -45.0]
        [0.85, -0.35, 0.35, 0.15],  # -45° around y: [180.0, -45.0, -45.0]

        [0.92, -0.38, -0.0, 0.0],  # ref-pose
    ]:
        target = copy.deepcopy(init_pose)
        target.orientation.x = q[0]
        target.orientation.y = q[1]
        target.orientation.z = q[2]
        target.orientation.w = q[3]
        compapy.move_l(target)
    compapy.move_l(init_pose)


if __name__ == '__main__':
    execute_ref_actions()
