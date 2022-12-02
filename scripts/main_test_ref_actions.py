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


if __name__ == '__main__':
    execute_ref_actions()
