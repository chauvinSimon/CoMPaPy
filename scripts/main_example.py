import copy

from compapy.scripts.CoMPaPy import CoMPaPy


def main():
    compapy = CoMPaPy()

    compapy.open_gripper()
    compapy.close_gripper()

    init_pose = compapy.move_group.get_current_pose().pose
    target_pose = copy.deepcopy(init_pose)
    target_pose.position.z += 0.1

    compapy.move_j(target_pose)


if __name__ == '__main__':
    main()
