import time
from pathlib import Path

import pybullet as p
import pybullet_data


AXIS_LENGTH = 0.08
LABEL_Z_OFFSET = 0.03


def draw_frame(pos, orn, axis_length=AXIS_LENGTH, line_width=2):
    rot = p.getMatrixFromQuaternion(orn)
    x_axis = [rot[0], rot[3], rot[6]]
    y_axis = [rot[1], rot[4], rot[7]]
    z_axis = [rot[2], rot[5], rot[8]]

    x_end = [pos[0] + axis_length * x_axis[0], pos[1] + axis_length * x_axis[1], pos[2] + axis_length * x_axis[2]]
    y_end = [pos[0] + axis_length * y_axis[0], pos[1] + axis_length * y_axis[1], pos[2] + axis_length * y_axis[2]]
    z_end = [pos[0] + axis_length * z_axis[0], pos[1] + axis_length * z_axis[1], pos[2] + axis_length * z_axis[2]]

    p.addUserDebugLine(pos, x_end, [1, 0, 0], line_width)
    p.addUserDebugLine(pos, y_end, [0, 1, 0], line_width)
    p.addUserDebugLine(pos, z_end, [0, 0, 1], line_width)


def get_link_world_pose(body_id, link_index):
    if link_index == -1:
        return p.getBasePositionAndOrientation(body_id)

    link_state = p.getLinkState(body_id, link_index, computeForwardKinematics=True)
    return link_state[4], link_state[5]


def main():
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, 0)

    # Keep this ON so debug sliders are visible
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)

    script_dir = Path(__file__).resolve().parent
    urdf_path = script_dir.parent / "Housden created STL and URDF 5" / "RightArm.urdf"

    robot_id = p.loadURDF(
        str(urdf_path),
        basePosition=[0, 0, 1],
        useFixedBase=True,
    )

    num_joints = p.getNumJoints(robot_id)

    joint_slider_ids = {}
    for joint_index in range(num_joints):
        info = p.getJointInfo(robot_id, joint_index)
        joint_name = info[1].decode("utf-8")
        joint_type = info[2]
        lower = info[8]
        upper = info[9]

        if joint_type in (p.JOINT_REVOLUTE, p.JOINT_PRISMATIC):
            if lower > upper:
                lower, upper = -3.14, 3.14

            joint_slider_ids[joint_index] = p.addUserDebugParameter(
                f"{joint_index}: {joint_name}",
                lower,
                upper,
                0.0,
            )

    yaw_slider = p.addUserDebugParameter("Camera Yaw", -180, 180, 50)
    pitch_slider = p.addUserDebugParameter("Camera Pitch", -89, 89, -30)
    dist_slider = p.addUserDebugParameter("Camera Distance", 0.1, 5.0, 1.5)

    labels_visible = False
    frames_visible = False

    while p.isConnected():
        for joint_index, slider_id in joint_slider_ids.items():
            target = p.readUserDebugParameter(slider_id)
            p.resetJointState(robot_id, joint_index, target)

        yaw = p.readUserDebugParameter(yaw_slider)
        pitch = p.readUserDebugParameter(pitch_slider)
        dist = p.readUserDebugParameter(dist_slider)

        base_pos, _ = p.getBasePositionAndOrientation(robot_id)
        p.resetDebugVisualizerCamera(dist, yaw, pitch, base_pos)

        keys = p.getKeyboardEvents()

        if ord("l") in keys and keys[ord("l")] & p.KEY_WAS_TRIGGERED:
            labels_visible = not labels_visible

        if ord("f") in keys and keys[ord("f")] & p.KEY_WAS_TRIGGERED:
            frames_visible = not frames_visible

        p.removeAllUserDebugItems()

        for link_index in range(-1, num_joints):
            pos, orn = get_link_world_pose(robot_id, link_index)

            if labels_visible:
                if link_index == -1:
                    label = "base"
                    color = [1, 1, 1]
                else:
                    info = p.getJointInfo(robot_id, link_index)
                    label = f"{info[12].decode('utf-8')}"
                    color = [1, 1, 0]
                p.addUserDebugText(
                    label,
                    [pos[0], pos[1], pos[2] + LABEL_Z_OFFSET],
                    textColorRGB=color,
                    textSize=1.2,
                )

            if frames_visible:
                draw_frame(pos, orn)

        p.stepSimulation()
        time.sleep(1 / 3)

    p.disconnect()


if __name__ == "__main__":
    main()
