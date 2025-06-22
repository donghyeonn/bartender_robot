# 팔레트 무작위 -> 정렬

import rclpy

import time
DR = None
ON, OFF = 1, 0
# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 300, 30
def main(args=None):
    
    global DR
   
    import DR_init

    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL
    

    rclpy.init(args=args)
    node = rclpy.create_node("gear_node", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    # here
    try:
        import DSR_ROBOT2 as DR
        from DSR_ROBOT2 import (
            release_compliance_ctrl,
            release_force,
            check_position_condition,
            task_compliance_ctrl,
            set_desired_force,
            get_current_posx,
            amove_periodic,
            set_tool,
            set_tcp,
            movel,
            movej,
            wait,
            set_digital_output,
            get_digital_input,
            DR_FC_MOD_REL,
            DR_AXIS_Z,
            DR_TOOL,
            DR_BASE,
        )
        from DR_common2 import posj,posx
    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")

    set_tool("Tool Weighttest")
    set_tcp("GripperDA_v2")
    JReady = posj([0, 0, 90, 0, 90, 0])
    # pallet1
    pallet1 = [
    [346.12, -155, 62.25, 176.26, 179.85, 176.15],
    [399.02, -155.37, 61.94, 176.26, 179.85, 176.15],
    [448.31, -156.22, 62.64, 176.26, 179.85, 176.15],
    [348.34, -105.82, 63.08, 176.26, 179.85, 176.15],
    [398.13, -106.4, 63.36, 176.26, 179.85, 176.15],
    [447.39, -106.23, 64.59, 176.26, 179.85, 176.15],
    [347.15, -54.52, 62.58, 176.26, 179.85, 176.15],
    [397.31, -53.74, 64.05, 176.26, 179.85, 176.15],
    [449.26, -54.15, 64.51, 176.26, 179.85, 176.15]
    ]
    # pallet2
    pallet2 = [
    [345.88, 45.08, 36.23, 176.26, 179.85, 176.15],
    [397.73, 43.16, 37.37, 176.26, 179.85, 176.15],
    [448.71, 42.31, 36.82, 176.26, 179.85, 176.15],
    [347.08, 97.11, 35.72, 176.26, 179.85, 176.15],
    [399.48, 95.38, 34.33, 176.26, 179.85, 176.15],
    [451.95, 94.48, 32.08, 176.26, 179.85, 176.15],
    [349.69, 147.41, 30.13, 176.26, 179.85, 176.15],
    [400.02, 144.9, 30.54, 176.26, 179.85, 176.15],
    [452.5, 145.39, 32.1, 176.26, 179.85, 176.15]
    ]

    if rclpy.ok():

        print(f"Moving to joint position: {JReady}")
        movej(JReady, vel=VELOCITY, acc=ACC)
        grip()
        #check height
        height_list = []
        for idx,pose in enumerate(pallet1):
            pose_up = pose.copy()
            pose_up[2] = pose[2] + 100
            movel(pose_up, vel=VELOCITY, acc=ACC)
            movel(pose, vel=VELOCITY, acc=ACC)
            check_pose = check_bar()[0]
            height_list.append([idx,abs(pose[2] - check_pose[2])])
            movel(pose_up, vel=VELOCITY, acc=ACC)

        height_list.sort(key=lambda x: x[1])
        sorted_index = [item[0] for item in height_list]
        #  print(sorted_index)

        print(f"Moving to joint position: {JReady}")
        movej(JReady, vel=VELOCITY, acc=ACC)
        release()
        for idx, pose2 in enumerate(pallet2):
            pose1 = pallet1[sorted_index[idx]]
            pose1_down = pose1.copy()
            pose1_down[2] = pose1[2] - 35
            pose1_up = pose1.copy()
            pose1_up[2] = pose1[2] + 100
            down_grip_up(pose1_down,pose1_up)
            pose2_up = pose2.copy()
            pose2_up[2] = pose2[2] + 100
            down_release_up(pose2,pose2_up)
        #aline
        
        print(f"Moving to joint position: {JReady}")
        movej(JReady, vel=VELOCITY, acc=ACC)

    rclpy.shutdown()


if __name__ == "__main__":
    main()

def wait_digital_input(sig_num):
    while not DR.get_digital_input(sig_num):
        time.sleep(0.5)
        print(f"Wait for digital input: {sig_num}")
        pass

def down_grip_up(gear,gear_trans):
    print(f"Moving to joint position: {gear_trans}")
    DR.movel(gear_trans, vel=VELOCITY, acc=ACC)

    print(f"Moving to joint position: {gear}")
    DR.movel(gear, vel=VELOCITY, acc=ACC)
    grip()

    print(f"Moving to joint position: {gear_trans}")
    DR.movel(gear_trans, vel=VELOCITY, acc=ACC)

def down_release_up(gear,gear_trans):
    print(f"Moving to joint position: {gear_trans}")
    DR.movel(gear_trans, vel=VELOCITY, acc=ACC)

    print(f"Moving to joint position: {gear}")
    DR.movel(gear, vel=VELOCITY, acc=ACC)
    release_force()
    release()

    print(f"Moving to joint position: {gear_trans}")
    DR.movel(gear_trans, vel=VELOCITY, acc=ACC)

def release():
        print("set for digital output 0 1 for release")
        DR.set_digital_output(1, OFF)
        DR.set_digital_output(2, ON)
        time.sleep(0.5)
        # wait_digital_input(2)

def grip():
        print("set for digital output 1 0 for grip")
        DR.set_digital_output(1, ON)
        DR.set_digital_output(2, ON)
        time.sleep(0.5)
        # wait_digital_input(1)

def check_bar():
    DR.task_compliance_ctrl(stx=[5, 5, 500, 100, 100, 100])
    time.sleep(0.1)
    print("Starting set_desired_force")
    DR.set_desired_force(fd=[0, 0, -10, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR.DR_FC_MOD_REL)

    while DR.check_force_condition(DR.DR_AXIS_Z, min=10,ref=DR.DR_TOOL):
        print("Waiting for an external position greater than 10 ")
        time.sleep(0.5)
        pass
    
    print("Starting release_force")
    DR.release_force()
    time.sleep(0.1)
    pos = DR.get_current_posx(DR.DR_BASE)
    print("Starting release_compliance_ctrl")      
    DR.release_compliance_ctrl()
    return pos

def release_force():
    DR.task_compliance_ctrl(stx=[5, 5, 500, 100, 100, 100])
    time.sleep(0.1)
    print("Starting set_desired_force")
    DR.set_desired_force(fd=[0, 0, -10, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR.DR_FC_MOD_REL)

    while DR.check_force_condition(DR.DR_AXIS_Z, min=10,ref=DR.DR_TOOL):
        print("Waiting for an external position greater than 10 ")
        time.sleep(0.5)
        pass
    
    print("Starting release_force")
    DR.release_force()
    time.sleep(0.1)

    print("Starting release_compliance_ctrl")      
    DR.release_compliance_ctrl()
