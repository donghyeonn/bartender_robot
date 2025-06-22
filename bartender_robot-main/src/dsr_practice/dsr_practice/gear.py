# 기어 조립

import rclpy

import time
DR = None
ON, OFF = 1, 0
# for single robot
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 100, 30
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
    #gear1
    gear11 = posx(362, -155.96, 44.12, 168, 179.55, 168.38)
    gear12 = posx(449.16, -100.12, 37.16, 105.01, -179.05, 106.19)
    gear13 = posx(456.78, -206.14, 38.36, 109.62, -178.69, 110.87)
    gear14 = posx(421.51, -155.5, 37.72, 108.92, -178.72, 110.16)
    
    gear11_trans = posx(362, -155.96, 144.12, 168, 179.55, 168.38)
    gear12_trans = posx(449.16, -100.12, 137.16, 105.01, -179.05, 106.19)
    gear13_trans = posx(456.78, -206.14, 138.36, 109.62, -178.69, 110.87)
    gear14_trans = posx(421.51, -155.5, 137.72, 108.92, -178.72, 110.16)

    #gear2
    gear21 = posx(390.93, 195.01, 67.85, 128.6, -178.67, 129.94)
    gear22 = posx(485.83, 147.58, 71.7, 132.82, -178.25, 134.68)
    gear23 = posx(396.98, 89.34, 71.4, 132.74, -178.28, 134.42)
    gear24 = posx(425.4, 144.35, 69.78, 137.92, -178.3, 139.45)

    gear21_trans = posx(390.93, 195.01, 137.85, 128.6, -178.67, 129.94)
    gear22_trans = posx(485.83, 147.58, 141.7, 132.82, -178.25, 134.68)
    gear23_trans = posx(396.98, 89.34, 141.4, 132.74, -178.28, 134.42)
    gear24_trans = posx(425.4, 144.35, 139.78, 137.92, -178.3, 139.45)

    if rclpy.ok():

        print(f"Moving to joint position: {JReady}")
        movej(JReady, vel=VELOCITY, acc=ACC)
        release()
        # #1down grap up
        # down_grip_up(gear11,gear11_trans)

        # #1down release up
        # down_release_up(gear12,gear12_trans)

        # #2down grap up
        # down_grip_up(gear21,gear21_trans)

        # #2down release up
        # down_release_up(gear22,gear22_trans)

        # #3down grap up
        # down_grip_up(gear13,gear13_trans)

        # #3down release up
        # down_release_up(gear23,gear23_trans)
        #4down grap up
        down_grip_up(gear14,gear14_trans)

        # 4down release up
        print(f"Moving to joint position: {gear24_trans}")
        movel(gear24_trans, vel=VELOCITY, acc=ACC)

        print(f"Moving to joint position: {gear24}")
        movel(gear24, vel=VELOCITY, acc=ACC)
        gear_spin()
        release()

        print(f"Moving to joint position: {gear24_trans}")
        movel(gear24_trans, vel=VELOCITY, acc=ACC)

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
    DR.movel(gear, vel=VELOCITY, acc=ACC)

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
    push_gear()
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

def push_gear():
    DR.task_compliance_ctrl(stx=[5, 5, 500, 100, 100, 100])
    time.sleep(0.1)
    print("Starting set_desired_force")
    DR.set_desired_force(fd=[0, 0, -10, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR.DR_FC_MOD_REL)

    while not DR.check_position_condition(DR.DR_AXIS_Z, max=44,ref=DR.DR_BASE):
        print("Waiting for an external position greater than 44 ")
        time.sleep(0.5)
        pass

    print("Starting release_force")
    DR.release_force()
    time.sleep(0.1)
    
    print("Starting release_compliance_ctrl")      
    DR.release_compliance_ctrl()

def gear_spin():
    # MovePeriodicNode
    DR.task_compliance_ctrl(stx=[5, 5, 500, 100, 100, 100])
    time.sleep(0.1)
    print("Starting set_desired_force")
    DR.set_desired_force(fd=[0, 0, -10, 0, 0, 0], dir=[0, 0, 1, 0, 0, 0], mod=DR.DR_FC_MOD_REL)

    DR.amove_periodic(amp=[0.00, 0.00, 0.00, 0.00, 0.00, 15.00], period=[0.00, 0.00, 0.00, 0.00, 0.00, 2.00], atime=1.00, repeat=10, ref=DR.DR_TOOL)
    # RepeatNode
    while True:
        if not DR.check_position_condition(axis=DR.DR_AXIS_Z, max=44, ref=DR.DR_BASE):
            pass
            time.sleep(0.50)
            break
    print("Starting release_force")
    DR.release_force()
    time.sleep(0.1)
    
    print("Starting release_compliance_ctrl")      
    DR.release_compliance_ctrl()
