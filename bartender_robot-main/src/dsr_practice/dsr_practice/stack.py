# 선반에 적재

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
    pallet1 = {
    'room_1':
    [[598.56, -134.27, 152.68, 57.59, 95.23, 88.33], #back 
    [751.59, 103.39, 129.21, 57.64, 95.12, 88.46], # up
    [750.6, 99.9, 66.92, 57.39, 95.98, 88.54]], # down
    'room_2':
    [[601.36, -110.42, 372.16, 57.42, 95.37, 88.18], #back
    [738.93, 109.18, 346.18, 57.7, 95.62, 88.17], # up
    [737.69, 104.27, 273.57, 57.69, 95.89, 88.55]], #down
    'area_1':
    [[406.54, -237.24, 55.61, 53.37, 91.1, 91.05], # pick back
    [433.52, -198.02, 64.36, 54.97, 90.09, 88.85]] #pick
    }
    dict = {'pallet1':{
            'room_1': None,
            'room_2': None,
            }
        } 
    while rclpy.ok():

        print(f"Moving to joint position: {JReady}")
        movej(JReady, vel=VELOCITY, acc=ACC)
        release()

        key_found = None
        #ask
        stack = input('1:stack, 2: unstack : ')
        if stack == '1':
            space = input('where you stac(room_1/room_2) : ')

        name = input('what you unstack : ')

        # condition
        if stack == '2':  # unstack
            if name not in dict['pallet1'].values():
                print(f"{name} does not exist in pallet1!")
            else:
                for key, value in dict['pallet1'].items():
                    if value == name:
                        key_found = key
                        break
                for pos in pallet1[key_found]:
                    movel(pos,vel=VELOCITY,acc=ACC)
                grip()
                for pos in pallet1[key_found][::-1]:
                    movel(pos=pos,vel=VELOCITY,acc=ACC)
                movel(pallet1['area_1'][1],vel=VELOCITY,acc=ACC)
                release()
                movel(pallet1['area_1'][0],vel=VELOCITY,acc=ACC)
                pick_up = pallet1['area_1'][0].copy()
                pick_up[2] = pallet1['area_1'][0][2] + 100
                movel(pick_up,vel=VELOCITY,acc=ACC)
                dict['pallet1'][space] = None 
                print(f"{name} has been unstacked in area 1")

        elif stack == '1':  # stack
            if space not in dict["pallet1"]:
                print(f"{space} does not exist in pallet1!")
            elif dict["pallet1"][space] is not None:
                print(f"{space} is already occupied!")
            else:
                pick_up = pallet1['area_1'][0].copy()
                pick_up[2] = pallet1['area_1'][0][2] + 100
                movel(pick_up,vel=VELOCITY,acc=ACC)
                movel(pallet1['area_1'][0],vel=VELOCITY,acc=ACC)
                movel(pallet1['area_1'][1],vel=VELOCITY,acc=ACC)
                grip()
                for pos in pallet1[space]:
                    movel(pos,vel=VELOCITY,acc=ACC)
                release()
                for pos in pallet1[space][::-1]:
                    movel(pos=pos,vel=VELOCITY,acc=ACC)

                dict['pallet1'][space] = name  # 또는 원하는 값
                print(f"{name} has been stacked in {space}.")
        else:
            print("Invalid input. Please enter 1 or 2.")
        
        print(f"Moving to joint position: {JReady}")
        movej(JReady, vel=VELOCITY, acc=ACC)
        print(dict)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

# def wait_digital_input(sig_num):
#     while not DR.get_digital_input(sig_num):
#         time.sleep(0.5)
#         print(f"Wait for digital input: {sig_num}")
#         pass


def release():
        print("set for digital output 0 1 for release")
        DR.set_digital_output(1, OFF)
        DR.set_digital_output(2, OFF)
        time.sleep(0.5)
        # wait_digital_input(2)

def grip():
        print("set for digital output 1 0 for grip")
        DR.set_digital_output(1, ON)
        DR.set_digital_output(2, OFF)
        time.sleep(0.5)
        # wait_digital_input(1)
