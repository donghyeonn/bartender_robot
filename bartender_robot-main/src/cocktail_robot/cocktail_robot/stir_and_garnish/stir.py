# pick and place in 1 method. from pos1 to pos2 @20241104
import DR_init
import time
from ..utils.base_action import BaseAction



VELOCITY, ACCURACY = 85, 60

ON, OFF = 1, 0
### 힘 제어 : BASE 좌표계 기준
class StirAction(BaseAction):
    def __init__(self, node, poses):
        DR_init.__dsr__node = node

        try:
            import DSR_ROBOT2

        except ImportError as e:
            print(f"Error importing DSR_ROBOT2 : {e}")
            return
        
        global DR
        DR = DSR_ROBOT2

        self.stir_pose = poses
        self.grasp_option = 1

        
    def execute(self):
        # print('task ready')
        DR.movej(pos=[0,0,90,0,90,0], vel=VELOCITY*0.5, acc=ACCURACY)
        self.release(self.grasp_option)
        DR.movel(pos=self.stir_pose["task_ready"]["task"], vel=VELOCITY, acc = ACCURACY)

        # print('grasp position')
        DR.movej(self.stir_pose["spoon_grasp_ready"]["joint"], vel=VELOCITY*0.5, acc = ACCURACY)
        DR.movej(self.stir_pose["spoon_grasp_ready_down"]["joint"], vel=VELOCITY*0.4, acc = ACCURACY)
        DR.movej(self.stir_pose["spoon_grasp"]["joint"], vel=VELOCITY*0.4, acc = ACCURACY)
        
        # print('grasp')
        self.grasp(self.grasp_option)

        # print('grasp up')
        DR.movel(pos=[0,0,330,0,0,0], vel=VELOCITY, acc = ACCURACY, mod=DR.DR_MV_MOD_REL, ref=DR.DR_BASE)

        # print('stir position')
        DR.movej(pos=self.stir_pose["task_ready"]["joint"], vel=VELOCITY*0.7, acc = ACCURACY)        
        
        DR.movej(self.stir_pose["stir_ready"]["joint"], vel=VELOCITY*0.6, acc = ACCURACY)
        # print('stir')
        self.down_stir(target_pos=self.stir_pose["stir"]["task"])
        
        # print('stir up')
        DR.movel(self.stir_pose["stir_ready"]["task"], vel=VELOCITY*0.8, acc = ACCURACY)
        
        # print('go back and release')
        DR.movej(self.stir_pose["task_ready"]["joint"], vel=VELOCITY*0.7, acc = ACCURACY)
        print('ready')
        # DR.movej(self.stir_pose["spoon_grasp_up"]["joint"], vel=VELOCITY*0.3, acc = ACCURACY)

        DR.movej(self.stir_pose["spoon_put_up"]["joint"], vel=VELOCITY*0.8, acc = ACCURACY)
        DR.movel(pos=[0,0,-330,0,0,0], vel=VELOCITY, acc = ACCURACY, mod=DR.DR_MV_MOD_REL, ref=DR.DR_BASE)
        self.release(self.grasp_option)
        
        DR.movej(self.stir_pose["spoon_grasp_ready_down"]["joint"], vel=VELOCITY*0.4, acc = ACCURACY)
        DR.movej(self.stir_pose["spoon_grasp_ready"]["joint"], vel=VELOCITY*0.4, acc = ACCURACY)
        DR.movej(self.stir_pose["task_ready"]["joint"], vel=VELOCITY*0.5, acc = ACCURACY)


    def force_down_stir(self, target_pos, turning_radius=10, stir_repeat=4, force_desired=30):
        DR.set_ref_coord(DR.DR_BASE)
        k_d = [5, 5, 10, 200, 200, 200] ## need to check

        f_d = [0.0, 0.0, -force_desired, 0.0, 0.0, 0.0]
        f_dir = [0, 0, 1, 0, 0, 0]
        
        DR.task_compliance_ctrl(k_d)
        time.sleep(0.1)
        DR.set_desired_force(fd=f_d, dir=f_dir, mod=DR.DR_FC_MOD_ABS)
        
        while True:
            if not DR.check_position_condition(axis=DR.DR_AXIS_Z, max=target_pos, ref=DR.DR_BASE):
                time.sleep(0.5)
                break

        DR.release_force(time=0.5)
        DR.release_compliance_ctrl()

        DR.move_periodic(
            amp=[turning_radius, turning_radius, 0, 0, 0, 0],
            period=[2, 3, 0, 0, 0, 0],
            repeat=stir_repeat,
            ref=DR.DR_BASE
            )

    def down_stir(self, target_pos, turning_radius=10, stir_repeat=4):
        DR.movel(target_pos, vel=VELOCITY*0.8, acc=ACCURACY)
        DR.move_periodic(
            amp=[turning_radius, turning_radius, 0, 0, 0, 0],
            period=[2, 3, 0, 0, 0, 0],
            repeat=stir_repeat,
            ref=DR.DR_BASE
            )

    def grasp(self, x):
        self._set_custom_grasp(x)
        DR.set_digital_output(1, ON)
        time.sleep(0.5)

    def release(self, x):
        self._set_custom_grasp(x)
        DR.set_digital_output(1, OFF)
        time.sleep(0.5)

    def _set_custom_grasp(self, x):
        if x == 0:
            DR.set_digital_output(2, OFF)
        elif x == 1:
            DR.set_digital_output(2, ON)
        
