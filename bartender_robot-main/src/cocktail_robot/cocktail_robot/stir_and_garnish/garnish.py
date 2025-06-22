# pick and place in 1 method. from pos1 to pos2 @20241104
import time
import rclpy
import DR_init
from ..utils.base_action import BaseAction


ON, OFF = 1, 0
VELOCITY, ACCURACY = 100, 60
### 힘 제어 : BASE 좌표계 기준
class GarnishAction(BaseAction):
    def __init__(self, node, poses, topping):
        DR_init.__dsr__node = node

        try:
            import DSR_ROBOT2

        except ImportError as e:
            print(f"Error importing DSR_ROBOT2 : {e}")
            return
        
        global DR
        DR = DSR_ROBOT2

        self.garnish_pose = poses
        self.grasp_option = 1
        self.topping = topping

    def execute(self):
        # uncomment if task ready position is needed
        DR.movej(self.garnish_pose["garnish_ready"]["joint"], vel=VELOCITY*0.3, acc=ACCURACY)

        # DR.movel(self.garnish_pose[f"{self.topping}_0"]["task"], vel=VELOCITY, acc=ACCURACY)
        topping_up = self.garnish_pose[f"{self.topping}"]["task"].copy()
        topping_up[2] += 33
        DR.movel(topping_up, vel=VELOCITY, acc=ACCURACY)
        DR.movel(self.garnish_pose[f"{self.topping}"]["task"], vel=VELOCITY, acc=ACCURACY)

        self.grasp(self.grasp_option)
        time.sleep(0.8)
        DR.movel(self.garnish_pose["garnish_drop_ready"]["task"], vel=VELOCITY, acc=ACCURACY)
        DR.movel(self.garnish_pose["garnish_drop"]["task"], vel=VELOCITY, acc=ACCURACY)
        self.release(self.grasp_option)

        # uncomment if task ready position is needed
        # self.movel(self.garnish_pose["garnish_ready"]["task"], vel=VELOCITY, acc=ACCURACY)

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