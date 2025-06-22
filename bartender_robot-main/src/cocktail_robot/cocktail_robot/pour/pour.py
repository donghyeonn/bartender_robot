import time
import rclpy
import DR_init
from ..utils.base_action import BaseAction


VELOCITY, ACCURACY = 100, 60
ON, OFF = 1, 0
DR = None

# PourAction(arm, "tequila", 50, shaker, poses["pour_tequila"])

class PourAction:
    def __init__(self, node, poses, ingredient, amount, target):
        DR_init.__dsr__node = node

        try:
            import DSR_ROBOT2

        except ImportError as e:
            print(f"Error importing DSR_ROBOT2 : {e}")
            return

        global DR
        DR = DSR_ROBOT2
        
        self.grasp_option = 0
        self.ingredient = ingredient
        self.amount = amount
        self.target = target
        self.pour_pose = poses


    def execute(self):
        pick_before = self.pour_pose[self.ingredient]["task"].copy()
        pick_after = self.pour_pose[self.ingredient]["task"].copy()
        shaker_pick_after = self.pour_pose['shaker_pick']['task'].copy()
        pick_before[1] = self.pour_pose[self.ingredient]["task"][1] - 30
        pick_after[2] = self.pour_pose[self.ingredient]["task"][2] + 100
        shaker_pick_after[2]=self.pour_pose["shaker_pick"]["task"][2] + 300


        # set position
        # DR.movej([0,0,90,0,90,0], vel=VELOCITY*0.3, acc = ACCURACY)
        # DR.movej(self.pour_pose["pour_ready"]["joint"], vel=VELOCITY, acc = ACCURACY) #시작점
        self.release(self.grasp_option)

        # pick
        if self.ingredient == "shaker_":
            DR.movel(self.pour_pose['shaker_pick_before']['task'],vel=VELOCITY,acc=ACCURACY)
            DR.movel(self.pour_pose['shaker_pick']['task'],vel=VELOCITY,acc=ACCURACY)
            self.grasp(self.grasp_option)
            DR.movel(shaker_pick_after, vel=VELOCITY, acc = ACCURACY)

            # pour
            DR.movel(self.pour_pose["shaker_glass"]["ready"]["task"], vel=VELOCITY, acc = ACCURACY)
            DR.movel(self.pour_pose["shaker_glass"]["start"]["task"], vel=VELOCITY*0.15, acc = ACCURACY)
            time.sleep(1.5)
            DR.movel(self.pour_pose["shaker_glass"]["ready"]["task"], vel=VELOCITY, acc = ACCURACY)

            # go to place
            DR.movel(shaker_pick_after, vel=VELOCITY, acc = ACCURACY)
            DR.movel(self.pour_pose['shaker_pick']['task'],vel=VELOCITY,acc=ACCURACY)
            self.release(self.grasp_option)
            DR.movel(self.pour_pose['shaker_pick_before']['task'],vel=VELOCITY,acc=ACCURACY)
            DR.movel(shaker_pick_after, vel=VELOCITY, acc = ACCURACY)
            DR.movej([0,0,90,0,90,0], vel=VELOCITY*0.3, acc=ACCURACY)


        else:
            DR.movejx(self.pour_pose["pick_back"]["task"], vel=VELOCITY*0.4, acc = ACCURACY,sol=7)

            DR.movel(pick_before, vel=VELOCITY, acc=ACCURACY)
            DR.movel(self.pour_pose[self.ingredient]["task"], vel=VELOCITY, acc = ACCURACY)
            self.grasp(self.grasp_option)
            DR.movel(pick_before, vel=VELOCITY, acc=ACCURACY)
            DR.movejx(self.pour_pose["pick_back"]["task"], vel=VELOCITY*0.3, acc = ACCURACY,sol=7)
            DR.movel(self.pour_pose["pick3"]["task"], vel=VELOCITY, acc = ACCURACY)

            # pour
            DR.movel(self.pour_pose[self.target]["ready"]["task"], vel=VELOCITY, acc = ACCURACY)
            DR.movel(self.pour_pose[self.target]["start"]["task"], vel=VELOCITY*0.15, acc = ACCURACY)
            time.sleep(1.5)
            DR.movel(self.pour_pose[self.target]["ready"]["task"], vel=VELOCITY, acc = ACCURACY)

            # go to place
            DR.movel(self.pour_pose["pick3"]["task"], vel=VELOCITY, acc = ACCURACY)
            DR.movejx(self.pour_pose["pick_back"]["task"], vel=VELOCITY*0.3, acc = ACCURACY,sol=7)
            DR.movel(pick_before,vel=VELOCITY,acc=ACCURACY)
            DR.movel(self.pour_pose[self.ingredient]["task"], vel=VELOCITY, acc = ACCURACY)
            self.release(self.grasp_option)
            DR.movel(pick_before,vel=VELOCITY,acc=ACCURACY)


        # end
        # DR.movej([0,0,90,0,90,0], vel=VELOCITY*0.3, acc = ACCURACY)



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



