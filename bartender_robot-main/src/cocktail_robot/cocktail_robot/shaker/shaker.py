# 예시 코드
from ..utils.base_action import BaseAction
import DR_init
import time
ON, OFF = 1, 0
DR = None
VEL, ACC = 100, 100
class ShakerAction(BaseAction):
    def __init__(self, node, poses):
        DR_init.__dsr__node = node
        self.poses = poses
        global DR
        try:
            import DSR_ROBOT2 as DR
        except ImportError as e:
            print(f"Error importing DSR_ROBOT2 : {e}")
            return

    def execute(self):
        pick_poses = self.poses['pick']
        pick_before_poses = self.poses['pick_before']
        shaking_poses = self.poses['shaking']
        place_before_poses = self.poses["place_before"]
        # 1. 잡는 위치로 이동
        self.release()
        DR.movej([0,0,90,0,90,0],vel=VEL,acc=ACC)
        DR.movel(pick_before_poses['task'],vel=VEL,acc=ACC)
        DR.movel(pick_poses['task'],vel=VEL,acc=ACC)
        self.grasp()

        DR.movel(place_before_poses['task'],vel=VEL,acc=ACC)
        DR.movel(shaking_poses['task'],vel=VEL,acc=ACC,ref=DR.DR_BASE)
    
        pos1 = DR.get_current_posj().copy()
        qlist = [DR.posj(20.28,-6.19,55.16,12.27,38.84,-166.27),
                 DR.posj(18.92,-8.32,81.26,6.07,44.71,-158.54),
                 DR.posj(-36.67,-8.15,58.66,12.45,34.93,-24.48),
                 DR.posj(-16.30,-10.84,94.22,12.44,36.81,-61.21),
                 DR.posj(pos1)]
        
        for _ in range(4):
            DR.movesj(qlist,vel=250,acc=150)

        # DR.move_periodic([50, 50, 20, 0, 20, 15], [1,2.5, 1.5, 0, 1.5, 2],repeat=10,atime=0.1,ref=DR.DR_TOOL)
        # 2. 충돌 방지 경로
        for _ in range(4):
            moving_pose = [0,60,40,0,35,0]
            moving_pose1 = [0,-60,-40,0,-35,0]
            DR.amovej(moving_pose1,vel=250,acc=120,mod=DR.DR_MV_MOD_REL)
            DR.wait(0.45)
            DR.amovej(moving_pose,vel=120,acc=120,mod=DR.DR_MV_MOD_REL) 
            DR.wait(0.45)
        # 옆으로 흔들기
        # for _ in range(4):
        #     DR.movejx(shaking_poses['task'],vel=VEL,acc=ACC,ref=DR.DR_BASE,sol=7)
        #     DR.movejx(shaking_poses['task'],vel=VEL,acc=ACC,ref=DR.DR_BASE,sol=2)
        
        DR.movel(shaking_poses['task'],vel=VEL,acc=ACC)
        DR.movel(place_before_poses['task'],vel=VEL,acc=ACC)
        # 3. 내려놓기
        DR.movel(pick_poses['task'],vel=VEL,acc=ACC)
        self.release()
        DR.movel(pick_before_poses['task'],vel=VEL,acc=ACC)
        DR.movel(place_before_poses['task'],vel=VEL,acc=ACC)
        home_pose = place_before_poses['task'].copy()
        home_pose[3:] = [0,-180,0]
        DR.movel(home_pose,vel=VEL,acc=ACC)
        DR.movej([0,0,90,0,90,0],vel=VEL,acc=ACC)
        
        
    def grasp(self):
        DR.set_digital_output(1,ON)
        DR.set_digital_output(2,OFF)
        DR.wait(1)

    def release(self):
        DR.set_digital_output(1,OFF)
        DR.set_digital_output(2,OFF)
        DR.wait(1)

        