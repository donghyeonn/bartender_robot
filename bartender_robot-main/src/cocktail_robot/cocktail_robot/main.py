import threading
import time
import rclpy
from rclpy.node import Node
import DR_init
import os, yaml
import numpy as np

from .pour.pour import PourAction
from .shaker.shaker import ShakerAction
from .stir_and_garnish.stir import StirAction
from .stir_and_garnish.garnish import GarnishAction
from .tumbler.tumbler import TumblerAction
from ament_index_python.packages import get_package_share_directory
from .pour.pour import PourAction
from .bartender_gui import BartenderGUI

POSE_PATH = os.path.join(
    get_package_share_directory("cocktail_robot"),
    "pose.yaml"
)

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60

ON, OFF = 1, 0

def load_yaml(POSE_PATH):
    with open(POSE_PATH, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    return data

def get_recipes(node, poses):
    return {
        'Margarita': [
            PourAction(node, poses=poses["pour"], ingredient="tequila", amount=50, target="shaker"), # tequila -> shaker
            PourAction(node, poses=poses["pour"], ingredient="blue_juice", amount=20, target="shaker"), # blue_juice -> shaker
            TumblerAction(node, poses=poses["tumbler"], move="close"), # close
            ShakerAction(node, poses=poses["shake"]), # shake
            TumblerAction(node, poses=poses["tumbler"], move="open"), # open
            PourAction(node, poses=poses["pour"], ingredient="shaker_", amount=80, target="glass"), # shaker -> glass
            GarnishAction(node, poses=poses["garnish"], topping="lime")
        ],
        'China Red': [
            PourAction(node, poses=poses["pour"], ingredient="tequila", amount=50, target="glass"),
            PourAction(node, poses=poses["pour"], ingredient="blue_juice", amount=30, target="glass"),
            StirAction(node, poses['stir']), # stir
            GarnishAction(node, poses=poses["garnish"], topping="cherry")
        ]
    }

def recursive_check(data_dict):
    if isinstance(data_dict, dict):
        for key, value in data_dict.items():
            recursive_check(value)
    elif isinstance(data_dict, list):
        if len(data_dict) == 6 and all(isinstance(v, (int, float)) for v in data_dict):
            arr = np.array(data_dict, dtype=np.float64)
            print(f"float64[6] OK: {arr}")
        elif len(data_dict) == 6:
            raise TypeError(f"6개인데 float/int 아님: {data_dict}")
        elif all(isinstance(v, (int, float)) for v in data_dict):
            raise IndexError(f"값 개수 오류 ({len(data_dict)}개): {data_dict}")
        else:
            raise TypeError(f"값 개수와 타입 모두 문제: {data_dict}")


def main():
    rclpy.init()
    node = rclpy.create_node("main", namespace=ROBOT_ID)
    DR_init.__dsr__node = node

    try:
        from DSR_ROBOT2 import (
            movej,
            set_digital_output,
            set_tool,
            set_tcp,
            set_ref_coord,
            DR_BASE
        )

    except ImportError as e:
        print(f"Error importing DSR_ROBOT2 : {e}")
        return
    
    set_tool("GripperDA_v2")
    set_tcp("Tool Weighttest")
    set_ref_coord(DR_BASE)

    poses = load_yaml(POSE_PATH)
    recursive_check(poses)
    recipes = get_recipes(node, poses)
    gui_recipes = ['표지'] + list(recipes.keys())

    # ================== 동작 관리 ==================
    stop_flag = threading.Event()
    action_thread = None

    def robot_action_callback(recipe_name):
        nonlocal action_thread
        if recipe_name == "STOP":
            app.set_status_msg("중단 요청!")
            stop_flag.set()
            return

        if action_thread and action_thread.is_alive():
            app.set_status_msg("이미 동작중! 중단 후 재실행하세요.")
            return

        def run_actions():
            movej([0,0,90,0,90,0], vel=VELOCITY, acc=ACC)
            release(0)
            app.set_status_msg(f"[{recipe_name}] 제조 시작!")
            stop_flag.clear()
            for idx, action in enumerate(recipes[recipe_name], 1):
                if stop_flag.is_set():
                    app.set_status_msg(f"[{recipe_name}] → 동작 중단됨!")
                    
                    break
                ing = getattr(action, 'ingredient', '-')
                stepname = action.__class__.__name__
                app.set_status_msg(f" - Step {idx}: [{ing}] [{stepname}] 실행 중.")
                action.execute()
                time.sleep(0.2)
            else:
                app.set_status_msg(f"[{recipe_name}] 제조 완료!")
            movej([0,0,90,0,90,0], vel=VELOCITY, acc=ACC)
            release(0)
        action_thread = threading.Thread(target=run_actions, daemon=True)
        action_thread.start()

        def release(x):
            _set_custom_grasp(x)
            set_digital_output(1, OFF)
            time.sleep(0.5)

        def _set_custom_grasp(x):
            if x == 0:
                set_digital_output(2, OFF)
            elif x == 1:
                set_digital_output(2, ON)
            

    app = BartenderGUI(gui_recipes, recipes, robot_action_callback=robot_action_callback)
    app.mainloop()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
