import mujoco

import numpy as np

import matplotlib.pyplot as plt

import mujoco.viewer

# import mujoco_viewer

import mujoco

import time

# 加载 MJCF 文件

model = mujoco.MjModel.from_xml_path("scene.xml")

data = mujoco.MjData(model)




# 创建一个窗口

viewer = mujoco.viewer.launch(model, data)


# 运行仿真

while True:

    mujoco.mj_step(model, data)


    time.sleep(1)

    viewer.render()

    if viewer.is_closed:

        break


# 关闭窗口

viewer.close()