#!/bin/bash

# 检查是否提供了文件路径
if [ -z "$1" ]; then
  echo "Usage: $0 <path_to_mujoco_xml_file>"
  exit 1
fi

XML_FILE="$1"

# 检查文件是否存在
if [ ! -f "$XML_FILE" ]; then
  echo "Error: XML file not found at '$XML_FILE'"
  exit 1
fi

# 确保你的 Python 虚拟环境被激活 (如果需要)
# 如果你的 MuJoCo 环境在 'robodiff' 虚拟环境中，请取消注释下一行并修改路径
source "$HOME/miniconda3/etc/profile.d/conda.sh"
conda activate robodiff

# 打印正在运行的文件
echo "Launching MuJoCo simulation for: $XML_FILE"

# 使用 heredoc 方式构建一个临时的 Python 脚本并执行
python3 -c "
import mujoco
import mujoco.viewer
import time
import os

xml_path = os.path.abspath('$XML_FILE') # 获取绝对路径，确保文件能被找到

try:
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)
    viewer = mujoco.viewer.launch(model, data)

    while viewer.is_running(): # 使用 is_running() 检查 viewer 状态
        mujoco.mj_step(model, data)
        # time.sleep(0.002) # 可以适当减小，让模拟运行更快，或者完全移除让其以最大速度运行
        viewer.sync() # 同步 viewer 显示

except Exception as e:
    print(f'An error occurred: {e}')
    print(f'Please ensure MuJoCo is installed and configured, and the XML file is valid.')
finally:
    if 'viewer' in locals() and viewer.is_running():
        viewer.close() # 确保关闭 viewer
"

# 如果你激活了虚拟环境，可以在这里停用它
# deactivate