#!/bin/bash

# 检查是否提供了两个参数
if [ "$#" -ne 2 ]; then
    echo "使用方法: ./add_numbers.sh <数字1> <数字2>"
    exit 1
fi

# 获取命令行参数
NUM1="$1"
NUM2="$2"

# 使用 here document 将 Python 代码传递给 python 解释器
# 注意：我们将 Bash 变量 $NUM1 和 $NUM2 嵌入到 Python 字符串中
python3 <<EOF
try:
    # Python 直接使用 Bash 传递的变量值
    num1 = float("$NUM1")
    num2 = float("$NUM2")
    sum_result = num1 + num2
    print(f"这两个数字的和是: {sum_result}")
except ValueError:
    print("错误: 输入的必须是有效的数字。")
    # 使用 exit() 在 Python 脚本中退出，与 sys.exit() 效果类似
    exit(1)
EOF