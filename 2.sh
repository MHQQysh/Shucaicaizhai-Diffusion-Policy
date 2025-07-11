if [ -f "$HOME/miniconda3/etc/profile.d/conda.sh" ]; then
    source "$HOME/miniconda3/etc/profile.d/conda.sh"
    echo '1'
conda activate robodiff
echo '2'

elif [ -f "/opt/miniconda3/etc/profile.d/conda.sh" ]; then
    source "/opt/miniconda3/etc/profile.d/conda.sh"
    echo '3'

else
    echo "警告: 未找到 Conda 初始化脚本。请确保 Miniconda 已正确安装并配置 PATH，或手动指定其路径。"
fi