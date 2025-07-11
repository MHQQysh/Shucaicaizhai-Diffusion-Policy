

conda activate robodiff

cd diffusion_policy/


# 1


python train.py --config-dir=. --config-name=image_pusht_diffusion_policy_cnn.yaml training.seed=42 training.device=cuda:0 hydra.run.dir='data/outputs/${now:%Y.%m.%d}/${now:%H.%M.%S}_${name}_${task_name}'


# 2

python eval.py --checkpoint data/0550-test_mean_score=0.969.ckpt --output_dir data/pusht_eval_output --device cuda:0



# 3

在总文件下面



自己纠错
HYDRA_FULL_ERROR=1 python collect_data.py --config-name image_ur5_grab_diffusion_policy_cnn --config-dir=.

需要
pip install spatialmath
注释 # import mujoco_viewer
之类


# 4

仿真看位置


这里我写了一个
./sim.sh imitation_learning_dp-master/imitation_learning_dp/assets/scenes/scene.xml




# 5