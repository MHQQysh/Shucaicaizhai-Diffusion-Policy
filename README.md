



# 蔬菜采摘机器人-总体步骤讲解


1. 首先是自己采集数据，自己动手，然后读取了这个过程中机械臂和摄像头，然后数据会收集在data里面,
2. 然后把收集的数据仿真出来，是用的imitation_learning_dp的collect_data
3. 然后是diffusion_policy中的train.py，这里是算法的核心，包括数据的预处理，然后diffusion不同算法的使用，和最终的输出形式。   我们采用服务器的方式，训练得到一个ckpt
4. 然后ckpt放到推理里面直接测试eval.py



# 分步脚本


打开conda环境

`conda activate robodiff`

`cd imitation_learining_dp-master`


### 1. 收集数据



放到diffusion_policy下面的data里面


### 2. 仿真收集的数据




这里环境配置的时候缺少一些库，需要自己
pip install spatialmath
注释 # import mujoco_viewer
之类

`HYDRA_FULL_ERROR=1 python collect_data.py --config-name image_ur5_grab_diffusion_policy_cnn --config-dir=.`

这个是我用来帮助看自己的机械臂xml文件是否正确的一个脚本，可以在majoco中control link 


`./sim.sh imitation_learning_dp-master/imitation_learning_dp/assets/scenes/scene.xml`

### 3. 训练

输入: data

输出：

`python train.py --config-dir=. --config-name=image_pusht_diffusion_policy_cnn.yaml training.seed=42 training.device=cuda:0 hydra.run.dir='data/outputs/${now:%Y.%m.%d}/${now:%H.%M.%S}_${name}_${task_name}'`


### 4. 推理


`python eval.py --checkpoint data/0550-test_mean_score=0.969.ckpt --output_dir data/pusht_eval_output --device cuda:0`



