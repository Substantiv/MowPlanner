该仓库包含了面向差速机器人爬坡轨迹优化问题的项目代码、技术文档和使用说明，涵盖了爬坡轨迹优化的核心算法实现。

## 1. 四轮差速机器人gazebo建模及爬坡问题建模

*```willand_description```：主要用于搭建Gazebo环境中的车辆模型和环境模型，并处理里程计信息。车辆模型中包含两种传感器：IMU,二维激光雷达和Intel D435深度相机，但控制过程中用到的里程计信息是Gazebo中模型的位置和姿态。此外，"RobotWrench"类建立了爬坡相关的轨迹优化项，相关见support_file文件夹中的"打滑相关优化建模"文件。Gazebo读取接触点的力和力矩方法见support_file文件夹中的"读取Gazebo接触点的力&力矩.md".

## 2. 四轮差速非线性模型预测控制

安装MPC控制器的优化库依赖包

```
pip3 install casadi  
```

该部分在 ```mpc_follower``` 功能包中，使用MPC建立的planner和controller；

## 3. 轨迹优化

轨迹优化相关文件在"path_planner"功能包中，路径发布在path_pub.py文件中，轨迹优化在ALMTrajOpt对应类中，主要思路为使用拉格朗日增广法将带约束的优化问题转换为无约束的优化问题，然后使用L-BFGS算法进行求解。相关理论说明见"无约束优化问题.md"和"约束优化问题.md"。


