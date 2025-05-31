# 一款基于达妙电机和低成本钣金结构件的六轴机械臂

项目初衷是想帮助更多想玩动力学的学生党能玩上低成本的机械臂

有很多同学或RMer实验室有达妙电机，所以方案选择达妙电机，同学们只用出两百五左右的钣金和一些零零碎碎的材料钱就可以了。

基本功能演示视频：https://www.bilibili.com/video/BV1w6dBYxEPu?vd_source=9fd17ff99dc79491d23776f35bfbd4e1

这个仓库仅用于开源最原始的panther机械臂，仅有ros-noetic中一些基本功能：重力补偿模式，拖动示教，基于action通信的moveit轨迹规划
（功能包中的catesian_plan.cpp是调用moveit在笛卡尔空间下画圆，可能有点问题需要自己尝试）

## 相关说明
Panther名字由来：原始设计整机为黑色所以命名为黑豹（Panther），而Panthera是豹类，代表整个Panther机械臂开源项目，希望大家能加入Panthera项目中，创造相对更低成本、性能更好的关节电机方案机械臂，同时实现更多好玩的功能。

## Panthera
后续我们会将Panther从ros中迁移至纯Python环境下来控制

Mujoco仿真及数字孪生

Pinocchio库的使用

diffusion policy模仿学习

强化学习相关内容

跟宇树go2做协同控制

......

后续一些内容的更新在 **KelvinLauMiau(亲爱的小一咯)** 的Ragtime_Panthera仓库中：
https://github.com/KelvinLauMiau/Ragtime_Panthera

