# PoseEstimationFromLines
基于直线对应的刚体目标的位姿估计，仅需要依赖Opencv2以上的第三方库。
由OSG生成的目标的序列图，包含在文件Rotation_New文件夹的Sequences中,Pla和Cam文件夹是位姿真值。在Sequences中有Model_bake.model文件。
目前在I7-6700处理器，Release模式下，位姿跟踪速率达到25fps左右，同时精确度较高。
#TODO
目前在ubuntu系统下有段错误，Windows系统可正确运行。望积极找Bug。
#TODO
1.改进直线匹配的方法
2.加入新的基于直线的位姿估计算法，如ASPNL+Ransac
