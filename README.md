# ITRI 工研院計畫
---
My work
1. 安裝linux and ROS機器人作業系統
2. 建立起自己的work space
3. roslaunch itri featureMatch.launch 

launch檔裡面跑 
1. try.cpp
2. parameter.cpp
3. process.cpp
---
OPtitrack
1. roslaunch itri_node run.launch
---
參考一
ROS tutorial 安裝:http://wiki.ros.org/ROS/Tutorials

---要修改的參數
process.h 
1. camera參數

try.h 
1. camera參數
2. 外部參數R和t 要特別注意在哪個frame上面

---
參考二
相機內參校正:https://www.youtube.com/watch?v=f6p2Cx_bXoE&feature=youtu.be&ab_channel=%E9%A1%8F%E9%9A%86
