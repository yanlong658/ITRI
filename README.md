# ITRI 工研院計畫

## System A
### 事前安裝軟體
1. opencv(3.x version)
2. Eigen
3. ros機器人作業系統

---
### 啟動
1. 建立起自己的work space
2. roslaunch itri featureMatch.launch 

---
### launch檔裡面跑 
1. try.cpp
2. parameter.cpp
3. process.cpp

---
### 要修改的參數
process.h 
1. camera參數

try.h 
1. camera參數
2. 外部參數R和t 要特別注意在哪個frame上面

## System B
---
### OPtitrack
1. roslaunch itri_node run.launch

---
參考一
ROS tutorial 安裝:http://wiki.ros.org/ROS/Tutorials

參考二
相機內參校正:https://www.youtube.com/watch?v=f6p2Cx_bXoE&feature=youtu.be&ab_channel=%E9%A1%8F%E9%9A%86
