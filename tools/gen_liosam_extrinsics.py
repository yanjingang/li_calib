#!/usr/bin/python3

# Cmd:  python3 tools/gen_liosam_extrinsics.py ~/catkin_calib/pandar40p-h30.0/calib_result.csv

import os
import sys
import numpy as np

# 1.读取标定外参
filename = '/home/work/catkin_calib/pandar40p-h30.0/calib_result.csv'
if len(sys.argv) >= 2:
    filename = sys.argv[1]
file = open(filename, 'r')
calib_result = file.readlines()
file.close()

# 2.计算均值
count = 0
P_IinL = {'x':0.0, 'y':0.0, 'z':0.0}            # P_IinL.x、P_IinL.y、P_IinL.z，以激光雷达为参考坐标系时，IMU的三轴坐标，组成了雷达到IMU的平移向量；
q_ItoL = {'x':0.0, 'y':0.0, 'z':0.0, 'w':0.0}   # q_ItoL.x、q_ItoL.y、q_ItoL.z、q_ItoL.w，以激光雷达为参考坐标系时，IMU的三轴旋转四元数，可后续转化为旋转矩阵。
cols = {}   # 表头
lastExtrinsic = ''
print('p_IinL.x p_IinL.y p_IinL.z q_ItoL.x q_ItoL.y q_ItoL q_ItoL.w')
for row in range(len(calib_result)):
    if row != 0 and row < len(calib_result) * 3.3 / 10:  # 前1/3的数据不精准，丢掉（保留表头）
        continue
    val = calib_result[row].split(",")
    if len(val) < 2:
        continue
    if val[0] == 'bag_path':
        i = 0
        for name in val:
            cols[name] = i
            i += 1
        # print('Header: ', cols)
        continue
    extrinsic = val[cols['p_IinL.x']] + ',' + val[cols['p_IinL.y']] + ',' + val[cols['p_IinL.z']] + ',' + val[cols['q_ItoL.x']] + ',' + val[cols['q_ItoL.y']] + ',' + val[cols['q_ItoL']] + ',' + val[cols['q_ItoL.w']]
    if lastExtrinsic != extrinsic:    # 只有连续Refinement迭代优化的结果完全一样时，标定结果才是可用的
        lastExtrinsic = extrinsic
        continue
    print(extrinsic)
    P_IinL['x'] += float(val[cols['p_IinL.x']])
    P_IinL['y'] += float(val[cols['p_IinL.y']])
    P_IinL['z'] += float(val[cols['p_IinL.z']])
    q_ItoL['x'] += float(val[cols['q_ItoL.x']])
    q_ItoL['y'] += float(val[cols['q_ItoL.y']])
    q_ItoL['z'] += float(val[cols['q_ItoL']])
    q_ItoL['w'] += float(val[cols['q_ItoL.w']])
    count += 1
# print('Sum: ', count, P_IinL, q_ItoL)
if count == 0:
    print("Res: not found good extrinsic!")
    exit
# avg
P_IinL['x'] = round(P_IinL['x'] / count, 8)
P_IinL['y'] = round(P_IinL['y'] / count, 8)
P_IinL['z'] = round(P_IinL['z'] / count, 8)
q_ItoL['x'] = round(q_ItoL['x'] / count, 8)
q_ItoL['y'] = round(q_ItoL['y'] / count, 8)
q_ItoL['z'] = round(q_ItoL['z'] / count, 8)
q_ItoL['w'] = round(q_ItoL['w'] / count, 8)
print('Avg: ', count, ' / ', len(calib_result))
print(P_IinL['x'], P_IinL['y'], P_IinL['z'], q_ItoL['x'], q_ItoL['y'], q_ItoL['z'], q_ItoL['w'])


# 3. 将四元数转化为旋转矩阵
# 输入四元数的值，xyzw分别代表q_ItoL.x，q_ItoL.y，q_ItoL.z，q_ItoL.w
x = q_ItoL['x']
y = q_ItoL['y']
z = q_ItoL['z']
w = q_ItoL['w']
# 计算旋转矩阵
R_IMU_to_LiDAR = np.array([
    [1 - 2*y*y - 2*z*z, 2*x*y - 2*w*z, 2*x*z + 2*w*y],
    [2*x*y + 2*w*z, 1 - 2*x*x - 2*z*z, 2*y*z - 2*w*x],
    [2*x*z - 2*w*y, 2*y*z + 2*w*x, 1 - 2*x*x - 2*y*y]
])
print("IMU_to_LiDAR:", R_IMU_to_LiDAR)


# 4.生成LIO-SAM配置参数
print("\nExtrinsics (lidar -> IMU):")
print("extrinsicTrans: ", [P_IinL['x'], P_IinL['y'], P_IinL['z']] )
print("extrinsicRot: ", R_IMU_to_LiDAR )
print("extrinsicRPY: ", R_IMU_to_LiDAR )
