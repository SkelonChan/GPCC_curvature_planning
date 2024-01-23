# !/usr/bin/env python
# -*-coding:utf-8 -*-
# @Time    : 2024/01/11 11:09
# @Author  : Skelon_Chan
# @File    : kappa_test_2.py
# @Version : python3.9
# @Desc    : $END$
import math
import kappa_planning_base as ka
import numpy as np
import copy
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import bi_tree as bt
from matplotlib.path import Path
from scipy.spatial.distance import euclidean
import cv2

# init = np.array([-3, 0])
# goal = np.array([3, 2])
init = np.array([-4, -2])
goal = np.array([4, 2])
get_pic = True
pic_count = 0
dt = 0.01
ka_zero = 0.0001
back_obs = 0.8  # 碰到障碍物后回退的弧长ds
add_theta = 0.1  # 纯曲线部分更改
scan_len = 0.5 #扫描到障碍物后，碰撞点前移距离
last_check = False  #
path_end = None #用于存储最后一段到达终点的路径

theta_init = 0  # 初始的行进方向与横轴的夹角
x_value_p = np.array([init[0]])
y_value_p = np.array([init[1]])
theta_p = np.array([theta_init])
# plt.ion()
# 绘制环境
fig, ax = plt.subplots()
fig.set_dpi(200)
ax.set_xlim(-5, 5)
ax.set_ylim(-5, 5)
ax.set_aspect('equal')
# ax.autoscale(enable=True, axis='both', tight=True)
a1 = [(-3,0.25),(3,0.25),(3,5),(-3,5),(-3,0.25)]
a2 = [(-3,-0.25),(-3,-5),(3,-5),(3,-0.25),(-3,-0.25)]
gen_obs_ = [a1, a2]
OBS = ka.gen_obs(gen_obs_, ax)
plt.scatter(init[0], init[1], marker='o', color='lime')
plt.scatter(goal[0], goal[1], marker='o', color='red')
# plt.show()

if get_pic:
    pic_count = pic_count + 1
    filename = 'temp_image_' + str(pic_count) + '.png'  # 生成唯一的文件名
    plt.savefig(filename, dpi=300)

theta_init_goal = np.arctan2(goal[1] - init[1], goal[0] - init[0])
# 构建二叉树以及顺序字典
Bi_tree = bt.BinaryTree()
Order_dict = bt.OrderedKeyValueStructure()

# 得到的初始曲率解系
ka_ = ka.kappa_init(init, goal, theta=theta_init)
kappa_se = ka_
# 积分得到轨迹
path_first = ka.integer_kappa(init[0], init[1], theta_init=theta_init, kappa_seq=kappa_se)
# 检测得到障碍物点的索引
plt.plot(path_first.transpose()[0], path_first.transpose()[1], color='gray')

if get_pic:
    pic_count = pic_count + 1
    filename = 'temp_image_' + str(pic_count) + '.png'  # 生成唯一的文件名
    plt.savefig(filename, dpi=300)
obs_idx = ka.obs_detect(path_first.transpose()[0], path_first.transpose()[1],OBS)
obs_line_len = len(path_first[:, 0])
re_back_step = math.ceil(obs_idx - back_obs / dt)  # 回退索引

obs_idx_first = obs_idx
if obs_idx_first is not None:
    re_back_point = np.array([path_first[re_back_step, 0], path_first[re_back_step, 1]])
    weight = euclidean(re_back_point, goal)
    # 定义二叉树的根节点
    node_root = bt.In_tree_value(weight, path_first[:re_back_step, :])
    # 二叉树根节点插入
    Bi_tree.add_root_node(copy.deepcopy(node_root))
    # 将根节点的信息加入顺序字典,权重和定位序列
    Order_dict.add_item(weight, [])
    path_first = path_first[:re_back_step, :]
else:
    path = path_first
# test_time = 3
while obs_idx_first is not None:
    # for i in range(4):
    add_theta_tmp = add_theta
    ccw_obs_len = obs_line_len
    cw_obs_len = obs_line_len
    ccw_check = cw_check = True
    obs_idx_near_cw = obs_idx_near_ccw = None
    obs_idx_ccw = 1
    obs_idx_cw = obs_idx_ccw
    path_tmp_ccw = path_tmp_cw = None
    # 回退点在二叉树中的定位序列
    re_back_point = None
    # 最后测试是否一步直达重点的temp path
    path_check_end = None
    re_back_seq = Order_dict.get_min_key_value()[1]
    re_back_node = Bi_tree.get_node(re_back_seq)
    if re_back_node.left is not None:
        ccw_check = False
    if re_back_node.right is not None:
        cw_check = False
    re_back_path = Bi_tree.get_tree_path_sque(re_back_seq)
    plt.scatter(re_back_path[-1,0], re_back_path[-1,1], color='dodgerblue')
    if get_pic:
        pic_count = pic_count + 1
        filename = 'temp_image_' + str(pic_count) + '.png'  # 生成唯一的文件名
        plt.savefig(filename, dpi=300)
    ka_line = []
    while True:
        # 默认碰撞节点都是存在的
        obs_idx_ccw_old = obs_idx_ccw
        obs_idx_cw_old = obs_idx_cw
        path_tmp_ccw_old = path_tmp_ccw
        path_tmp_cw_old = path_tmp_cw
        if ccw_check:
            # 逆时针曲线+直线路径生成（无避碰）
            ka_circle_ccw = ka.circle_update_ccw(re_back_path[-1, 3], re_back_path[-1, 2], add_theta_tmp)
            # 曲线后直线部分延申
            if int(ccw_obs_len) > len(ka_circle_ccw):
                ka_line = np.ones(ccw_obs_len - len(ka_circle_ccw)) * ka_zero
            else:
                ka_line = np.ones(1) * ka_zero
            ka_com_ccw = np.concatenate((ka_circle_ccw, ka_line))
            path_tmp_ccw = ka.integer_kappa(re_back_path[-1, 0], re_back_path[-1, 1], re_back_path[-1, 2], kappa_seq=ka_com_ccw)
            plt.plot(path_tmp_ccw.transpose()[0], path_tmp_ccw.transpose()[1], color='gray', linestyle='--', linewidth=0.5)
            if get_pic:
                pic_count = pic_count + 1
                filename = 'temp_image_' + str(pic_count) + '.png'  # 生成唯一的文件名
                plt.savefig(filename, dpi=300)
            # 对逆时针更新的曲线检查是否存在碰撞
            obs_idx_ccw = ka.obs_detect(x=path_tmp_ccw.transpose()[0], y=path_tmp_ccw.transpose()[1], obs = OBS)
            if obs_idx_ccw:
                ccw_obs_len = obs_idx_ccw + int(scan_len/dt) # 前移0.5弧度长
        if cw_check:
            # 逆时针发生了碰撞，则检测顺时针情况 TODO
            # if obs_idx_ccw is not None or not ccw_check:
                # 顺时针曲线+直线路径生成（无避碰）
            ka_circle_cw = ka.circle_update_cw(re_back_path[-1, 3], re_back_path[-1, 2], add_theta_tmp)
            if int(cw_obs_len) > len(ka_circle_cw):
                ka_line = np.ones(cw_obs_len - len(ka_circle_cw)) * ka_zero
            else:
                ka_line = np.ones(1) * ka_zero
            ka_com_cw = np.concatenate((ka_circle_cw, ka_line))
            path_tmp_cw = ka.integer_kappa(re_back_path[-1, 0], re_back_path[-1, 1], re_back_path[-1, 2], kappa_seq = ka_com_cw)
            # 顺时针细分，无法绘出，满0.1才找到
            plt.plot(path_tmp_cw.transpose()[0], path_tmp_cw.transpose()[1], color='gray', linestyle='--', linewidth=0.5)
            if get_pic:
                pic_count = pic_count + 1
                filename = 'temp_image_' + str(pic_count) + '.png'  # 生成唯一的文件名
                plt.savefig(filename, dpi=300)
            #     对顺时针更新的曲线进行检测是否存在碰撞
            obs_idx_cw = ka.obs_detect(x=path_tmp_cw.transpose()[0], y=path_tmp_cw.transpose()[1],obs = OBS)
            if obs_idx_cw:
                cw_obs_len = obs_idx_cw + int(scan_len/dt) # 前移0.5弧度
        # 判断更新
        if obs_idx_ccw is not None and obs_idx_cw is not None and add_theta_tmp < 3.15 and not last_check:
            add_theta_tmp += 0.1
            continue
        else:
            #增量超过半圈，手动关闭节点
            if add_theta_tmp >= 3.15:
                if ccw_check:
                    re_back_node.left = bt.TreeNode('1', '1')
                elif cw_check:
                    re_back_node.right = bt.TreeNode('1', '1')
                break
            if not last_check:
                add_theta_tmp = add_theta_tmp - 0.1 + 0.01
                last_check = True
                if obs_idx_ccw is None:
                    cw_check = False
                elif obs_idx_cw is None:
                    ccw_check = False
                continue
            elif (ccw_check and obs_idx_ccw is not None) or (cw_check and obs_idx_cw is not None):
                add_theta_tmp = add_theta_tmp + 0.01
            else:
                last_check = False
                if ccw_check:
                    # 此处时针对于线上没有障碍物，但实际延长线上存在障碍物的情况
                    if obs_idx_ccw_old is not None:
                        xy = np.array([path_tmp_ccw[obs_idx_ccw_old,0],path_tmp_ccw[obs_idx_ccw_old,1]])
                        obs_idx_near_ccw = ka.obs_idx_near(xy,path_tmp_ccw_old)
                    else: #指定延长线上没有找到碰撞
                        obs_idx_near_ccw = math.ceil(len(path_tmp_ccw_old)/10)
                else:
                    if obs_idx_cw_old is not None:
                        xy = np.array([path_tmp_cw[obs_idx_cw_old,0],path_tmp_cw[obs_idx_cw_old,1]])
                        obs_idx_near_cw = ka.obs_idx_near(xy,path_tmp_cw_old)
                    else: #指定延长线上没有找到碰撞
                        obs_idx_near_cw = math.ceil(len(path_tmp_cw_old)/10)
                break
#     找到了新的可加入的预节点：
#     回退点在二叉树中的路径
    if add_theta_tmp >=3.15:
        continue
    path_seq_copy = re_back_seq.copy()
    # 在当前回退点的逆时针侧添加路径
    if obs_idx_near_ccw is not None:
        re_back_point = np.array([path_tmp_ccw[obs_idx_near_ccw, 0], path_tmp_ccw[obs_idx_near_ccw, 1]])
        weight = euclidean(re_back_point, goal)
        tree_node = bt.In_tree_value(weight, path_tmp_ccw[:obs_idx_near_ccw, :])
        plt.plot(path_tmp_ccw[:obs_idx_near_ccw, 0],path_tmp_ccw[:obs_idx_near_ccw, 1], color='gray')
        if get_pic:
            pic_count = pic_count + 1
            filename = 'temp_image_' + str(pic_count) + '.png'  # 生成唯一的文件名
            plt.savefig(filename, dpi=300)
        tree_node.ori = 'l'
        path_seq_copy.append('l')
        tree_node.sequ = path_seq_copy.copy()
        Bi_tree.add_node(tree_node)
        Order_dict.add_item(weight, path_seq_copy.copy())
        path_check_end = path_tmp_ccw[obs_idx_near_ccw, :]
    #在当前回退点的顺时针侧添加路径
    else:
        re_back_point = np.array([path_tmp_cw[obs_idx_near_cw, 0], path_tmp_cw[obs_idx_near_cw, 1]])
        weight = euclidean(re_back_point, goal)
        tree_node = bt.In_tree_value(weight, path_tmp_cw[:obs_idx_near_cw, :])
        plt.scatter(path_tmp_cw[obs_idx_near_cw, 0],path_tmp_cw[obs_idx_near_cw, 1], color='dodgerblue')
        plt.plot(path_tmp_cw[:obs_idx_near_cw, 0],path_tmp_cw[:obs_idx_near_cw, 1], color='slategray')
        if get_pic:
            pic_count = pic_count + 1
            filename = 'temp_image_' + str(pic_count) + '.png'  # 生成唯一的文件名
            plt.savefig(filename, dpi=300)
        tree_node.ori = 'r'
        path_seq_copy.append('r')
        tree_node.sequ = path_seq_copy.copy()
        Bi_tree.add_node(tree_node)
        Order_dict.add_item(weight, path_seq_copy.copy())
        path_check_end = path_tmp_cw[obs_idx_near_cw, :]
    # 将找到的路径加入
    # path = np.concatenate((path, path_check_end))
#     在新点向终点方向寻找，如果没有碰撞则退出全部循环，否则检测新的回退点，并加入
    #     二叉树及顺序字典，并进入下一循环
#     该点位不需要回退，不需要再寻找最短欧氏距离函数
    ka_end = ka.kappa_init(re_back_point,goal,path_check_end[2])
    path_tmp = ka.integer_kappa(path_check_end[0],path_check_end[1],theta_init=path_check_end[2],kappa_seq=ka_end)
    plt.scatter(path_tmp[0,0], path_tmp[0,1], color='dodgerblue')
    plt.plot(path_tmp[:,0], path_tmp[:,1], color='slategray')
    if get_pic:
        pic_count = pic_count + 1
        filename = 'temp_image_' + str(pic_count) + '.png'  # 生成唯一的文件名
        plt.savefig(filename, dpi=300)
    last_collision_check_idx = ka.obs_detect(path_tmp.transpose()[0], path_tmp.transpose()[1], obs = OBS)
    plt.plot(path_tmp[last_collision_check_idx,0], path_tmp[last_collision_check_idx,1], color='gray')
    if get_pic:
        pic_count = pic_count + 1
        filename = 'temp_image_' + str(pic_count) + '.png'  # 生成唯一的文件名
        plt.savefig(filename, dpi=300)
    # path_end = Bi_tree.get_tree_path_sque(path_seq_copy)
    if last_collision_check_idx is None:
        # path = np.concatenate((path, path_tmp))
        path_end = path_tmp
        break
    # TODO 碰到下一阶段障碍物，将回退点加入二叉树及顺序字典，进入下一循环
    else:
        # 末端新段的碰撞点的新增距离,小于了回退距离
        re_back_len = math.ceil(back_obs / dt)
        tmp = last_collision_check_idx - re_back_len
        if tmp <= 0: #TODO
            # 小于0则源路径向前延申0.2弧度的长度
            conti_ka = np.ones(math.ceil(back_obs / dt / 5 )) * ka_zero
            conti_line_path = ka.integer_kappa(path_check_end[0],path_check_end[1],theta_init=path_check_end[2],kappa_seq=conti_ka)
            # plt.scatter(conti_line_path[0, 0], conti_line_path[0, 1], color='dodgerblue')
            plt.scatter(conti_line_path[-1, 0], conti_line_path[-1, 1], color='dodgerblue')
            plt.plot(conti_line_path[:, 0], conti_line_path[:, 1], color='slategray')
            if get_pic:
                pic_count = pic_count + 1
                filename = 'temp_image_' + str(pic_count) + '.png'  # 生成唯一的文件名
                plt.savefig(filename, dpi=300)
            re_back_point = np.array([conti_line_path[0, 0], conti_line_path[0, 1]])
            # 将原来回退点调整
            weight = euclidean(re_back_point, goal)

            path_seq_copy.append(tree_node.ori)
            tree_node_conti = bt.In_tree_value(weight, conti_line_path)
            tree_node_conti.ori = tree_node.ori
            tree_node_conti.sequ = path_seq_copy.copy()
            Bi_tree.add_node(tree_node_conti)
            Order_dict.add_item(weight, path_seq_copy.copy())
            # try 终端加入无碰撞检测，碰撞不做任何，继续，无碰撞，跳出
            try_path_end_ka = ka.kappa_init(np.array([conti_line_path[-1,0], conti_line_path[-1,1]]),goal,theta=conti_line_path[-1,2])
            try_path_end = ka.integer_kappa(conti_line_path[-1,0],conti_line_path[-1,1],conti_line_path[-1,2], kappa_seq=try_path_end_ka)
            plt.plot(try_path_end[:,0],try_path_end[:,1], color='slategray')
            if get_pic:
                pic_count = pic_count + 1
                filename = 'temp_image_' + str(pic_count) + '.png'  # 生成唯一的文件名
                plt.savefig(filename, dpi=300)
            try_path_end_idx = ka.obs_detect(try_path_end.transpose()[0], try_path_end.transpose()[1], obs = OBS)
            if try_path_end_idx is not None:
                if try_path_end_idx > re_back_len:
                #加入新回退点
                    plt.scatter(try_path_end[re_back_len,0], try_path_end[re_back_len,1], color='dodgerblue')
                    plt.plot(try_path_end[:re_back_len,0], try_path_end[:re_back_len,1], color='slategray')
                    if get_pic:
                        pic_count = pic_count + 1
                        filename = 'temp_image_' + str(pic_count) + '.png'  # 生成唯一的文件名
                        plt.savefig(filename, dpi=300)
                    tt = None
                    if try_path_end_ka[2] > 0:
                        tt = 'l'
                    else:
                        tt = 'r'
                    re_back_point = np.array([try_path_end[re_back_len,0], try_path_end[re_back_len,1]])
                    weight = euclidean(re_back_point, goal)
                    path_seq_copy.append(tt)
                    tree_node = bt.In_tree_value(weight, try_path_end[:re_back_len,:])
                    tree_node.ori = tt
                    tree_node.sequ = path_seq_copy.copy()
                    Bi_tree.add_node(tree_node)
                    Order_dict.add_item(weight, path_seq_copy.copy())
            else:
                path_end = try_path_end
                break
            continue
        else:
            plt.scatter(path_tmp[tmp, 0], path_tmp[tmp, 1], color='dodgerblue')
            plt.plot(path_tmp[:tmp, 0], path_tmp[:tmp, 1], color='slategray')
            if get_pic:
                pic_count = pic_count + 1
                filename = 'temp_image_' + str(pic_count) + '.png'  # 生成唯一的文件名
                plt.savefig(filename, dpi=300)
            re_back_point = np.array([path_tmp[tmp, 0], path_tmp[tmp, 1]])
            weight = euclidean(re_back_point, goal)
            tt = None
            if ka_end[2] > 0:
                tt = 'l'
            else:
                tt = 'r'
            path_seq_copy.append(tt)
            tree_node = bt.In_tree_value(weight, path_tmp[:tmp, :])
            tree_node.ori = tt
            tree_node.sequ = path_seq_copy.copy()
            Bi_tree.add_node(tree_node)
            Order_dict.add_item(weight, path_seq_copy.copy())

min_path_seg = Order_dict.get_min_key_value()[1]
path_seg = Bi_tree.get_tree_path_sque(min_path_seg)
# path = np.concatenate((path_first,path_seg))
path = np.concatenate((path_seg,path_end))
plt.plot(path.transpose()[0], path.transpose()[1], color='limegreen')
if get_pic:
    pic_count = pic_count + 1
    filename = 'temp_image_' + str(pic_count) + '.png'  # 生成唯一的文件名
    plt.savefig(filename, dpi=300)

plt.show()
