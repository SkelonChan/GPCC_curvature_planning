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
import bi_tree as bt
from scipy.spatial.distance import euclidean



init = np.array([-4, -4])
goal = np.array([4, 4])
dt = 0.01
ka_zero = 0.0001
back_obs = 0.8
add_theta = 0.1
scan_len = 0.5
last_check = False  #
path_end = None
theta_init = 0
x_value_p = np.array([init[0]])
y_value_p = np.array([init[1]])
theta_p = np.array([theta_init])
# plt.ion()

fig, ax = plt.subplots()
fig.set_dpi(200)
ax.set_xlim(-5, 5)
ax.set_ylim(-5, 5)
ax.set_aspect('equal')
a1 = [(-2.5,2),(-2.5, 3),(-1,1),(-3.5, 2)]
a2 = [(-4, 1),(-4.85, 0),(-3,0),(-4, 1)]
a3 = [(-4, -1),(-3,-1),(-2.7,-2),(-4,-3), (-4, -1)]
a4 = [(-2, -2),(-1,-2),(-1,-5),(-2,-5), (-2, -2)]
a5 = [(1, 3),(2,3),(2,5),(1,5), (1, 3)]
a6 = [(0, 1.5),(-1, 0),(1,0),(1,1), (0, 1.5)]
a7 = [(0.5, -1.3),(0.2,-2.5),(1,-3.8),(2,-3), (0.5,-1.3)]
a8 = [(3, 1.5),(5, 1.5),(5,-1.5),(3,-1.5), (3,1.5)]
a9 = [(-2.5, 5),(-0.5,5),(0.5,3),(-1.5,3), (-2.5, 5)]
circle1 = ((-1.8, -0.3), 0.8)
ellipse1 = ((1.7, 2),3, 1)


gen_obs_ = [a1, a2, a3, a4, a5, a6, a7, a8, a9, circle1, ellipse1]
OBS = ka.gen_obs(gen_obs_, ax)
plt.scatter(init[0], init[1], marker='o', color='lime')
plt.scatter(goal[0], goal[1], marker='o', color='red')

theta_init_goal = np.arctan2(goal[1] - init[1], goal[0] - init[0])
Bi_tree = bt.BinaryTree()
Order_dict = bt.OrderedKeyValueStructure()
ka_ = ka.kappa_init(init, goal, theta=theta_init)
kappa_se = ka_
path_first = ka.integer_kappa(init[0], init[1], theta_init=theta_init, kappa_seq=kappa_se)
plt.plot(path_first.transpose()[0], path_first.transpose()[1], color='gray')
obs_idx = ka.obs_detect2(path_first.transpose()[0], path_first.transpose()[1],OBS,ax=ax)
obs_line_len = len(path_first[:, 0])
re_back_step = math.ceil(obs_idx - back_obs / dt)

obs_idx_first = obs_idx
if obs_idx_first is not None:
    re_back_point = np.array([path_first[re_back_step, 0], path_first[re_back_step, 1]])
    weight = euclidean(re_back_point, goal)
    node_root = bt.In_tree_value(weight, path_first[:re_back_step, :])
    Bi_tree.add_root_node(copy.deepcopy(node_root))
    Order_dict.add_item(weight, [])
    path_first = path_first[:re_back_step, :]
else:
    path = path_first
while obs_idx_first is not None:
    add_theta_tmp = add_theta
    ccw_obs_len = obs_line_len
    cw_obs_len = obs_line_len
    ccw_check = cw_check = True
    obs_idx_near_cw = obs_idx_near_ccw = None
    obs_idx_ccw = 1
    obs_idx_cw = obs_idx_ccw
    path_tmp_ccw = path_tmp_cw = None
    re_back_point = None
    path_check_end = None
    re_back_seq = Order_dict.get_min_key_value()[1]
    re_back_node = Bi_tree.get_node(re_back_seq)
    if re_back_node.left is not None:
        ccw_check = False
    if re_back_node.right is not None:
        cw_check = False
    re_back_path = Bi_tree.get_tree_path_sque(re_back_seq)
    plt.scatter(re_back_path[-1,0], re_back_path[-1,1],  color='dodgerblue')
    ka_l4ine = []
    while True:
        obs_idx_ccw_old = obs_idx_ccw
        obs_idx_cw_old = obs_idx_cw
        path_tmp_ccw_old = path_tmp_ccw
        path_tmp_cw_old = path_tmp_cw
        if ccw_check:
            ka_circle_ccw = ka.circle_update_ccw(re_back_path[-1, 3], re_back_path[-1, 2], add_theta_tmp)
            if int(ccw_obs_len) > len(ka_circle_ccw):
                ka_line = np.ones(ccw_obs_len - len(ka_circle_ccw)) * ka_zero
            else:
                ka_line = np.ones(1) * ka_zero
            ka_com_ccw = np.concatenate((ka_circle_ccw, ka_line))
            path_tmp_ccw = ka.integer_kappa(re_back_path[-1, 0], re_back_path[-1, 1], re_back_path[-1, 2], kappa_seq=ka_com_ccw)
            plt.plot(path_tmp_ccw.transpose()[0], path_tmp_ccw.transpose()[1], color='gray', linestyle='--', linewidth=0.5)
            obs_idx_ccw = ka.obs_detect2(x=path_tmp_ccw.transpose()[0], y=path_tmp_ccw.transpose()[1], obs = OBS,ax=ax)
            if obs_idx_ccw:
                ccw_obs_len = obs_idx_ccw + int(scan_len/dt)
        if cw_check:
            ka_circle_cw = ka.circle_update_cw(re_back_path[-1, 3], re_back_path[-1, 2], add_theta_tmp)
            if int(cw_obs_len) > len(ka_circle_cw):
                ka_line = np.ones(cw_obs_len - len(ka_circle_cw)) * ka_zero
            else:
                ka_line = np.ones(1) * ka_zero
            ka_com_cw = np.concatenate((ka_circle_cw, ka_line))
            path_tmp_cw = ka.integer_kappa(re_back_path[-1, 0], re_back_path[-1, 1], re_back_path[-1, 2], kappa_seq = ka_com_cw)
            plt.plot(path_tmp_cw.transpose()[0], path_tmp_cw.transpose()[1], color='gray', linestyle='--', linewidth=0.5)
            obs_idx_cw = ka.obs_detect2(x=path_tmp_cw.transpose()[0], y=path_tmp_cw.transpose()[1],obs = OBS,ax=ax)
            if obs_idx_cw:
                cw_obs_len = obs_idx_cw + int(scan_len/dt)
        if obs_idx_ccw is not None and obs_idx_cw is not None and add_theta_tmp < 3.15 and not last_check:
            add_theta_tmp += 0.1
            continue
        else:
            if add_theta_tmp >= 3.15:
                if ccw_check:
                    re_back_node.left = bt.TreeNode('1', '1')
                elif cw_check:
                    re_back_node.right = bt.TreeNode('1', '1')
                break
            if not last_check:
                add_theta_tmp = add_theta_tmp - 0.1
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
                    if obs_idx_ccw_old is not None:
                        xy = np.array([path_tmp_ccw[obs_idx_ccw_old,0],path_tmp_ccw[obs_idx_ccw_old,1]])
                        obs_idx_near_ccw = ka.obs_idx_near(xy,path_tmp_ccw_old)
                    else:
                        obs_idx_near_ccw = math.ceil(len(path_tmp_ccw_old)/10)
                else:
                    if obs_idx_cw_old is not None:
                        xy = np.array([path_tmp_cw[obs_idx_cw_old,0],path_tmp_cw[obs_idx_cw_old,1]])
                        obs_idx_near_cw = ka.obs_idx_near(xy,path_tmp_cw_old)
                    else:
                        obs_idx_near_cw = math.ceil(len(path_tmp_cw_old)/10)
                break
    if add_theta_tmp >=3.15:
        continue
    path_seq_copy = re_back_seq.copy()
    if obs_idx_near_ccw is not None:
        re_back_point = np.array([path_tmp_ccw[obs_idx_near_ccw, 0], path_tmp_ccw[obs_idx_near_ccw, 1]])
        weight = euclidean(re_back_point, goal)
        tree_node = bt.In_tree_value(weight, path_tmp_ccw[:obs_idx_near_ccw, :])
        plt.plot(path_tmp_ccw[:obs_idx_near_ccw, 0],path_tmp_ccw[:obs_idx_near_ccw, 1], color='gray')
        tree_node.ori = 'l'
        path_seq_copy.append('l')
        tree_node.sequ = path_seq_copy.copy()
        Bi_tree.add_node(tree_node)
        Order_dict.add_item(weight, path_seq_copy.copy())
        path_check_end = path_tmp_ccw[obs_idx_near_ccw, :]
    else:
        re_back_point = np.array([path_tmp_cw[obs_idx_near_cw, 0], path_tmp_cw[obs_idx_near_cw, 1]])
        weight = euclidean(re_back_point, goal)
        tree_node = bt.In_tree_value(weight, path_tmp_cw[:obs_idx_near_cw, :])
        plt.scatter(path_tmp_cw[obs_idx_near_cw, 0],path_tmp_cw[obs_idx_near_cw, 1], color='dodgerblue')
        plt.plot(path_tmp_cw[:obs_idx_near_cw, 0],path_tmp_cw[:obs_idx_near_cw, 1], color='slategray')
        tree_node.ori = 'r'
        path_seq_copy.append('r')
        tree_node.sequ = path_seq_copy.copy()
        Bi_tree.add_node(tree_node)
        Order_dict.add_item(weight, path_seq_copy.copy())
        path_check_end = path_tmp_cw[obs_idx_near_cw, :]
    ka_end = ka.kappa_init(re_back_point,goal,path_check_end[2])
    path_tmp = ka.integer_kappa(path_check_end[0],path_check_end[1],theta_init=path_check_end[2],kappa_seq=ka_end)
    plt.scatter(path_tmp[0,0], path_tmp[0,1], color='dodgerblue')
    plt.plot(path_tmp[:,0], path_tmp[:,1], color='slategray')
    last_collision_check_idx = ka.obs_detect2(path_tmp.transpose()[0], path_tmp.transpose()[1], obs = OBS,ax=ax)
    plt.plot(path_tmp[last_collision_check_idx,0], path_tmp[last_collision_check_idx,1], color='slategray')
    if last_collision_check_idx is None:
        path_end = path_tmp
        break
    else:
        # 末端新段的碰撞点的新增距离,小于了回退距离
        re_back_len = math.ceil(back_obs / dt)
        tmp = last_collision_check_idx - re_back_len
        if tmp <= 0: #TODO
            conti_ka = np.ones(math.ceil(back_obs / dt / 5 )) * ka_zero
            conti_line_path = ka.integer_kappa(path_check_end[0],path_check_end[1],theta_init=path_check_end[2], kappa_seq=conti_ka)
            plt.scatter(conti_line_path[-1, 0], conti_line_path[-1, 1], color='dodgerblue')
            plt.plot(conti_line_path[:, 0], conti_line_path[:, 1], color='slategray')
            re_back_point = np.array([conti_line_path[0, 0], conti_line_path[0, 1]])
            weight = euclidean(re_back_point, goal)

            path_seq_copy.append(tree_node.ori)
            tree_node_conti = bt.In_tree_value(weight, conti_line_path)
            tree_node_conti.ori = tree_node.ori
            tree_node_conti.sequ = path_seq_copy.copy()
            Bi_tree.add_node(tree_node_conti)
            Order_dict.add_item(weight, path_seq_copy.copy())
            try_path_end_ka = ka.kappa_init(np.array([conti_line_path[-1,0], conti_line_path[-1,1]]),goal,theta=conti_line_path[-1,2])
            try_path_end = ka.integer_kappa(conti_line_path[-1,0],conti_line_path[-1,1],conti_line_path[-1,2], kappa_seq=try_path_end_ka)
            plt.plot(try_path_end[:,0],try_path_end[:,1], color='slategray')
            try_path_end_idx = ka.obs_detect2(try_path_end.transpose()[0], try_path_end.transpose()[1], obs = OBS,ax=ax)
            if try_path_end_idx is not None:
                if try_path_end_idx > re_back_len:
                    plt.scatter(try_path_end[re_back_len,0], try_path_end[re_back_len,1], color='dodgerblue')
                    plt.plot(try_path_end[:re_back_len,0], try_path_end[:re_back_len,1], color='slategray')
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
path = np.concatenate((path_seg,path_end))
plt.plot(path.transpose()[0], path.transpose()[1], color='limegreen')
plt.show()



