'''
Author: Skelon_Chan 269529898@msn.cn
Date: 2024-01-04 13:34:49
LastEditors: Skelon_Chan 269529898@msn.cn
LastEditTime: 2024-01-22 14:27:41
FilePath: \K_planning\kappa_planning_base.py
Description: 
'''
import numpy as np
import math
from matplotlib.patches import Circle, Ellipse, Path, PathPatch
from matplotlib.path import Path
from matplotlib import patches


# TODO:
# ERROR:
# 当设定的曲率过小，即半径过大的时候，回到导致实现的曲线在横轴平行处出现来回的画龙现象
# 导致的原因为当前点更新的目标角度在-180 处发生了符号变化，导致进入了反向曲率循环，
# 因此造成在横轴出画半圆，来回画龙
# 可以输出[x,y,theta.kappa],避免再次积分
def kappa_init(init_pos, goal_pos, theta=0, dt=0.01, ka_limit=2.5,
               ka_dot_limit=40, ka_zero=0.0001, v_=1):
    '''
    :param init_pos:    初始位置numpy.array()
    :param goal_pos:    终止位置numpy.array()
    :param dt:  默认欧拉积分时间
    :param ka_limit:    默认曲率极值限制[-ka_limit,ka_limit]
    :param ka_dot_limit:    曲率增量限制
    :param ka_zero: 直线曲率值
    :param v_:  默认曲线长度变化率, ds/dt
    :return: numpy.array(ka), 关于时间的曲率序列
    '''
    x_value = np.array([init_pos[0]])
    y_value = np.array([init_pos[1]])
    theta = np.array([theta])
    ka = np.array([0])
    target_value = np.arctan2((goal_pos[1] - init_pos[1]), (goal_pos[0] - init_pos[0]))
    x_integer = theta[0]  #
    if theta[0] != target_value:
        # if theta == 0:
        while True and len(x_value) <= 2000:
            x_ = x_value[-1]
            y_ = y_value[-1]
            if x_integer < target_value:
                ka_ = ka[-1] + ka_dot_limit * dt
                if ka_ > ka_limit:
                    ka_ = ka_limit
                theta_ = theta[-1] + ka_ * dt
                x_integer = theta_
                if x_integer > target_value:
                    break
            elif x_integer > target_value:
                ka_ = ka[-1] - ka_dot_limit * dt
                if ka_ < -ka_limit:
                    ka_ = -ka_limit
                theta_ = theta[-1] + ka_ * dt
                x_integer = theta_
                if x_integer < target_value:
                    break
            x_new = x_ + v_ * np.cos(theta_) * dt
            y_new = y_ + v_ * np.sin(theta_) * dt
            ka = np.append(ka, ka_)
            theta = np.append(theta, theta_)
            x_value = np.append(x_value, x_new)
            y_value = np.append(y_value, y_new)
            target_value = np.arctan2((goal_pos[1] - y_new), (goal_pos[0] - x_new))

    dis_gap = np.linalg.norm(np.array([x_value[-1], y_value[-1]]) - goal_pos)
    ka_add = np.ones(math.ceil(dis_gap / dt / v_) - 1) * np.sin(target_value) * ka_zero
    ka = np.append(ka, ka_add)

    return ka


def obs_detect(x, y, obs):
    '''
    :param obs_xy: 障碍物点的左下角坐标
    :param obs_height: 障碍物高
    :param obs_width: 障碍物宽度
    :param x: list,恢复出的轨迹的x序列[x]
    :param y: list,恢复出的轨迹的y序列[y]
    :return: idx, 发生碰撞的索引坐标
    '''
    for idx, (xi, yi) in enumerate(zip(x, y)):
        for ob in obs:
            if ob.contains_point((xi, yi)):
                return idx


def circle_update_ccw(kappa, theta, add_theta, dt=0.01, ka_limit=1.5,
                      ka_dot_limit=40):
    theta = np.array([theta])
    ka = np.array([kappa])
    theta_integer = 0
    while abs(theta_integer) <= add_theta:
        ka_ = ka[-1] + ka_dot_limit * dt
        if ka_ > ka_limit:
            ka_ = ka_limit
        theta_integer = theta_integer + ka_ * dt
        theta_ = theta[-1] + theta_integer
        theta = np.append(theta, theta_)
        ka = np.append(ka, ka_)
    return ka


def circle_update_cw(kappa, theta, add_theta, dt=0.01, ka_limit=1.5,
                     ka_dot_limit=40):
    theta = np.array([theta])
    ka = np.array([kappa])
    theta_integer = 0
    while abs(theta_integer) <= add_theta:
        ka_ = ka[-1] - ka_dot_limit * dt
        if ka_ < ka_limit:
            ka_ = -ka_limit
        theta_integer = theta_integer + ka_ * dt
        theta_ = theta[-1] + theta_integer
        theta = np.append(theta, theta_)
        ka = np.append(ka, ka_)
    return ka


# 废弃
def kappa_circle_update(kappa, theta, add_theta, inverse=1, dt=0.01, ka_limit=1.5,
                        ka_dot_limit=40):
    """
    对碰撞点回退进行方向变更
    :param kappa: 回退点处的曲率
    :param add_theta: update中增加的角度,作为退出条件
    :param inverse: 是否转变方向角
    :param theta: 起始theta朝向角
    :param dt: 欧拉积分时间
    :param ka_limit: 曲率最大值限定
    :param ka_dot_limit: 曲率变化最大值限定
    :return: np.array: new kappa sequence
    """
    theta = np.array([theta * inverse])
    ka = np.array([kappa])
    theta_integer = 0
    # 此处选择theta改变的方向与原方向相同
    while abs(theta_integer) <= add_theta:
        if theta[0] >= 0:
            ka_ = ka[-1] + ka_dot_limit * dt
            if ka_ > ka_limit:
                ka_ = ka_limit
            theta_integer = theta_integer + ka_ * dt
            theta_ = theta[-1] + theta_integer
        else:
            ka_ = ka[-1] - ka_dot_limit * dt
            if ka_ < -ka_limit:
                ka_ = -ka_limit
            theta_integer = theta_integer + ka_ * dt
            theta_ = theta[-1] + theta_integer
        theta = np.append(theta, theta_)
        ka = np.append(ka, ka_)
    return ka


def integer_kappa(x_init, y_init, theta_init, kappa_seq, dt=0.01, v_=1):
    '''
    对已有的曲率函数进行欧拉积分展示轨迹
    :param x_init: 初始点的x坐标
    :param y_init: 初始点的y坐标
    :param theta_init: 初始点的朝向角
    :param kappa_seq: 给定的待积分曲率函数序列
    :param dt: 默认欧拉积分时间间隔
    :param v_: 默认速率 ds/dt
    :return: numpy 轨迹点的序列[[x,y,theta,kappa]]
    '''
    x_value_p = np.array([x_init])
    y_value_p = np.array([y_init])
    theta_p = np.array([theta_init])
    ka_tmp = np.append(kappa_seq, kappa_seq[-1])
    for k in kappa_seq[:]:
        x = x_value_p[-1]
        y = y_value_p[-1]
        v_p = v_
        the = theta_p[-1] + k * dt  # 更新方向角度
        theta_p = np.append(theta_p, the)
        x_new_p = x + v_p * np.cos(the) * dt
        y_new_p = y + v_p * np.sin(the) * dt
        x_value_p = np.append(x_value_p, x_new_p)
        y_value_p = np.append(y_value_p, y_new_p)
    res = np.array([x_value_p, y_value_p, theta_p, ka_tmp]).transpose()
    return res


def obs_idx_near(obs_pos_xy, new_path_tmp):
    '''
    寻找距离点obs_pos_xy 距离路径path最近点的索引
    :param obs_pos_xy: np.array(x,y),
    :param new_path_tmp: np.array([[x,y,theta,kappa],...])
    :return: int: idx
    '''
    path = new_path_tmp[:, :2]
    distance_ = np.linalg.norm(path - obs_pos_xy, axis=1)
    idx = np.argmin(distance_)
    return idx


def gen_obs(gen_obs, ax):
    res = []
    for obs in gen_obs:
        if len(obs) == 2:  # 如果是圆
            circle = Circle(obs[0], obs[1], edgecolor='black', facecolor='lightgrey')
            ax.add_patch(circle)
            res.append(circle)
        elif len(obs) == 3:  # 如果是椭圆
            ellipse = Ellipse(obs[0], obs[1], obs[2], edgecolor='black', facecolor='lightgrey')
            ax.add_patch(ellipse)
            res.append(ellipse)
        else:  # 如果是多边形
            path = Path(obs, closed=True)
            patch = PathPatch(path, edgecolor='black', facecolor='lightgrey')
            ax.add_patch(patch)
            res.append(path)
    return res


def obs_detect2(x, y, obs,ax):
    '''
    :param obs_xy: 障碍物点的左下角坐标
    :param obs_height: 障碍物高
    :param obs_width: 障碍物宽度
    :param x: list,恢复出的轨迹的x序列[x]
    :param y: list,恢复出的轨迹的y序列[y]
    :return: idx, 发生碰撞的索引坐标
    '''
    for idx, (xi, yi) in enumerate(zip(x, y)):
        for ob in obs[:-2]:
            if ob.contains_point((xi, yi)):
                return idx
        for ob in obs[-2:]:
            point = ax.transData.transform((xi, yi))
            if ob.contains_point(point):
                return idx

        
