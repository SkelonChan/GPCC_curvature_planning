
from matplotlib import pyplot as plt
from matplotlib import patches
from matplotlib import pyplot as plt
from matplotlib.patches import Circle, Ellipse, Path, PathPatch

def gen_obs(gen_obs, ax):
    res = []
    for obs in gen_obs:
        if len(obs) == 2:  # 如果是圆
            circle = Circle(obs[0], obs[1], edgecolor='black', facecolor='lightgrey')
            ax.add_patch(circle)
            res.append(circle)
        elif len(obs) == 3:  # 如果是椭圆
            ellipse = Ellipse(obs[0], obs[1], obs[2],angle= -5, edgecolor='black', facecolor='lightgrey')
            ax.add_patch(ellipse)
            res.append(ellipse)
        else:  # 如果是多边形
            path = Path(obs, closed=True)
            patch = PathPatch(path, edgecolor='black', facecolor='lightgrey')
            ax.add_patch(patch)
            res.append(path)
    return res


# 绘制路径
fig, ax = plt.subplots()
ax.set_xlim(-5, 5)
ax.set_ylim(-5, 5)
ax.set_aspect('equal')
# # 检查点是否在路径内部

a1 = [(-2.5,2),(-2.5, 3),(-1,1),(-3.5, 2)]
a2 = [(-4, 1),(-4.85, 0),(-3,0),(-4, 1)]
a3 = [(-4, -1),(-3,-1),(-2.7,-2),(-4,-3), (-4, -1)]
a4 = [(-2, -2),(-1,-2),(-1,-5),(-2,-5), (-2, -2)]
a5 = [(1, 3),(2,3),(2,5),(1,5), (1, 3)]
a6 = [(0, 1.5),(-1, 0),(1,0),(1,1), (0, 1.5)]
a7 = [(0.5, -1.3),(0.2,-2.5),(1,-3.8),(2,-3), (0.5,-1.3)]
a8 = [(3, 1.5),(5, 1.5),(5,-1.5),(3,-1.5), (3,1.5)]
a9 = [(-2.5, 5),(-0.5,5),(0.5,3),(-1.5,3), (-2.5, 5)]
# 定义圆和椭圆
circle1 = ((-1.8, -0.3), 0.5)
ellipse1 = ((1.7, 2), 2, 1)
gen_obs_ = [a1, a2, a3, a4, a5, a6, a7, a8, a9, circle1, ellipse1]
OBS = gen_obs(gen_obs_, ax)

# 待检测的点
point = (2, 2)
plt.scatter(point[0], point[1], marker='o', color='red')

# 将点的坐标转换到图形坐标系下
point_in_data_coordinates = point
point_in_figure_coordinates = ax.transData.transform(point_in_data_coordinates)

# 进行判断
for ob in OBS:
    is_inside = ob.contains_point(point_in_figure_coordinates)
    print(is_inside)

plt.show()




