# -*- coding: utf-8 -*-
"""
Created on Tue May 14 16:26:31 2019

@author: 拔凉拔凉冰冰凉
"""
import matplotlib.pyplot as plt
import numpy as np

data_file = "data.txt"

n = 0
m = 0
r = 7
A_s = 0
A_o = 0
chargers = []
sensors = []
cover = []
N = []
with open(data_file, "r", encoding="utf-8") as f:
    lines = f.readlines()
    line1 = lines[0].split("\n")[0]
    line1 = line1.split(" ")
    n = int(line1[0])
    m = int(line1[1])
    
    for i in range(1, 1+n):
        line = lines[i].split("\n")[0]
        line = line.split(" ")
        x = float(line[0])
        y = float(line[1])
        chargers.append([x, y])
    for i in range(1+n, 1+n+m):
        line = lines[i].split("\n")[0]
        line = line.split(" ")
        x = float(line[0])
        y = float(line[1])
        dir_ = float(line[2])
        e = float(line[3])
        sensors.append([x, y, dir_, e])
    line = lines[1+m+n].split("\n")[0]
    line = line.split(" ")
    A_s = float(line[0])
    A_o = float(line[1])
    A_s = A_s*np.pi/180
    A_o = A_o*np.pi/180
    for i in range(2+m+n, 2+m+n+m):
        line = lines[i].split("\n")[0]
        line = line.split(" ")
        cover_line = []
        for j in range(m):
            cover_line.append(float(line[j]))
        N.append(cover_line)
    for i in range(2+m+n+m, 2+m+n+m+n):
        line = lines[i].split("\n")[0]
#        print(line)
        line = line.split(" ")
        cover_line = []
        for j in range(m):
            cover_line.append(int(line[j]))
        cover.append(cover_line)
#    print(A_s, A_o)

c_plot = chargers
s_plot = []
c_xuhao = []
s_xuhao = []
for i in range(n):
    c_xuhao.append(i)
for i in range(len(sensors)):
    s_plot.append(sensors[i][:2])
    s_xuhao.append(i)

c_plot = np.array(c_plot)
s_plot = np.array(s_plot)


fig = plt.figure(figsize=(25, 25))
ax1 = fig.add_subplot(111)
#设置X轴标签
plt.xlabel('X')
#设置Y轴标签
plt.ylabel('Y')
ax1.scatter(x=c_plot[:,0],y=c_plot[:,1],c = 'r',marker = 'o', s=90)
ax1.scatter(x=s_plot[:,0],y=s_plot[:,1],c = 'b',marker = 'o', s=90)
for i in range(n):
    plt.annotate(c_xuhao[i], xy = (c_plot[i][0], c_plot[i][1]), xytext = (c_plot[i][0]+0.1, c_plot[i][1]+0.1)) # 这里xy是需要标记
for i in range(m):
    plt.annotate(s_xuhao[i], xy = (s_plot[i][0], s_plot[i][1]), xytext = (s_plot[i][0]+0.1, s_plot[i][1]+0.1)) # 这里xy是需要标记
#画圆
theta = np.linspace(0, 2*np.pi,800)
x,y = np.cos(theta), np.sin(theta)
x = x*r
y = y*r
for i in range(c_plot.shape[0]):
    ax1.plot(x+c_plot[i][0], y+c_plot[i][1], color='green', linewidth=0.5)

for i in range(n):
    for j in range(m):
        if cover[i][j] == 1:
            ax1.plot([s_plot[j][0], c_plot[i][0]], [s_plot[j][1], c_plot[i][1]], color="orange", linewidth=0.5)

for i in range(m):
    for j in range(m):
        if N[i][j] >0 and N[j][i]>0:
            ax1.plot([s_plot[i][0], s_plot[j][0]], [s_plot[i][1], s_plot[j][1]], color="black", linewidth=0.5)
#画扇形
#for i in range(m):
#    dir_ = sensors[i][2]
#    dir_s = dir_ - A_o/2
#    dir_e = dir_ + A_o/2
#    theta = np.linspace(dir_s, dir_e,800)
#    x,y = np.cos(theta), np.sin(theta)
#    x = x*r
#    y = y*r
#    ax1.plot(x+s_plot[i][0], y+s_plot[i][1], color='orange', linewidth=0.5)
#    ax1.plot([x[0]+s_plot[i][0], s_plot[i][0]], [y[0]+s_plot[i][1], s_plot[i][1]], color="orange", linewidth=0.5)
#    ax1.plot([x[-1]+s_plot[i][0], s_plot[i][0]], [y[-1]+s_plot[i][1], s_plot[i][1]], color="orange", linewidth=0.5)
#设置图标
plt.legend('x1')
#显示所画的图
plt.show()