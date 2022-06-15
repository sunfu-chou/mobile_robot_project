#! /usr/bin/env python 

import pandas as pd
import numpy as np
import math
# import matplotlib.pyplot as plt

def cart(g_x,g_y):
    x=g_x.copy()
    y=g_y.copy()
    c_1=[]
    c_2=[]
    c_3=[]
    c_4=[]
    for i in range(len(g_x)):
        if x[i]>14.37:
            if y[i]>14.765:
                c_1.append(x[i])
                c_1.append(y[i])
            else:
                c_4.append(x[i])
                c_4.append(y[i])
        else:
            if y[i]>14.765:
                c_2.append(x[i])
                c_2.append(y[i])
            else:
                c_3.append(x[i])
                c_3.append(y[i])
    c_1=np.array(c_1).reshape([int(len(c_1)/2),2])
    c_2=np.array(c_2).reshape([int(len(c_2)/2),2])
    c_3=np.array(c_3).reshape([int(len(c_3)/2),2])
    c_4=np.array(c_4).reshape([int(len(c_4)/2),2])
    c_1=(pd.DataFrame(c_1,columns=['x','y'])).sort_values(by=["y"],ascending=True)
    c_2=(pd.DataFrame(c_2,columns=['x','y'])).sort_values(by=["y"],ascending=False)
    c_3=(pd.DataFrame(c_3,columns=['x','y'])).sort_values(by=["y"],ascending=False)
    c_4=(pd.DataFrame(c_4,columns=['x','y'])).sort_values(by=["y"],ascending=True)
    
    data=pd.concat([c_1, c_2,c_3,c_4], ignore_index=True)
    print(data)
    return data

map_data=pd.read_csv("map_redesign.csv")
yellow_data=map_data[['Yx','Yy']]
blue_data=map_data[['Bx','By']].dropna()
g_x=[]
g_y=[]
for i in range (len(blue_data)):
    dist=[]
    blue_x=blue_data['Bx'].copy()
    blue_y=blue_data['By'].copy()
    for j in range (len(yellow_data)):
        yellow_x=yellow_data['Yx'].copy()
        yellow_y=yellow_data['Yy'].copy()
        dis=math.sqrt((blue_x[i]-yellow_x[j])**2+(blue_y[i]-yellow_y[j])**2)
        dist.append(dis)
    for t in range(2):
        for j in range(len(yellow_x)):
            if dist[j]==min(dist):
                if blue_y[i]>13.5 and blue_y[i]<16:
                    x=(0.5*blue_x[i]+0.5*yellow_x[j])
                    y=(0.5*blue_y[i]+0.5*yellow_y[j])
                else:
                    x=(0.5*blue_x[i]+0.5*yellow_x[j])
                    y=(0.5*blue_y[i]+0.5*yellow_y[j])
                dist[j]=1000
                g_x.append(x)
                g_y.append(y)
                break
data=cart(g_x, g_y)
g_x=list(data["x"].copy())
g_y=list(data["y"].copy())
goal_x=[]
goal_y=[]

for i in range(len(g_x)):
    goal_x.append(g_x[i])
    goal_y.append(g_y[i])
    a=i+1
    if a==len(g_x):
        a=0
    else:
        dis=math.sqrt((g_x[i]-g_x[a])**2+(g_y[i]-g_y[a])**2)
        if dis>0.6:
            x=(0.6*g_x[i]+0.4*g_x[a])
            y=(0.6*g_y[i]+0.4*g_y[a])
            goal_x.append(x)
            goal_y.append(y)
        else:
            continue
goal_x=pd.DataFrame(goal_x,columns=['x'])
goal_y=pd.DataFrame(goal_y,columns=['y'])
goal_data=pd.concat([goal_x,goal_y],axis=1)
goal_data.to_csv("goal_data.csv",index=False)
A=pd.read_csv("goal_data.csv")
# plt.title("Route")
# plt.scatter(yellow_x, yellow_y,label='yellow',c='yellow')
# plt.scatter(blue_x, blue_y,label='blue',c='blue')
#plt.scatter(g_x, g_y,label='red',c='red')
# plt.scatter(goal_x, goal_y,label='red',c='red')
#plt.legend(loc="lower left")
# plt.show()
