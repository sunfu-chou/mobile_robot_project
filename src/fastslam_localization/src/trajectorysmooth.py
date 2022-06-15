import matplotlib.pyplot as plt
import numpy as np
from scipy import interpolate
import pandas as pd


map_data=pd.read_csv("map_redesign.csv")
yellow_data=map_data[['Yx','Yy']]
blue_data=map_data[['Bx','By']].dropna()

data=pd.read_csv(r'goal_data.csv')
arr = data.to_numpy()
#arr = np.array([[0,0],[2,.5],[2.5, 1.25],[2.6,2.8],[1.3,1.1]])
x, y = zip(*arr)
#in this specific instance, append an endpoint to the starting point to create a closed shape
x = np.r_[x, x[0]]
y = np.r_[y, y[0]]
#create spline function
f, u = interpolate.splprep([x, y], s=0, per=True)
#create interpolated lists of points
xint, yint = interpolate.splev(np.linspace(0, 0.05, 25), f)
xint1, yint1 = interpolate.splev(np.linspace(0.06, 0.44, 100), f)
xint2, yint2 = interpolate.splev(np.linspace(0.45, 0.62,50), f)
xint3, yint3 = interpolate.splev(np.linspace(0.62, 0.85, 100), f)
xint4, yint4 = interpolate.splev(np.linspace(0.85, 0.99,25), f)
x_array=np.concatenate((xint,xint1,xint2,xint3,xint4))
y_array=np.concatenate((yint,yint1,yint2,yint3,yint4))

goal_x=pd.DataFrame(x_array)
goal_y=pd.DataFrame(y_array)
goal_data=pd.concat([goal_x,goal_y],axis=1)
goal_data.to_csv("goal.csv",index=False)

plt.scatter(yellow_data['Yx'], yellow_data['Yy'],label='yellow',c='yellow')
plt.scatter(blue_data['Bx'], blue_data['By'],label='blue',c='blue')
plt.scatter(x, y,c='orange')
#plt.plot(xint, yint)
plt.scatter(x_array, y_array,c='red')
plt.show()