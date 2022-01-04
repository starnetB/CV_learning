import numpy as np
import matplotlib.pyplot as plt


plt.style.use('ggplot')  
plt.rcParams['figure.figsize']=(12,8)

#Normal distributed x and y vector with mean 0 and standard deviation 1
x=np.random.normal(0,1,200)
y=np.random.normal(0,1,200)
X = np.vstack((x, y)) # 2xn]

sx, sy = 0.5, 2.0  
Scale = np.array([[sx, 0], [0, sy]])

# Rotation maxtrix  
theta=np.pi/6

c,s=np.cos(theta),np.sin(theta)
Rot=np.array([[c,-s],[s,c]])

#Transformation matrix
T=Rot.dot(Scale)

#Apply transformation matrix to X
Y=T.dot(X)

# 原始点集
plt.scatter(X[0, :], X[1, :])
# 缩放、旋转后
plt.scatter(Y[0, :], Y[1, :])
plt.title('Generated Data')
plt.axis('equal')
plt.show()
