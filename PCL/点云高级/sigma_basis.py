import numpy as np  
import matplotlib.pyplot as plt 

plt.style.use('ggplot')
plt.rcParams['figure.figsize']=(12,8)

#Normal distributed x and y vector with mean 0 and standard deviation 1
x=np.random.normal(0,1,200)
y=np.random.normal(0,1,200)
X=np.vstack((x,y))


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

# 计算二元协方差矩阵
def cov(x,y):
    xbar,ybar=x.mean(),y.mean()
    return np.sum((x-xbar)*(y-ybar))/(len(x)-1)

def cov_mat(X):
    return np.array([[cov(X[0],X[0]),cov(X[0],X[1])],
                     [cov(X[1],X[0]),cov(X[1],X[1])]])

C=cov_mat(Y)
print(C)  

eVa,eVe=np.linalg.eig(C)

plt.scatter(Y[0, :], Y[1, :]) 

# 画出基  
for value, eigen in zip(eVa, eVe.T):
    plt.plot(
        [0, 3 * np.sqrt(value) * eigen[0]],
        [0, 3 * np.sqrt(value) * eigen[1]],
         lw=5)

plt.title('Transformed Data')
plt.axis('equal')
plt.show()