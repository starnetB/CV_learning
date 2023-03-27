import numpy as np
import matplotlib.pyplot as plt  

# % matplotalib inline 

plt.style.use('ggplot')
plt.rcParams['figure.figsize']=(12,8)

# Normal distributed x and y vector with mean 0 and standard deviation 1
x=np.random.normal(0,1,200)
y=np.random.normal(0,1,200)
X=np.vstack((x,y))

# 计算二元协方差矩阵
def cov(x,y):
    xbar,ybar=x.mean(),y.mean()
    return np.sum((x-xbar)*(y-ybar))/(len(x)-1)

def cov_mat(X):
    return np.array([[cov(X[0],X[0]),cov(X[0],X[1])],
                     [cov(X[1],X[0]),cov(X[1],X[1])]])

print(cov_mat(X))

plt.scatter(X[0,:],X[1,:])
plt.title('Generated Data')
plt.axis('equal')
plt.show()