# EKF由哪些局限性呢？     
1. 马尔卡夫模型，也就是k时刻的状态只与k-1时刻相关,前面的信息往往丢失李
2. EKF滤波其仅仅在k-1时刻作了一次线性化  
3. EKF要存储状态量的均值和方差，并对他们进行维护，所以存储量很大
4. EKF等滤波方法，没有异常检测机制。
* 但是在计算资源有限，或带估计量比较简单的场合，EKF仍不失为一种有效的方式    
* P241