# 正態分布變換配准(NDT)
正态分布变换( Normal Distributions Transform )进行配准   
用正态分布变换算法来确定两个大型点云（都超过 100 000个点）之间的刚体变换。正态分布变换算法是一个配准算法 , 它应用于 三维点的统计模型，使用标准最优化技术来确定两个点云间的最优的匹配，因为其在配准过程中不利用对应点的特征计算和匹配，所以时间比其他方法快，更多关于正态分布变换算法的详细的信息，请看 Martin Magnusson 博士的博士毕业论文“The Three-Dimensional Normal Distributions Transform – an Efficient Representation for Registration, Surface Analysis, and Loop Detection.”。  
*  相关代码为PCL-Demo中的