# 引言
这两天在尝试使用opencv的cv::StereoSGBM::create()的api，但很多地方无法理解，因此专门花时间学习了立体匹配的相关内容。看了很多博客和文献终于有了拨开云雾见天日的感觉。这里做一点的总结，当然这里只是介绍了opencv的cv::StereoSGBM::create()的相关算法内容。

## 代价函数
双目相机存在两幅相似的图片，分别是一左一右，如下图所示![极限搜索示意图](https://img-blog.csdnimg.cn/c6b0d6ffbabd498caa6f45401f8646dc.png?x-oss-process=image/watermark,type_d3F5LXplbmhlaQ,shadow_50,text_Q1NETiBAd2VpeGluXzM4NDk4NjI5,size_18,color_FFFFFF,t_70,g_se,x_16#pic_center)上图来源于[博客](https://blog.csdn.net/He3he3he/article/details/101162766)        


火柴的初心,上面详细介绍了立体匹配算法的相关内容，非常感谢博主。        
对于左图中每一个像素点$I_l(x,y)$，在右图中我们都可以找到对应的像素点$I_r(x,y)$,但实际上左右图是存在一定的视差的，我们将这个视差称为d，那么如果将视差计算进去的话，右图对应的像素应该就是$I_r(x-d,y)$,现在我们取$d\in[0,1,\cdots,15]$的话，我们对应没一个像素就存在一个对应的$I_r(x-d,y)$。

##### AD代价函数
这里先从AD代价函数开始了解
$$c(x,y,d)=|I_l(x,y)-I_r(x-d,y)$$
我们可以看到，对应与没一个像素点/d都存在一个C(p,d)值与之对因，但实际上d是一个数组，有多个值，于是上面代价函数可以使用以下方块表示。     

![在这里插入图片描述](https://img-blog.csdnimg.cn/908ce4226afd4a27a3db4855e6aa749e.png?x-oss-process=image/watermark,type_d3F5LXplbmhlaQ,shadow_50,text_Q1NETiBAd2VpeGluXzM4NDk4NjI5,size_14,color_FFFFFF,t_70,g_se,x_16#pic_center)      


这里x,y代表像素点的位置，d表示视差，每个小方格代表一个代价值。我们目标有两个，一个就是要优化这个代价空间，另外一个就是要从代价空间中获取每个(x,y)的正确视差，也就是代价值最小的d。

##### BT代价函数
BT的代价也是像素灰度值差值的绝对值，不同之处在于BT利用了亚像素的灰度信息。也是使用了插值。这里参考了[博客](https://blog.csdn.net/qq_27606639/article/details/108831965)
![在这里插入图片描述](https://img-blog.csdnimg.cn/a140cac210c74b14a1a7223659c0fffc.png?x-oss-process=image/watermark,type_d3F5LXplbmhlaQ,shadow_50,text_Q1NETiBAd2VpeGluXzM4NDk4NjI5,size_18,color_FFFFFF,t_70,g_se,x_16#pic_center)     


![在这里插入图片描述](https://img-blog.csdnimg.cn/660d27df19744b70b0d33d5f86a64431.png?x-oss-process=image/watermark,type_d3F5LXplbmhlaQ,shadow_50,text_Q1NETiBAd2VpeGluXzM4NDk4NjI5,size_18,color_FFFFFF,t_70,g_se,x_16#pic_center)     

## 代价聚合     
Box Filtering这个其实就是均值滤波      
![在这里插入图片描述](https://img-blog.csdnimg.cn/5844406490e849bcbc574792ff12d19e.png?x-oss-process=image/watermark,type_d3F5LXplbmhlaQ,shadow_50,text_Q1NETiBAd2VpeGluXzM4NDk4NjI5,size_18,color_FFFFFF,t_70,g_se,x_16#pic_center)     

双边滤波  
![在这里插入图片描述](https://img-blog.csdnimg.cn/e3a9a3a7b9454f2a970339dfa7b5a401.png?x-oss-process=image/watermark, type_d3F5LXplbmhlaQ,shadow_50,text_Q1NETiBAd2VpeGluXzM4NDk4NjI5,size_18,color_FFFFFF,t_70,g_se,x_16#pic_center)   

下面是全局的能量函数滤波，这个的话，需要在对一同一d在不同的视差下进行滤波，但是由于时间复杂度太高，因此一般不再实际中使用。     

![在这里插入图片描述](https://img-blog.csdnimg.cn/37dd8653ba0e4970a5e0142bca07a1cb.png?x-oss-process=image/watermark,type_d3F5LXplbmhlaQ,shadow_50,text_Q1NETiBAd2VpeGluXzM4NDk4NjI5,size_17,color_FFFFFF,t_70,g_se,x_16#pic_center)         

由于基于能量函数的话时间复杂度比较高，因此我们往往采用线扫描方式进行优化。  每次优化的时候我们从边上进行动态规划，记录上一次的值，并对每一层d对应代价函数进行优化。      

![在这里插入图片描述](https://img-blog.csdnimg.cn/dadbf3f717d9428f92c79abeca147525.png?x-oss-process=image/watermark,type_d3F5LXplbmhlaQ,shadow_50,text_Q1NETiBAd2VpeGluXzM4NDk4NjI5,size_18,color_FFFFFF,t_70,g_se,x_16#pic_center)      
 
## WTA
这个简单，就是对于每个像素点上，找到对应的代价值最小的点作为我们需要的视差值。     
![在这里插入图片描述](https://img-blog.csdnimg.cn/d7f636ced8464fe6849a9391d8a27cb8.png?x-oss-process=image/watermark,type_d3F5LXplbmhlaQ,shadow_50,text_Q1NETiBAd2VpeGluXzM4NDk4NjI5,size_18,color_FFFFFF,t_70,g_se,x_16#pic_center)        


## 最后是左右匹配问题   
![在这里插入图片描述](https://img-blog.csdnimg.cn/38406ef19710450f9de15daf9e25a7b4.png?x-oss-process=image/watermark,type_d3F5LXplbmhlaQ,shadow_50,text_Q1NETiBAd2VpeGluXzM4NDk4NjI5,size_18,color_FFFFFF,t_70,g_se,x_16#pic_center)         















