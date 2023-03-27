# 平稳分布基本概念
1. 我们先看下面的这个公式
$$\pi (x^*)=\int \pi(x) p(x->x^*) dx$$
2. 当存在$p(x->x^*)$使得$\pi(x)=\pi(x^*)$，并且不变时，那么这时候$\pi$表示走向平稳了,$\pi$我们称为平稳分布。
3. 由于存在平稳分布的概念，我们可以构造一个马氏链，使得它趋向于一个目标平稳分布p(z),我们就可以采集p(z)的多个采样点了。精彩
4. 现在的问题，就在于我们如果构建这个马氏链了。

# 细节平衡的概念
1. 公式如下
$$\pi(x) P(x->x^*)=\pi(x^*) P(x^*->x)$$
以上公式是平稳分布的充分非必要条件，细节平衡公式可以推导平稳分布，但反过来就不行了！,如下推导过程。    
$$\begin{aligned}
    \int \pi(x) P(x->x^*) dx&=\int \pi(x^*) P(x^*->x) dx \\
                            &=\pi(x^*) \int \pi(x^*->x) dx \\ 
                            &=\pi(x^*)
  \end{aligned}$$
$\int \pi(x^*->x)=1$因为状态转移矩阵的一行等于1  
总上所诉，可以通过细节平衡推出平稳分布。 

# 目标内容
1. 我们目标公式如下所示：  
$$ \begin{aligned}
p(z)->E[f(z)]&=\int_z f(z)p(z)dx
             &=\frac{1}{N} \sum_{i=1}^N f({z^{i}})
   \end{aligned}$$
2. 我们对应平稳分布应该是$\pi(x)$,通过马尔科夫链就可以得到一系列点了
3. 找到$P(x->x^*)$的方法。公式如下：
$$p(z)Q(z->z^*) \neq p(z^*)Q(z^*->z)$$   
下面就是如何构造一个Q使得以上公式可以相等呢？   
4. 进行以下改造：
$$p(z)Q(z->z^*) \alpha (z,z^*)=p(z^*) Q(z^*->z)\alpha (z^*,z)$$
$$p(z)p(z->z^*) = p(z^*)p(z^*->z)$$
$$\alpha(z,z^*) =min(1,\frac{p(z^*)Q(z^*->z)}{p(z)Q(z->z^*)}) $$
5. 代入推导
$$\begin{aligned}
    &p(z)Q(z->z^*) \alpha (z,z^*)  \\
    &=p(z)Q(z->z^*) min(1,\frac{p(z^*)Q(z^*->z)}{p(z)Q(z->z^*)}) \\
    &=min(p(z)Q(z->z^*),{p(z^*)Q(z^*->z)}) \\
    &=min(\frac{p(z)Q(z->z^*)}{p(z^*)Q(z^*->z)},1) p(z^*)Q(z^*->z) \\
    &=p(z^*) Q(z^*->z) alpha(z^*,z)
    
\end{aligned}$$
我们最后的工作就是要构造一个Q和一个$\alpha$

# MCMC过程(MH算法)
1. Metropolis-Hasting:  
   * u~U(0,1)均匀分布
   * $z^i$~$Q(z|z^{i-1})$
   * $\alpha =min(1,\frac{p(z^*)Q(z^*->z)}{p(z) Q(z->z^*)})$
     * $p(z)=\frac {\hat p(z)}{z_p},这里的z_p是概率的归一化因子，我们做了假设$
   * if $u\leq \alpha ,z^{i}=z^*$
   * else $z^{i}=z^{i-1}$  
   * 之前的z这里重复了一下
   * 最后采集获得数据
   * $\N \in (z^1,z^2,\cdots,z^n)$
   * 这里的Q，根据需要合理的获取一个随机分布就可以了
