> 本文由 [简悦 SimpRead](http://ksria.com/simpread/) 转码， 原文地址 [zhuanlan.zhihu.com](https://zhuanlan.zhihu.com/p/141018958)

懒癌患者终于来更新了。

首先我们还是考虑以下的离散的非线性随机系统：

$\begin{equation} x_{k+1}=f_k(x_k)+w_k\\ z_k =h_k(x_k)+v_k \end{equation}$\begin{equation} x_{k+1}=f_k(x_k)+w_k\\ z_k =h_k(x_k)+v_k \end{equation}

这里假设 $w_k\sim N(0,\Sigma_k^w),v_k\sim N(0,\Sigma_k^v)$w_k\sim N(0,\Sigma_k^w),v_k\sim N(0,\Sigma_k^v) .

所谓的 IEKF 就是如何改进 EKF，其直接的想法就是在 $x_k\rightarrow x_{k+1}$x_k\rightarrow x_{k+1} 的过程中进行多次迭代，以达到消减非线性的影响。

这里还需要引入如下的记号:(和 EKF 中的线性化一致)

我们先考虑如下的局部线性化：

$h_k(x_k)=h_k(\hat{x}_{k|k-1})+H_k(x_k-\hat{x}_{k|k-1})+o(x_k-\hat{x}_{k|k-1})$h_k(x_k)=h_k(\hat{x}_{k|k-1})+H_k(x_k-\hat{x}_{k|k-1})+o(x_k-\hat{x}_{k|k-1})

这里 $H_k=\frac{\partial h_k(x_k)}{\partial x_k}|_{x_k=\hat{x}_{k|k-1}}$H_k=\frac{\partial h_k(x_k)}{\partial x_k}|_{x_k=\hat{x}_{k|k-1}}

$f_k(x_k)=f_k(\hat{x}_{k|k-1})+F_k(x_k-\hat{x}_{k|k-1})+o(x_k-\hat{x}_{k|k-1})$f_k(x_k)=f_k(\hat{x}_{k|k-1})+F_k(x_k-\hat{x}_{k|k-1})+o(x_k-\hat{x}_{k|k-1})

这里 $F_k=\frac{\partial f_k(x_k)}{\partial x_k}|_{x_k=\hat{x}_{k|k-1}}$F_k=\frac{\partial f_k(x_k)}{\partial x_k}|_{x_k=\hat{x}_{k|k-1}}

下面可以给出我们的 IEKF 的算法过程了：

![](https://pic3.zhimg.com/v2-fc59cf0e2568d2768ccf0dfb3f058fc2_r.jpg)![](https://pic2.zhimg.com/v2-5737b21d29bb02018a2d1d58aa5dd5b5_r.jpg)

如上的过程其实有着统计意义上的解释，下面我们来介绍一下这件事情。

事实上我们的后验 $p(x_k|Z_k)$p(x_k|Z_k) 有着以下的性质：

$p(x_k|Z_k)\propto p(y_k|x_k)p(x_k|Z_{k-1})\propto\exp(-\frac{1}{2}((z_k-h(x_k))^{\top}(\Sigma_k^{v})^{-1}(z_k-h(x_k))+(\hat{x}_{k|k-1}-x_k)^{\top}P_{k|k-1}^{-1}(\hat{x}_{k|k-1}-x_k))$p(x_k|Z_k)\propto p(y_k|x_k)p(x_k|Z_{k-1})\propto\exp(-\frac{1}{2}((z_k-h(x_k))^{\top}(\Sigma_k^{v})^{-1}(z_k-h(x_k))+(\hat{x}_{k|k-1}-x_k)^{\top}P_{k|k-1}^{-1}(\hat{x}_{k|k-1}-x_k)) 事实上 IEKF 在极大这个后验，即它在求解如下的一个 MAP 问题：

![](https://pic3.zhimg.com/v2-1ef0718023809e46c79cb38556655d52_r.jpg)

而由于这里我们假定了后验分布是一个高斯分布，加上我们知道若误差是一个高斯分布的话，我们的回归问题（最小二乘）和极大后验是等价的。事实上 IEKF 就对应着 Gauss-Newton 方法.

这里先简要介绍一下 Gauss-Newton 方法.

**Gauss-Newton 方法**

Gauss-Newton 方法事实上要利用局部线性化解决如下的非线性回归问题：

$\arg\min_{\theta}\sum_{i=1}^n(y_i-f(x_i;\theta))^2$\arg\min_{\theta}\sum_{i=1}^n(y_i-f(x_i;\theta))^2

我们需要如下的参数更新：

$\theta^{(t)}\leftarrow\theta^{(t+1)}$\theta^{(t)}\leftarrow\theta^{(t+1)}

而在如上述步骤更新的过程，我们考虑如下的局部线性化过程：

$\begin{equation} f(x,\theta)\sim f(x,\theta^{(t)})+(\theta-\theta^{(t)})^{\top}f'(x,\theta^{(t)})=\tilde{f}(x,\theta^{(t)},\theta) \end{equation}$\begin{equation} f(x,\theta)\sim f(x,\theta^{(t)})+(\theta-\theta^{(t)})^{\top}f'(x,\theta^{(t)})=\tilde{f}(x,\theta^{(t)},\theta) \end{equation}

即求解如下的非线性回归问题：

$\begin{equation} \arg\min_{\theta^{(t+1)}}\sum_{i=1}^{n}(y_i-\tilde{f}(x_i,\theta^{(t)},\theta))^2 \end{equation}$\begin{equation} \arg\min_{\theta^{(t+1)}}\sum_{i=1}^{n}(y_i-\tilde{f}(x_i,\theta^{(t)},\theta))^2 \end{equation}

在明确了 Gauss-newton 方法之后，我们再回头来看我们的 IEKF.

我们先将记号改写如下：

$\exp(-\frac{1}{2}((z_k-h(x_k))^{\top}(\Sigma_k^{v})^{-1}(z_k-h(x_k))+(\hat{x}_{k|k-1}-x_k)^{\top}P_{k|k-1}^{-1}(\hat{x}_{k|k-1}-x_k))=V(x_k)=\frac{1}{2}r(x_k)^{\top}r(x_k)$\exp(-\frac{1}{2}((z_k-h(x_k))^{\top}(\Sigma_k^{v})^{-1}(z_k-h(x_k))+(\hat{x}_{k|k-1}-x_k)^{\top}P_{k|k-1}^{-1}(\hat{x}_{k|k-1}-x_k))=V(x_k)=\frac{1}{2}r(x_k)^{\top}r(x_k)

这里 $r(x)=\begin{bmatrix} (\Sigma_k^v)^{-\frac{1}{2}}(z-h(x)) \\ P_{t|t-1}^{-\frac{1}{2}}(\hat{x}_{t|t-1}-x) \end{bmatrix}$ r(x)=\begin{bmatrix} (\Sigma_k^v)^{-\frac{1}{2}}(z-h(x)) \\ P_{t|t-1}^{-\frac{1}{2}}(\hat{x}_{t|t-1}-x) \end{bmatrix}

即现在我们的问题变成了求解如下的不动点迭代：

$\nabla^2(V(x))p=-\nabla V(x)$\nabla^2(V(x))p=-\nabla V(x)

其标准的 Newton 迭代格式如下：

![](https://pic4.zhimg.com/v2-088267f697c8758f2bb65ede77243e27_r.jpg)

如果我们在这里考虑 Gauss-newton 方法:

![](https://pic2.zhimg.com/v2-5276f86536f96c8c5fa40c773e8dce4d_r.jpg)

这样就可以看出 Gauss-newton 迭代与我们的 IEKF 是一致的.

下面给出具体实验：

我们这次考虑以下的非线性非平稳生灭过程：

![](https://pic3.zhimg.com/v2-8f2d61f729e8e9b58d595661f0458802_r.jpg)

```
import numpy as np
T = 50 # 进行50次
Q = 1
R = 1
x = np.random.normal(0.1,1) # 初值
P = 1 # 初始协方差
xe = x
itr = 20

f = lambda x: 0.5*x + 25 * (x/(1+x**2))
h = lambda x: (x**2)/20

fg = lambda x: 0.5+ 25*((1-x**2)/(1+x**2)**2)
hg = lambda x: x/10

real = []
estimations_iekf = []
estimations_ekf = []
RMSE_ekf = []
rmse_ekf = 0
RMSE_iekf = []
rmse_iekf = 0
for t in range(1,T):
    x = 0.5 * x + 25 * x / (1 + x ** 2) + 8*np.cos(t-1) + np.random.normal(0,Q)
    z = xe ** 2 / 20 + np.random.normal(0,R)
    real.append(x)
    # prediction
    F = fg(xe)
    xe = f(xe) + 8*np.cos(t-1)
    P = F*P*F + Q
        
    # update
    x_hat = xe
    for i in range(itr):
        H = hg(xe)
        S = H*P*H + R
        K = P*H*(1/S)
        W = K*(z-h(xe)-H*(x_hat-xe))
        xe = x_hat + W
        if i == 0:
            xe_ekf = xe
            estimations_ekf.append(xe_ekf)
        P = (1-K*H)*P
    rmse_iekf = rmse_iekf + abs(xe-x)
    RMSE_iekf.append(np.sqrt(rmse_iekf**2/t))
    estimations_iekf.append(xe)
    rmse_ekf = rmse_ekf + abs(xe_ekf-x)
    RMSE_ekf.append(np.sqrt(rmse_ekf**2/t))
    
import matplotlib.pyplot as plt
#创建图形
plt.figure(1)
'''
意思是在一个2行2列共4个子图的图中，定位第1个图来进行操作（画图）。
最后面那个1表示第1个子图。那个数字的变化来定位不同的子图
'''
#第一行第一列图形
ax1 = plt.subplot(2,2,1)
#第一行第二列图形
ax2 = plt.subplot(2,2,2)
#第二行
ax3 = plt.subplot(2,1,2)
#选择ax1
plt.sca(ax1)
plt.plot(range(len(estimations_iekf)), estimations_iekf, label = 'IEKF Estimation',color = 'r')
plt.plot(range(len(real)), real, label = 'Real statement',color = 'lime')
plt.legend()
#选择ax2
plt.sca(ax2)
plt.plot(range(len(real)), real, label = 'Real statement',color = 'lime')
plt.plot(range(len(estimations_ekf)), estimations_ekf, label = 'EKF Estimation',color = 'b')
plt.legend()
#选择ax3
plt.sca(ax3)
plt.plot(range(len(estimations_ekf)), estimations_ekf, label = 'EKF Estimation',color = 'b')
plt.plot(range(len(estimations_iekf)), estimations_iekf, label = 'IEKF Estimation',color = 'r')
plt.legend()
plt.show()

plt.plot(RMSE_iekf,label='RMSE of IEKF')
plt.plot(RMSE_ekf,label='RMSE of EKF')
plt.legend()

```

![](https://pic4.zhimg.com/v2-55dd62a12a2302ec2b7efa8f1938c413_b.jpg)![](https://pic3.zhimg.com/v2-3bfe11efdf4e8d7b2fef9c1f6933f1aa_b.jpg)

在上述试验中，我们考虑了 50 个点，其中对于 IEKF，每次设置 20 次迭代，第一次迭代的值为 EKF 的估计值。从上述试验结果可以看出，虽然 Tracking 轨迹没有太大区别（因为都做的挺好的）, 但是其 RMSE 明显降低了，说明 IEKF 还是起了作用的.