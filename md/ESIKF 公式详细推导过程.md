> 本文由 [简悦 SimpRead](http://ksria.com/simpread/) 转码， 原文地址 [zhuanlan.zhihu.com](https://zhuanlan.zhihu.com/p/488599232)

EKF 在 KF 的基础上适配了非线性系统，使用的方法是将递推函数和观测函数在局部线性化，这就意味这使用 EKF 的时候存在前提假设，首先是 predict 的步长不能够太长，否则局部线性化误差会很大；其次 update 阶段需要保证 predict 的结果与真值基本接近，因为观测函数是在 predict 的结果附近进行线性化的，如果 predict 结果与真值相差很大，同样存在线性化误差很大的问题。因此，ESIKF 使用迭代的方式对其进行了优化。

ESIKF 在进入 update 阶段时，并不是直接对观测模型线性化，而是基于贝叶斯公式先将先验概率密度函数（predct 的结果）与似然概率密度函数相乘（观测函数）得到后验概率密度函数 $p(x_k|z_k)$p(x_k|z_k) ，这时观测模型仍然是非线性且无损的，此后直接基于 MAP 的思路对后验概率密度函数 $p(x_k|z_k)$p(x_k|z_k) 处理，将其转换为一个最小二乘问题，细节参考 [Performance evaluation of iterated extended Kalman filter with variable step-length](https://link.zhihu.com/?target=https%3A//iopscience.iop.org/article/10.1088/1742-6596/659/1/012022/pdf%23pdfjs.action%3Ddownload) 和[迭代扩展卡尔曼滤波 (IEKF)](https://zhuanlan.zhihu.com/p/141018958)，这里补上构建了最小二乘问题后的详细公式推导过程：

![](https://pic4.zhimg.com/v2-2287d4a060355c27799c5afef2ddb287_r.jpg)

其中计算 K 的第一行就是 fast-lio 中的核心贡献，H 矩阵不需要再求逆，由于观测较多的时候 H 矩阵会异常巨大，使用这个形式的卡尔曼增益可以大量节省计算量。