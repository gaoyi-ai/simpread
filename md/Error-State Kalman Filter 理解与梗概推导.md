> 本文由 [简悦 SimpRead](http://ksria.com/simpread/) 转码， 原文地址 [zhuanlan.zhihu.com](https://zhuanlan.zhihu.com/p/359014822)

最近重读了一下 ESKF 经典 [[1]](#ref_1)，对一些内容有了一些新的体会，因此更新下本文，对一些问题进行勘误。由于 ESKF 整个推导过程极为繁琐，因此本文旨在尽可能简化的抽取梗概，从大体上理解 ESKF，再结合参考文献中的细节，即可了解全貌。全文中的文献均指 [[1]](#ref_1)。

直接法与间接法
-------

首先必须理解一下直接法与间接法，才能明白 ESKF 的精髓，实际上，间接法不止应用在滤波领域，在优化方法中也一样存在，典型就是牛顿法（高斯牛顿法是牛顿法的一种扩展，本质也是间接法，这里不打算继续扩大范畴，因此下文以牛顿法举例），实际上就是因为直接法不好求，牛顿法才转而从一个初始位置求解间接量 $\Delta x$\Delta x ，详细解释如下：

假设存在待优化目标函数：

$\min_xF(x)$\min_xF(x)

x 为 n 维向量，使用直接法求解，就是直接求解 F 对 x 的导数，如下：

$\frac{dF}{dx}=0$\frac{dF}{dx}=0

求出该公式所有的解，并依次带入 F(x)，其中最小的值对应的 x 即为所求。

当 F(x) 表示比较简单时，这种解法可以很快速得到最优解，但当 F(x) 很复杂时，以上直接对状态量的一阶导数表达式可能同样很复杂，对上式的求解将变得极为困难甚至不可能，为了在复杂系统中解决这个问题，就诞生牛顿法这样的间接法，假设存在初值 $x_0$x_0 ，可以在该值附近对 F(x) 进行一到二阶泰勒展开：

$F(x_0+\Delta x)=F(x_0)+\frac{dF}{d\Delta x}_{|x_0}\Delta x+\frac{d^2F}{d\Delta x^2}_{|x_0}\Delta x^2$F(x_0+\Delta x)=F(x_0)+\frac{dF}{d\Delta x}_{|x_0}\Delta x+\frac{d^2F}{d\Delta x^2}_{|x_0}\Delta x^2

此时的问题变成了，求解使 F(x) 为极值的$\Delta x$\Delta x ，并更新 x，再继续上述过程，直到迭代收敛。

注意看，直接法中，直接求解目标函数对状态量的导数，多维时称为雅可比矩阵；间接法中，求解目标函数对于一个小值$\Delta x$\Delta x 的导数，这是最大的不同，正是基于$\Delta x$\Delta x 很小的假设，我们可以认为其高阶导数值都非常小，可以忽略，因此可以在局部区域对目标函数线性化，由此简化计算。

后面改进的高斯牛顿法、LM 算法都是基于$\Delta x$\Delta x 的特性做了一些策略和调整，这里就不展开了。

为什么需要使用间接法
----------

以上求解极值问题中其实已经解释了部分使用间接法的原因，参考文献中有更详细的解释，这里再简单总结一下：

1.  间接法核心是求解 Error-state，而非直接的系统状态量，一般而言 error-state 值非常小，在计算过程中大量的高阶导数都可以被直接忽略，造成的精度损失会非常小，也会使得雅可比矩阵非常简单、求解快速，甚至在一定时间段内可以认为是常量，这将大大加速计算。这条是间接法通用特性。
2.  在 ESKF 应用中，可以避免一些冗余参数表示，旋转矩阵存在 6 个冗余参数，四元数存在 1 个，它们都需要增加额外的约束，而 ESKF 可以直接直接旋转向量作为状态量，完全没有冗余参数。
3.  在 ESKF 应用中，Error-state 一直在 0 值附近，距离奇异点较远，可以有效避免 gimbal-lock 问题。
4.  在 ESKF 应用中，Error-state 动力学变化较小较慢，原因在于其剔除了主成分。

当然，间接法也不是万能的，理论上来讲，它并不一定比直接法好，原因在于：

1.  间接法的 Error-state 系统方程实际上是推导出来的，并且过程中作了一定近似（例如去掉了所有的高阶项，尽管我们认为这些高阶项很小），虽然完成了系统线性化，但损失了精度；而直接法的系统方程是没有损失的，其精度损失在于在局部点上采用了泰勒一阶或二阶近似对系统线性化造成的精度损失，这取决于迭代速度、系统份非线性化程度等多个因素，实际工程实践中精度还是取决于使用情况和具体实现。

总之，至少在 IMU 与其他传感器融合这个领域，业界已经证明了 ESKF 这类间接法的优势。

关于 ESKF
-------

卡尔曼滤波作为一种贝叶斯滤波的具体实现被广泛应用于状态估计问题中，其优势与特点这里就不再赘述了，而众所周知的是，卡尔曼滤波只能应用于线性系统中，而实际大部分系统无法满足这个约束，于是诞生了扩展卡尔曼滤波（Extended Kalman Filter，EKF），而 EKF 中在某个状态附近将系统线性化时，采用的是直接法，这就带来了前文讲到的各种问题，因此，针对误差状态求解的误差状态卡尔曼滤波（Error State Kalman Filter，ESKF）应运而生。

ESKF 理解与公式推导
------------

我无意于将文献中大量的公式搬到此处，因此，这里主要抽象的介绍下 ESKF 推导的梗概，以及重要的理解，具体公式可以详细阅读该参考文献。

ESKF 的核心在于将系统状态分解为两个组成部分：

$X_t=X\oplus\delta X\tag1$ X_t=X\oplus\delta X\tag1

其中 $X_t$X_t 为系统状态真值，$X$X 为 Nominal state，包含系统状态主成分，$\delta X$\delta X 为 Error state，以下所有公式中下标均符合这个定义。注意，这里是个广义加法（具体展开可以查询文献 Table3），因为状态量中包含旋转部分 R 或 q 是没有加法而只有乘法运算的。

KF 分为 predict 和 update 两个过程，分开来讲，分别对应系统动力学模型与观测模型的推导。

### predict 过程

根据 IMU 中值积分模型，可直接得到 $X_t$X_t 的一阶导数：

$$\dot X_t=U_t(X_t,u_m,i)\tag2$$

\dot X_t=U_t(X_t,u_m,i)\tag2

$u_m,i$u_m,i 为 IMU 的测量值和各类噪声，下文不重复标注了，上式对应文献中式 235。

提取所有状态量主成分，构建一个 Nominal state 的动力学模型：

$$\dot X=U(X,u_m)\tag3$$

\dot X=U(X,u_m)\tag3

这个模型是自行构建的，理论上是可以根据需要调整的，注意观测，这个模型是完全不受噪声分量影响的，因为噪声分量的影响都放到了 Error-state 的动力学模型中。上式对应文献中式 237。

有 (1)(2)(3) 就可以推导得到 Error-state 的动力学模型了，方法是对 (1) 进行求导，将 (2)(3) 带入进去求解即可，其中对于速度分量和旋转分量的推导有一些麻烦，因为设计对旋转量的求导，其他都好推。此时得到：

$$\dot{\delta X}=U_{\delta}(X,\delta x,u_m,i)\tag4$$

\dot{\delta X}=U_{\delta}(X,\delta x,u_m,i)\tag4

上式对应文献中式 261。

对 (3)(4) 积分可得到离散时间下的系统递推方程为：

$X_{k+1}=f(X_{k},u_m)\\[3mm] \delta X_{k+1}=f_{\delta}(X_k,\delta X_{k},u_m,i)\tag5$ X_{k+1}=f(X_{k},u_m)\\[3mm] \delta X_{k+1}=f_{\delta}(X_k,\delta X_{k},u_m,i)\tag5

积分过程参考文献 4.6，上式对应文献中 260 与 261。有了系统递推方程后，剩下的部分就跟 EKF 基本一模一样了，将 Error-state 方程线性化为：

$\delta X_{x+1}=F_{\delta}(X,u_m)\delta X_k+F_ii\tag 6$\delta X_{x+1}=F_{\delta}(X,u_m)\delta X_k+F_ii\tag 6

这里 、$F_{\delta}、F_i$F_{\delta}、F_i 为 $f_{\delta}$f_{\delta} 对 、$\delta X、i$\delta X、i 的雅可比矩阵，实际上根据该文献的推导，在推导误差系统方程过程中，大量的高阶项已经被忽略，这里已经是一个线性模型，不需要额外的求导操作了。这就是 predict 部分最核心的公式了，有了这个线性模型，predict 部分就退化为一般针对 Error-state 的 EKF，套用 EKF 的 predict 公式即可：

$\delta \hat X_{x+1}=F_{\delta}(X,u_m)\delta \hat X_k\\[3mm] \hat P_{k+1}=F_{\delta}\check P_kF_{\delta}^T+F_iQ_iF_i^T \tag 7$\delta \hat X_{x+1}=F_{\delta}(X,u_m)\delta \hat X_k\\[3mm] \hat P_{k+1}=F_{\delta}\check P_kF_{\delta}^T+F_iQ_iF_i^T \tag 7

### update 过程

假设观测方程为：

$Y=h(X_t)+v\tag 8$Y=h(X_t)+v\tag 8

套用 EKF 的 update 公式如下：

$K=\hat P_{k+1}H^T(H\hat P_{k+1}H^T+V)^{-1}\\[3mm] \delta \check X_{k+1}=K(Y-h(\hat X_{t,k+1}))\\[3mm] \check P_{k+1}=(I-KH)\hat P_{k+1} \tag 9$K=\hat P_{k+1}H^T(H\hat P_{k+1}H^T+V)^{-1}\\[3mm] \delta \check X_{k+1}=K(Y-h(\hat X_{t,k+1}))\\[3mm] \check P_{k+1}=(I-KH)\hat P_{k+1} \tag 9

整个过程其实跟 EKF 完全一样，ESKF 与 EKF 的唯一不同是，式子中的 H 矩阵是 h 相对于 Error-state 的雅可比矩阵。换一种更容易理解的方式，因为 $X_t=X\oplus\delta X$X_t=X\oplus\delta X ，Nominal state 已知， $h(X_t)$h(X_t) 自然就退化为针对 Error-state 的方程，结合 (7) 就是一个完全的 EKF 了。

剩下唯一一个问题就是求解 H 矩阵，这里直接采用链式法则求解即可：

$$H\triangleq \frac{dh}{dX_t}_{|X}\frac{dX_t}{d\delta X}_{|X}\tag{10}$$

H\triangleq \frac{dh}{dX_t}_{|X}\frac{dX_t}{d\delta X}_{|X}\tag{10}

具体的计算过程参考该文献吧，这里不展开了。

### 收尾过程

之后将融合得到的 Error state 注入 nominal state 用于修正误差：

$X_{k+1}\gets X_{k+1}+\delta \check {X}_{k+1}\tag{11}$ X_{k+1}\gets X_{k+1}+\delta \check {X}_{k+1}\tag{11}

因为 Error state 主成分已经注入 nominal state 了，最后需要执行一个针对 Error-state 的 reset 操作：

$$\delta X_{k+1}=g(\delta X_{k+1})=\delta X_{k+1}\ominus\delta \check X_{k+1}\tag{12}$$

\delta X_{k+1}=g(\delta X_{k+1})=\delta X_{k+1}\ominus\delta \check X_{k+1}\tag{12}

同样因为旋转没有减法的原因，这个地方使用了一个广义减法，因此多了一些其他的操作，否则直接将误差状态期望清零并保持噪声项即可。

关于 reset 过程中的协方差的处理，跟 predict 和 update 过程中模型线性化过程类似，参考该文献即可，这里也不 copy 了。

VINS 中对于 ESKF 的使用
-----------------

VINS 是基于优化的方法计算，因此它并不是一个完整的 ESKF，只是在使用 IMU 构建 t 到 t+1 时刻的约束时使用了 ESKF 的 predict 过程，这里需要注意一点，因为 IMU 只用于构建约束，因此它 predict 过程是位于 $b_k$b_k （k 时刻 IMU 的位姿）坐标系下的，而非世界坐标系，因此这个 predict 过程是局部的，VINS 第四部分公式和 VIO 部分构建 IMU 约束中可以看到，都是转到 $b_k$b_k 坐标系下计算的，这意味着这个 predict 过程的起点是零，测量积分的结果就是当时的状态。

VINS 中是根据中值积分法计算 nominal state，参考论文第四部分，这部分与 ESKF 中 predict 的 nominal state 的推导过程完全一致。Error-state 主要用于处理积分过程中的协方差累计，以及当 $\delta b_a,\delta b_g$\delta b_a,\delta b_g 发生变化时快速的更新 IMU 预积分状态。由于在前端 VIO 优化过程中会进行很多次迭代，而每次迭代会轻微的更新 $\delta b_a,\delta b_g$\delta b_a,\delta b_g ，若每次更新 $\delta b_a,\delta b_g$\delta b_a,\delta b_g 后都对 nominal state 进行重新预积分，虽然精度会较高，但计算量太大，且根据 ESKF 的原理，若 $\delta b_a,\delta b_g$\delta b_a,\delta b_g 进行较小的更新时，使用对 Error-state 的雅克比矩阵乘以更新后的值再更新系统状态并不会带来太大误差，但计算量会大大减少。 具体结合下图解释：

![](https://pic2.zhimg.com/v2-ee4ce32592fb1531238f898ffe197979_r.jpg)

黄色的线代表了 IMU 预积分构成的约束，对于 sliding window 中每一个 frame，系统都会构建一个 intergration_base 的对象用以管理它与上一个 frame 之间 IMU 预积分的结果以及对应时间段内系统状态对 error state 的雅克比矩阵 F，例如在 ，$x_k，x_{t}$x_k，x_{t} 之间存在 n 个 IMU 测量值 $u_n$u_n ，可以由中值积分得到 nominal state：

$x_{t}^k=f(u_0,u_1...u_n,b_a,b_g)\tag{13}$ x_{t}^k=f(u_0,u_1...u_n,b_a,b_g)\tag{13}

因为针对 Error-state 的雅可比矩阵是与 nominal state 中的 R 以及当前测量值相关的，因此，每次得到当时的 nominal state 时，同时计算当前的雅各比矩阵 $F_t$F_t ，又由于 Error-state 的推导过程实质上已经线性化了，需要得到 t 到 t+1 时刻的雅各比矩阵只需要将雅各比矩阵累乘：

$F_{k+1}^k = F_n*F_{n-1}*...F_0\tag{14}$ F_{k+1}^k = F_n*F_{n-1}*...F_0\tag{14}

上式的 F 就包含了论文中（5）式需要的全部雅可比矩阵，先计算期望，噪声额外处理，于是有：

$\delta \hat x_{k+1}^k=F_{k+1}^k\delta \hat x_{k}^k \tag {15}$\delta \hat x_{k+1}^k=F_{k+1}^k\delta \hat x_{k}^k \tag {15}

最终有：

$x_{t,k+1}^k = x_{k+1}^k \oplus \delta \hat x_{k+1}^k \tag{16}$ x_{t,k+1}^k = x_{k+1}^k \oplus \delta \hat x_{k+1}^k \tag{16}

其中 $\delta x_{k+1}=F_{k+1}\delta x_{k}$\delta x_{k+1}=F_{k+1}\delta x_{k} ，其中优化开始前，默认 $\delta x_{k}=0$\delta x_{k}=0 ，因此上式变为：

$x_{t,k+1}^k = x_{k+1}^k \tag{17}$ x_{t,k+1}^k = x_{k+1}^k \tag{17}

若 VIO 运行优化的迭代过程中，对 $\delta x_{k}$\delta x_{k} 进行了更新，使其不再为 0，此时通过式 (15)(16) 快速更新 $\Delta x_{t,k+1}$\Delta x_{t,k+1} ，可以大大减小计算量。在优化完成后，再通过全部重新积分的方式更新 $x_{k+1}^k$x_{k+1}^k 以保证更高的精度。

参考
--

1.  ^[a](#ref_1_0)[b](#ref_1_1)Joan Sola ,《Quaternion kinematics for the error-state Kalman filter》