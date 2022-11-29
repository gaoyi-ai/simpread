> 本文由 [简悦 SimpRead](http://ksria.com/simpread/) 转码， 原文地址 [blog.csdn.net](https://blog.csdn.net/brightming/article/details/118057262)

**学习该文章：  
https://zhuanlan.zhihu.com/p/152662055**

状态定义
----

```
struct State {
    double timestamp;
    
    Eigen::Vector3d lla;       // WGS84 position.
    Eigen::Vector3d G_p_I;     // The original point of the IMU frame in the Global frame.
    Eigen::Vector3d G_v_I;     // The velocity original point of the IMU frame in the Global frame.
    Eigen::Matrix3d G_R_I;     // The rotation from the IMU frame to the Global frame.
    Eigen::Vector3d acc_bias;  // The bias of the acceleration sensor.
    Eigen::Vector3d gyro_bias; // The bias of the gyroscope sensor.

    // Covariance.
    Eigen::Matrix<double, 15, 15> cov;

    // The imu data.
    ImuDataPtr imu_data_ptr; 
};

```

包含：

```
Eigen::Vector3d G_p_I;     // The original point of the IMU frame in the Global frame.
    Eigen::Vector3d G_v_I;     // The velocity original point of the IMU frame in the Global frame.
    Eigen::Matrix3d G_R_I;     // The rotation from the IMU frame to the Global frame.
    Eigen::Vector3d acc_bias;  // The bias of the acceleration sensor.
    Eigen::Vector3d gyro_bias; // The bias of the gyroscope sensor.

```

5 个状态量，在协方差表示时，旋转也用三维的旋转角表示，所以，其协方差矩阵为 15。

ekf 的公式
-------

![](https://img-blog.csdnimg.cn/20210619180525661.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L2JyaWdodG1pbmc=,size_16,color_FFFFFF,t_70)

imu predict 的细节
---------------

```
void ImuProcessor::Predict(const ImuDataPtr last_imu, const ImuDataPtr cur_imu, State* state) {
    // Time.
    const double delta_t = cur_imu->timestamp - last_imu->timestamp;
    const double delta_t2 = delta_t * delta_t;

    // Set last state.
    State last_state = *state;

    // Acc and gyro.
    const Eigen::Vector3d acc_unbias = 0.5 * (last_imu->acc + cur_imu->acc) - last_state.acc_bias;
    const Eigen::Vector3d gyro_unbias = 0.5 * (last_imu->gyro + cur_imu->gyro) - last_state.gyro_bias;

    // Normal state. 
    // Using P58. of "Quaternion kinematics for the error-state Kalman Filter".
    state->G_p_I = last_state.G_p_I + last_state.G_v_I * delta_t + 
                   0.5 * (last_state.G_R_I * acc_unbias + gravity_) * delta_t2;
    state->G_v_I = last_state.G_v_I + (last_state.G_R_I * acc_unbias + gravity_) * delta_t;
    const Eigen::Vector3d delta_angle_axis = gyro_unbias * delta_t;
    if (delta_angle_axis.norm() > 1e-12) {
        state->G_R_I = last_state.G_R_I * Eigen::AngleAxisd(delta_angle_axis.norm(), delta_angle_axis.normalized()).toRotationMatrix();
    }
    // Error-state. Not needed.

    // Covariance of the error-state.   
    Eigen::Matrix<double, 15, 15> Fx = Eigen::Matrix<double, 15, 15>::Identity();
    Fx.block<3, 3>(0, 3)   = Eigen::Matrix3d::Identity() * delta_t;
    Fx.block<3, 3>(3, 6)   = - state->G_R_I * GetSkewMatrix(acc_unbias) * delta_t;
    Fx.block<3, 3>(3, 9)   = - state->G_R_I * delta_t;
    if (delta_angle_axis.norm() > 1e-12) {
        Fx.block<3, 3>(6, 6) = Eigen::AngleAxisd(delta_angle_axis.norm(), delta_angle_axis.normalized()).toRotationMatrix().transpose();
    } else {
        Fx.block<3, 3>(6, 6).setIdentity();
    }
    Fx.block<3, 3>(6, 12)  = - Eigen::Matrix3d::Identity() * delta_t;

    Eigen::Matrix<double, 15, 12> Fi = Eigen::Matrix<double, 15, 12>::Zero();
    Fi.block<12, 12>(3, 0) = Eigen::Matrix<double, 12, 12>::Identity();

    Eigen::Matrix<double, 12, 12> Qi = Eigen::Matrix<double, 12, 12>::Zero();
    Qi.block<3, 3>(0, 0) = delta_t2 * acc_noise_ * Eigen::Matrix3d::Identity();
    Qi.block<3, 3>(3, 3) = delta_t2 * gyro_noise_ * Eigen::Matrix3d::Identity();
    Qi.block<3, 3>(6, 6) = delta_t * acc_bias_noise_ * Eigen::Matrix3d::Identity();
    Qi.block<3, 3>(9, 9) = delta_t * gyro_bias_noise_ * Eigen::Matrix3d::Identity();

    state->cov = Fx * last_state.cov * Fx.transpose() + Fi * Qi * Fi.transpose();

    // Time and imu.
    state->timestamp = cur_imu->timestamp;
    state->imu_data_ptr = cur_imu;
}


```

predict 主要是对 nominal state 的运动学估计，以及对协方差的递推（为了在观测值来的时候，结合两者的协方差算出 K 值）。  
细节：  
1、gravity_这里是加号，为什么不是减号？  
2、协方差传递的 Fx 的计算，与《Quaternion Kinematics for the error-state KF》中的一致：  
（误差的传递与 nominal state 的传递都是一样的，遵循的都是相同的运动学模型，只是误差会在之前的数值的基础上继续包括新增的各种运动误差，如 imu 的 bias，随机游走等，而 nominal 则不管这些值，按照正常的运动学模型递推。）

![](https://img-blog.csdnimg.cn/20210619194157137.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L2JyaWdodG1pbmc=,size_16,color_FFFFFF,t_70)

![](https://img-blog.csdnimg.cn/2021061919403551.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L2JyaWdodG1pbmc=,size_16,color_FFFFFF,t_70)  
就以上面的 5.4.2 来说，这个 f() 包括了几个函数：  
关于 δ p \delta p δp 的函数，关于 δ v \delta v δv 的函数，共有 6 个函数，分别叫做 f1,f2,f3,f4,f5,f6 吧。  
则 Fx 的计算就是：  
[ ∂ f 1 ∂ δ p ∂ f 1 ∂ δ v ∂ f 1 ∂ δ θ . . . ∂ f 2 ∂ δ p ∂ f 2 ∂ δ v ∂ f 2 ∂ δ θ . . . . . . ∂ f 6 ∂ δ p ∂ f 6 ∂ δ v ∂ f 6 ∂ δ θ . . . ]

$$\begin{bmatrix} \frac{\partial f1}{\partial\delta p} \frac{\partial f1}{\partial\delta v} \frac{\partial f1}{\partial\delta \theta} ... \\ \frac{\partial f2}{\partial\delta p} \frac{\partial f2}{\partial\delta v} \frac{\partial f2}{\partial\delta \theta} ... \\ ... \\ \frac{\partial f6}{\partial\delta p} \frac{\partial f6}{\partial\delta v} \frac{\partial f6}{\partial\delta \theta} ... \\ \end{bmatrix}$$

⎣⎢⎢⎢⎡​∂δp∂f1​∂δv∂f1​∂δθ∂f1​...∂δp∂f2​∂δv∂f2​∂δθ∂f2​......∂δp∂f6​∂δv∂f6​∂δθ∂f6​...​⎦⎥⎥⎥⎤​

结果就是上面的公式 269。

3、Fi 的计算

Fi 代表的是误差状态传递函数对干扰项的导数。  
同样的，f1…f6 对干扰项 v i θ i a i w i v_i \\ \theta_i \\ a_i \\ w_i vi​θi​ai​wi​进行求导：  
![](https://img-blog.csdnimg.cn/20210619200117928.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L2JyaWdodG1pbmc=,size_16,color_FFFFFF,t_70)对应代码中也是如此设置：

```
 	Eigen::Matrix<double, 15, 12> Fi = Eigen::Matrix<double, 15, 12>::Zero();
    Fi.block<12, 12>(3, 0) = Eigen::Matrix<double, 12, 12>::Identity();

```

4、Qi 的计算  
Qi 代表的是干扰项的协方差，这些干扰项各自独立，自然只有对角线上有值。干扰项的含义，以及协方差的计算介绍：  
![](https://img-blog.csdnimg.cn/2021061920035127.png)![](https://img-blog.csdnimg.cn/20210619194222139.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L2JyaWdodG1pbmc=,size_16,color_FFFFFF,t_70)  
都是与 imu 相关的误差参数，这些值是可以标定出来的，例如：

```
accelerometer_noise_density: 0.012576 #continous 
accelerometer_random_walk: 0.000232  
gyroscope_noise_density: 0.0012615 #continous 
gyroscope_random_walk: 0.0000075  


```

代码中是这样设置的：

```
    Eigen::Matrix<double, 12, 12> Qi = Eigen::Matrix<double, 12, 12>::Zero();
    Qi.block<3, 3>(0, 0) = delta_t2 * acc_noise_ * Eigen::Matrix3d::Identity();
    Qi.block<3, 3>(3, 3) = delta_t2 * gyro_noise_ * Eigen::Matrix3d::Identity();
    Qi.block<3, 3>(6, 6) = delta_t * acc_bias_noise_ * Eigen::Matrix3d::Identity();
    Qi.block<3, 3>(9, 9) = delta_t * gyro_bias_noise_ * Eigen::Matrix3d::Identity();


```

这些参数，代码中是在初始化时设置的：

```
LocalizationWrapper::LocalizationWrapper(ros::NodeHandle& nh) {
    // Load configs.
    double acc_noise, gyro_noise, acc_bias_noise, gyro_bias_noise;
    nh.param("acc_noise",       acc_noise, 1e-2);
    nh.param("gyro_noise",      gyro_noise, 1e-4);
    nh.param("acc_bias_noise",  acc_bias_noise, 1e-6);
    nh.param("gyro_bias_noise", gyro_bias_noise, 1e-8);


```

```
<launch>
    <param  />
    <param  />
    <param  />
    <param  />

    <param  />
    <param  />
    <param  />

    <param  />

    <node  />
    <node  />

    <node pkg="rviz" type="rviz"  
      args="-d $(find imu_gps_localization)/ros_wrapper/rviz/default.rviz" required="true">
    </node>

</launch>

```

Qi 矩阵：  
![](https://img-blog.csdnimg.cn/20210619201346583.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L2JyaWdodG1pbmc=,size_16,color_FFFFFF,t_70)

递推协方差的计算
--------

协方差反应了递推的数据多不靠谱的程度。  
![](https://img-blog.csdnimg.cn/20210619201947781.png)  
经过上面计算的 Fx,Fi,Qi，以及上一时刻的 P，就可以计算了。  
代码中：

```
state->cov = Fx * last_state.cov * Fx.transpose() + Fi * Qi * Fi.transpose();


```

观测函数的 jacobian 的计算
------------------

gps 的观测，会得到一个位置的信息，其他都没有，就是说虽然 imu 估计了 p、v、q、ba、bg，可以将这些估计状态转换为观测值的是：  
G_p_I + G_R_I * I_p_Gps_  
其中，I_p_Gps_是一开机时计算出来的 gps 原点与 imu 的转换关系，是固定的。  
h 函数对状态量求导：  
对 G_p_I，得到 I 3 ∗ 3 I_{3*3} I3∗3​  
对 G_v_I 求导，0（3_3）  
对 G_R_I 求导，得到 - G_R_I * GetSkewMatrix(I_p_Gps_); 参考：https://zhuanlan.zhihu.com/p/156895046  
对 acc_bias 求导，得到 0（3_3）  
对 gyro_bias 求导，得到 0（3*3）。

也就是以下代码实现：

```
void GpsProcessor::ComputeJacobianAndResidual(const Eigen::Vector3d& init_lla,  
                                              const GpsPositionDataPtr gps_data, 
                                              const State& state,
                                              Eigen::Matrix<double, 3, 15>* jacobian,
                                              Eigen::Vector3d* residual) {
    const Eigen::Vector3d& G_p_I   = state.G_p_I;
    const Eigen::Matrix3d& G_R_I   = state.G_R_I;

    // Convert wgs84 to ENU frame.
    Eigen::Vector3d G_p_Gps;//测量值
    ConvertLLAToENU(init_lla, gps_data->lla, &G_p_Gps);

    // Compute residual.
    //I_p_Gps_在imu坐标系下的位移？是固定值？
    //G_p_I + G_R_I * I_p_Gps_预测的状态计算出来的坐标点
    *residual = G_p_Gps - (G_p_I + G_R_I * I_p_Gps_);

    // Compute jacobian.
    jacobian->setZero();
    jacobian->block<3, 3>(0, 0)  = Eigen::Matrix3d::Identity();
    jacobian->block<3, 3>(0, 6)  = - G_R_I * GetSkewMatrix(I_p_Gps_);
}

```

![](https://img-blog.csdnimg.cn/20210619201834729.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L2JyaWdodG1pbmc=,size_16,color_FFFFFF,t_70)

观测函数 h 在不同模型下是不同的，这里需要求出 h 对误差状态的 jacobian，为啥是对误差状态量的 jacobian 呢？  
算出这个 jacobian H，就可以与递推的 P，和观测噪声，共同计算 K 了。

上面 imu predict 的时候，是对什么进行递推？没有对具体的误差数值如 δ p \delta p δp 这些递推，只是递推了协方差的值，协方差本身就反应了误差数据，本身就代表了误差数据。

（啰嗦，还在深刻理解总结）  
我们一直在计算误差的协方差，但是我们始终假设误差的 mean 是零，这个协方差反应了我们的 nominal state 的多不靠谱，也是 error state 的表示，两个是一个意思。当观测数据到来的时候，相当于就是用 noimal state 推理的状态，映射到观测空间，与观测值计算出残差，K 乘于这个残差，就是对 nomial state 推理出来的数据的补偿。  
error state 推理的协方差，代表了 nominal state 的不可信度，观测数据的协方差，代表的是观测数据的不可信度。

nominal state 在递推，误差的协方差也在递推，在递推的过程中，只是知道越来越不可信，但是究竟数值有多少，没有客观参考，无法得知。直到观测数据到来，才能让误差现身。

![](https://img-blog.csdnimg.cn/20210619203648438.png)所谓的现身，是什么意思呢？就是将递推的数据，与观测的数据，一对比，就知道了。但是要让误差数值现身可不是随便来的，毕竟所得到的观测数据，自己也有误差啊。所以，需要 kalman 增益系数来综合两者，对观测数据和预测的状态的映射数据的差值乘上这个系数，才得到最可能的误差值。

**不明白的是为何要计算观测函数对误差状态的 jacobian。一会翻过来再理解一下 ekf**

总结一下：  
我们要用综合计算出来的 δ x ^ \delta \hat x δx^ 加到 nominal state x x x 上，得到最优的  
x ^ t \hat x_t x^t​，即  
x ^ t = x + δ x ^ \hat x_t = x + \delta \hat x x^t​=x+δx^

而 :  
δ x ^ = K ( y − h ( x ^ t ) ) \delta \hat x = K (y- h(\hat x_t)) δx^=K(y−h(x^t​))  
这里，y 表示观测值， x ^ t \hat x_t x^t​表示真值，但是这时候哪有真值，唯有用当前推理得到的 nominal state 代替，这也是合理的，因为我们还有不确定度这个数据。

而：  
K = P H t ( H P H t + V ) − 1 K= PH^t (HPH^t+V)^{-1} K=PHt(HPHt+V)−1

所以需要有 H，  
而 H 定义为观测函数对误差量的 jacobian 矩阵：  
H ≡ ∂ h ∂ δ x H \equiv \frac{\partial h}{\partial \delta x} H≡∂δx∂h​

h 其实是状态变量的函数，对误差项怎么求导？  
别忘了：  
真值，其实是 nominal state + 误差项，  
所以：

![](https://img-blog.csdnimg.cn/20210619205708659.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L2JyaWdodG1pbmc=,size_16,color_FFFFFF,t_70)  
对于 H x H_x Hx​ 其实就是一般的对状态量的 jacobian，跟这个观测方程有关。

第二项，其实就是真值对误差项目的求导，看下表：  
![](https://img-blog.csdnimg.cn/2021061920595347.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L2JyaWdodG1pbmc=,size_16,color_FFFFFF,t_70)composition 那一列：  
所以，这个 X δ x X_{\delta x} Xδx​ 的表达式是固定的：  
![](https://img-blog.csdnimg.cn/2021061921020323.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L2JyaWdodG1pbmc=,size_16,color_FFFFFF,t_70)![](https://img-blog.csdnimg.cn/20210619210309344.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L2JyaWdodG1pbmc=,size_16,color_FFFFFF,t_70)  
参考代码里的只是做了前面 H x H_x Hx​的计算，后面的 X δ x X_{\delta x} Xδx​没有计算。

```
   // Compute jacobian.
    jacobian->setZero();
    jacobian->block<3, 3>(0, 0)  = Eigen::Matrix3d::Identity();
    jacobian->block<3, 3>(0, 6)  = - G_R_I * GetSkewMatrix(I_p_Gps_);

```