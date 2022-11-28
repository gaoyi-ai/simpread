> 本文由 [简悦 SimpRead](http://ksria.com/simpread/) 转码， 原文地址 [blog.csdn.net](https://blog.csdn.net/brightming/article/details/117969841)

目的是利用 lidar 形成一帧[点云](https://so.csdn.net/so/search?q=%E7%82%B9%E4%BA%91&spm=1001.2101.3001.7020)时间段内的 imu 数据，补偿由于运动造成的 lidar 的畸变，能够把一帧所有的点云的坐标统一到结束时刻的坐标系下。

一、算法过程
------

假设已经得到了这一段时间的 imu 数据，以第一个 imu 的坐标为基准坐标，不断的递推计算其他时刻的 imu 相对于这个基准坐标的变化姿态，直到最后一个 imu 的数据算出最后一个姿态相对于第一个 imu 的坐标系的变化。  
这里涉及到左乘还是右乘的问题：由于 imu 得到的数值都是基于自身当下的坐标系的值，也就是说，是围绕这动轴在转的，需要用右乘的方式，及下一时刻的坐标姿态等于上一时刻的坐标姿态乘上两时刻间 imu 计算的变换。  
T n + 1 = T n ∗ d e l t a T T_{n+1}=T_{n}*deltaT Tn+1​=Tn​∗deltaT

比如说：  
以第 0 个 imu 为基准坐标，计算第一个 imu 相对于第 0 时刻的 imu 坐标系的变换：  
在这两个时刻之间，根据 imu 的角速度、线加速度，以及时间间距，可以得出 imu 在这两个时刻之间的旋转角度、距离：  
ω \omega ω 和 t。  
同个角度可以转换为旋转矩阵，那么，1 时刻的姿态就是：  
T 1 = T 0 ∗ d e l t a R o t 1 + t 1 T_1 = T_0 * deltaRot1 + t1 T1​=T0​∗deltaRot1+t1  
同样，在 2 时刻：  
T 2 = T 1 ∗ d e l t a R o t 2 + t 2 T_2 = T_1 * deltaRot2 + t2 T2​=T1​∗deltaRot2+t2

一直到最后一帧 imu。  
累加起来就是：  
T n = ( ( ( T 0 ∗ d e l t a R o t 1 + t 1 ) ∗ d e l t a R o t 2 + t 2 ) ∗ d e l t a R o t 3 + t 3 ) . . . . T_n = (((T_0*deltaRot1+t1)*deltaRot2+t2)*deltaRot3+t3).... Tn​=(((T0​∗deltaRot1+t1)∗deltaRot2+t2)∗deltaRot3+t3)....

如果是绕固定轴旋转，那么就是左乘了，即变化矩阵 * 上一时刻坐标姿态。

二、结合代码
------

在 livox_horizon_loam 中，对于利用 imu 补偿也是这样做的。  
这个代码仅仅是对旋转做了补偿，未对位移进行计算，因为位移需要有速度，但是仅仅依靠 imu，长时间积分速度会很不准，需要利用反馈信息来纠正，当前没有这样做，所以仅仅是利用了旋转量。

该算法每次都累计若干个 imu，对应一帧 lidar 数据。  
为了对这一帧的 lidar 点进行运动补偿，需要计算出这一帧时间内，旋转了多少。  
代码在：

### 主流程代码：

```
void ImuProcess::Process(const MeasureGroup &meas) {
  ROS_ASSERT(!meas.imu.empty());
  ROS_ASSERT(meas.lidar != nullptr);
  ROS_DEBUG("Process lidar at time: %.4f, %lu imu msgs from %.4f to %.4f",
            meas.lidar->header.stamp.toSec(), meas.imu.size(),
            meas.imu.front()->header.stamp.toSec(),
            meas.imu.back()->header.stamp.toSec());

  auto pcl_in_msg = meas.lidar;

  if (b_first_frame_) {
    /// The very first lidar frame

    /// Reset
    Reset();

    /// Record first lidar, and first useful imu
    last_lidar_ = pcl_in_msg;
    last_imu_ = meas.imu.back();

    ROS_WARN("The very first lidar frame");

    /// Do nothing more, return
    b_first_frame_ = false;
    return;
  }

  /// Integrate all input imu message
  IntegrateGyr(meas.imu);

  /// Compensate lidar points with IMU rotation
   Initial pose from IMU (with only rotation)
  SE3d T_l_c(gyr_int_.GetRot(), Eigen::Vector3d::Zero());
  dt_l_c_ =
      pcl_in_msg->header.stamp.toSec() - last_lidar_->header.stamp.toSec();
   Get input pcl
  pcl::fromROSMsg(*pcl_in_msg, *cur_pcl_in_);

  /// Undistort points
  //因为积分的时候，是以begin为基准的，得到其他时刻相对于begin时刻的坐标变化
  //而我们要补偿的是补偿到end时刻，所以在执行的时候要注意
  Sophus::SE3d T_l_be = T_i_l.inverse() * T_l_c * T_i_l;//end to begin的姿态变化
  pcl::copyPointCloud(*cur_pcl_in_, *cur_pcl_un_);
  UndistortPcl(cur_pcl_un_, dt_l_c_, T_l_be);
。。。
}

```

```
void ImuProcess::IntegrateGyr(
    const std::vector<sensor_msgs::Imu::ConstPtr> &v_imu) {
  /// Reset gyr integrator
  gyr_int_.Reset(last_lidar_->header.stamp.toSec(), last_imu_);
  /// And then integrate all the imu measurements
  for (const auto &imu : v_imu) {
    gyr_int_.Integrate(imu);
  }
  ROS_INFO("integrate rotation angle [x, y, z]: [%.2f, %.2f, %.2f]",
           gyr_int_.GetRot().angleX() * 180.0 / M_PI,
           gyr_int_.GetRot().angleY() * 180.0 / M_PI,
           gyr_int_.GetRot().angleZ() * 180.0 / M_PI);
}

```

```
void GyrInt::Integrate(const sensor_msgs::ImuConstPtr &imu) {
  /// Init
  if (v_rot_.empty()) {
    ROS_ASSERT(start_timestamp_ > 0);
    ROS_ASSERT(last_imu_ != nullptr);

    /// Identity rotation
    //处理数据时，总是把过程数据清空，把第一个imu对应的旋转矩阵设置为参考姿态，即无旋转。
    v_rot_.push_back(SO3d());

    /// Interpolate imu in
    sensor_msgs::ImuPtr imu_inter(new sensor_msgs::Imu());
    double dt1 = start_timestamp_ - last_imu_->header.stamp.toSec();
    double dt2 = imu->header.stamp.toSec() - start_timestamp_;
    ROS_ASSERT_MSG(dt1 >= 0 && dt2 >= 0, "%f - %f - %f",
                   last_imu_->header.stamp.toSec(), start_timestamp_,
                   imu->header.stamp.toSec());
    double w1 = dt2 / (dt1 + dt2 + 1e-9);
    double w2 = dt1 / (dt1 + dt2 + 1e-9);

    const auto &gyr1 = last_imu_->angular_velocity;
    const auto &acc1 = last_imu_->linear_acceleration;
    const auto &gyr2 = imu->angular_velocity;
    const auto &acc2 = imu->linear_acceleration;

    imu_inter->header.stamp.fromSec(start_timestamp_);
    imu_inter->angular_velocity.x = w1 * gyr1.x + w2 * gyr2.x;
    imu_inter->angular_velocity.y = w1 * gyr1.y + w2 * gyr2.y;
    imu_inter->angular_velocity.z = w1 * gyr1.z + w2 * gyr2.z;
    imu_inter->linear_acceleration.x = w1 * acc1.x + w2 * acc2.x;
    imu_inter->linear_acceleration.y = w1 * acc1.y + w2 * acc2.y;
    imu_inter->linear_acceleration.z = w1 * acc1.z + w2 * acc2.z;

    v_imu_.push_back(imu_inter);
  }

  ///
  const SO3d &rot_last = v_rot_.back();
  const auto &imumsg_last = v_imu_.back();
  const double &time_last = imumsg_last->header.stamp.toSec();
  Eigen::Vector3d gyr_last(imumsg_last->angular_velocity.x,
                           imumsg_last->angular_velocity.y,
                           imumsg_last->angular_velocity.z);
  double time = imu->header.stamp.toSec();
  Eigen::Vector3d gyr(imu->angular_velocity.x, imu->angular_velocity.y,
                      imu->angular_velocity.z);
  assert(time >= 0);
  double dt = time - time_last;
  auto delta_angle = dt * 0.5 * (gyr + gyr_last);
  auto delta_r = SO3d::exp(delta_angle);

  

  SO3d rot = rot_last * delta_r;

  v_imu_.push_back(imu);
  v_rot_.push_back(rot);
}

```

注意这里的积分右乘：

SO3d rot = rot_last * delta_r;
------------------------------

imu 的积分就是这些了，每次都是相当于从 0 开始，计算出相对变化姿态。

接下来就是利用得到的姿态对 lidar 进行补偿了。

### 对 lidar 进行补偿

这里需要注意：得到的是结束时刻对于起始时刻的姿态变化，而我们要做的是把所有点云坐标统一到结束时刻的坐标系下。

T b e 表 示 e n d 时 刻 相 对 b e g i n 时 刻 的 姿 态 变 化 ， 其 他 下 标 都 是 这 个 含 义 。 T_{be} 表示 end 时刻相对 begin 时刻的姿态变化，其他下标都是这个含义。 Tbe​表示 end 时刻相对 begin 时刻的姿态变化，其他下标都是这个含义。

```
void ImuProcess::UndistortPcl(const pcl::PointCloud<RsPointXYZIRT>::Ptr &pcl_in_out,
                              double dt_be, const Sophus::SE3d &Tbe) {
  const Eigen::Vector3d &tbe = Tbe.translation();
  Eigen::Vector3d rso3_be = Tbe.so3().log();
  double first_time=pcl_in_out->points[0].timestamp;
  for (auto &pt : pcl_in_out->points) {
    // int ring = int(pt.intensity);
    float dt_bi = pt.timestamp-first_time;//pt.intensity - ring;

    if (dt_bi == 0) laserCloudtmp->push_back(pt);
    double ratio_bi = dt_bi / dt_be;
    /// Rotation from i-e
    double ratio_ie = 1 - ratio_bi;

    Eigen::Vector3d rso3_ie = ratio_ie * rso3_be;
    SO3d Rie = SO3d::exp(rso3_ie);

    /// Transform to the 'end' frame, using only the rotation
    /// Note: Compensation direction is INVERSE of Frame's moving direction
    /// So if we want to compensate a point at timestamp-i to the frame-e
    /// P_compensate = R_ei * Pi + t_ei
    Eigen::Vector3d tie = ratio_ie * tbe;
    // Eigen::Vector3d tei = Eigen::Vector3d::Zero();
    Eigen::Vector3d v_pt_i(pt.x, pt.y, pt.z);
    Eigen::Vector3d v_pt_comp_e = Rie.inverse() * (v_pt_i - tie);

    /// Undistorted point
    pt.x = v_pt_comp_e.x();
    pt.y = v_pt_comp_e.y();
    pt.z = v_pt_comp_e.z();
  }
}

```

对于这句话的说明：

```
Eigen::Vector3d v_pt_comp_e = Rie.inverse() * (v_pt_i - tie);

```

p e = R e i ∗ p i + t e i p_e=R_{ei}*p_i+t_{ei} pe​=Rei​∗pi​+tei​

R i e ∗ p e + t i e = p i ⇒ p e = R i e − 1 p i − t i e R_{ie}*p_e+t_{ie}=p_i ⇒ p_e=R_{ie}^{-1}p_i-t_{ie} Rie​∗pe​+tie​=pi​⇒pe​=Rie−1​pi​−tie​