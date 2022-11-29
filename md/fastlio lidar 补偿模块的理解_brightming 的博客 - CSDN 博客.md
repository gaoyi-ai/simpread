> 本文由 [简悦 SimpRead](http://ksria.com/simpread/) 转码， 原文地址 [blog.csdn.net](https://blog.csdn.net/brightming/article/details/117519931?spm=1001.2014.3001.5502)

基本原理
----

![](https://img-blog.csdnimg.cn/20210603135844335.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L2JyaWdodG1pbmc=,size_16,color_FFFFFF,t_70)  
在上面的 sync_packages 代码中，已经拿到了符合要求的 imu 数据和 lidar 数据，现在，就基于这些数据对 lidar 点进行补偿，目的是补偿到结束时刻的 lidar 坐标系下。

坐标系介绍
-----

总共有 3 个坐标系：  
全局坐标系（world）、imu 坐标系（body）、lidar 坐标系

全局坐标系一般以开机点为原点；  
imu 坐标系时刻在变化，是一个纯粹以 imu 来推理的坐标系；  
lidar 坐标系：如果 lidar 与 imu 是刚性连接的，那么 lidar 坐标系与 imu 坐标系的转换关系就是固定的，知道了 imu 坐标系就可以知道 lidar 坐标系。

基本处理流程
------

### 1、imu 预积分

把堆积的 imu 数据进行预积分处理，推导出每一个 imu 数据时刻相对与全局坐标系的 T，以及对应的方差。  
有个细节要注意：  
每次开始之前，都会把上一次预积分的最后一个 imu 加入到本次 imu 队列的头部，以及对应的姿态结果作为此次预积分的开始姿态。  
可以想象，这种推理会导致 imu 推理的轨迹飞的很快，但是不影响我们总的结果，因为我们取的是短时间内的相对运动姿态。

另外，对于此次处理的点云数据，也按照相对时间顺序进行了升序排列。  
这里的

这里预积分用的是中值积分：

#### 数据准备：

```
/*** add the imu of the last frame-tail to the of current frame-head ***/
  auto v_imu = meas.imu;
  v_imu.push_front(last_imu_);//让imu的时间能包住lidar的时间
  const double &imu_beg_time = v_imu.front()->header.stamp.toSec();
  const double &imu_end_time = v_imu.back()->header.stamp.toSec();
  const double &pcl_beg_time = meas.lidar_beg_time;
  
  /*** sort point clouds by offset time ***/
  pcl_out = *(meas.lidar);
  std::sort(pcl_out.points.begin(), pcl_out.points.end(), time_list);
  const double &pcl_end_time = pcl_beg_time + pcl_out.points.back().curvature / double(1000);
  std::cout<<"[ IMU Process ]: Process lidar from "<<pcl_beg_time<<" to "<<pcl_end_time<<", " \
           <<meas.imu.size()<<" imu msgs from "<<imu_beg_time<<" to "<<imu_end_time<<std::endl;

  /*** Initialize IMU pose ***/
  IMUpose.clear();
  // IMUpose.push_back(set_pose6d(0.0, Zero3d, Zero3d, state.vel_end, state.pos_end, state.rot_end));
  IMUpose.push_back(set_pose6d(0.0, acc_s_last, angvel_last, state_inout.vel_end, state_inout.pos_end, state_inout.rot_end));


```

这是点云的排序函数：

```
const bool time_list(PointType &x, PointType &y) {return (x.curvature < y.curvature);};

```

#### 预积分过程

典型的：

```
/*** forward propagation at each imu point ***/
  Eigen::Vector3d acc_imu, angvel_avr, acc_avr, vel_imu(state_inout.vel_end), pos_imu(state_inout.pos_end);
  Eigen::Matrix3d R_imu(state_inout.rot_end);
  Eigen::MatrixXd F_x(Eigen::Matrix<double, DIM_OF_STATES, DIM_OF_STATES>::Identity());
  Eigen::MatrixXd cov_w(Eigen::Matrix<double, DIM_OF_STATES, DIM_OF_STATES>::Zero());
  double dt = 0;
  for (auto it_imu = v_imu.begin(); it_imu != (v_imu.end() - 1); it_imu++)
  {//中值积分
    auto &&head = *(it_imu);
    auto &&tail = *(it_imu + 1);
    
    angvel_avr<<0.5 * (head->angular_velocity.x + tail->angular_velocity.x),
                0.5 * (head->angular_velocity.y + tail->angular_velocity.y),
                0.5 * (head->angular_velocity.z + tail->angular_velocity.z);
    acc_avr   <<0.5 * (head->linear_acceleration.x + tail->linear_acceleration.x),
                0.5 * (head->linear_acceleration.y + tail->linear_acceleration.y),
                0.5 * (head->linear_acceleration.z + tail->linear_acceleration.z);

    angvel_avr -= state_inout.bias_g;
    acc_avr     = acc_avr * G_m_s2 / scale_gravity - state_inout.bias_a;

    #ifdef DEBUG_PRINT
    // fout<<head->header.stamp.toSec()<<" "<<angvel_avr.transpose()<<" "<<acc_avr.transpose()<<std::endl;
    #endif  
    dt = tail->header.stamp.toSec() - head->header.stamp.toSec();
    
    /* covariance propagation */
    Eigen::Matrix3d acc_avr_skew;
    Eigen::Matrix3d Exp_f   = Exp(angvel_avr, dt);
    acc_avr_skew<<SKEW_SYM_MATRX(angvel_avr);

    F_x.block<3,3>(0,0)  = Exp(angvel_avr, - dt);
    F_x.block<3,3>(0,9)  = - Eye3d * dt;
    // F_x.block<3,3>(3,0)  = R_imu * off_vel_skew * dt;
    F_x.block<3,3>(3,6)  = Eye3d * dt;
    F_x.block<3,3>(6,0)  = - R_imu * acc_avr_skew * dt;
    F_x.block<3,3>(6,12) = - R_imu * dt;
    F_x.block<3,3>(6,15) = Eye3d * dt;

    Eigen::Matrix3d cov_acc_diag(Eye3d), cov_gyr_diag(Eye3d);
    cov_acc_diag.diagonal() = cov_acc;
    cov_gyr_diag.diagonal() = cov_gyr;
    cov_w.block<3,3>(0,0).diagonal()   = cov_gyr * dt * dt * 10000;
    cov_w.block<3,3>(3,3)              = R_imu * cov_gyr_diag * R_imu.transpose() * dt * dt * 10000;
    cov_w.block<3,3>(6,6)              = R_imu * cov_acc_diag * R_imu.transpose() * dt * dt * 10000;
    cov_w.block<3,3>(9,9).diagonal()   = Eigen::Vector3d(0.0001, 0.0001, 0.0001) * dt * dt; // bias gyro covariance
    cov_w.block<3,3>(12,12).diagonal() = Eigen::Vector3d(0.0001, 0.0001, 0.0001) * dt * dt; // bias acc covariance

    state_inout.cov = F_x * state_inout.cov * F_x.transpose() + cov_w;

    /* propogation of IMU attitude */
    R_imu = R_imu * Exp_f;

    /* Specific acceleration (global frame) of IMU */
    acc_imu = R_imu * acc_avr + state_inout.gravity;

    /* propogation of IMU */
    pos_imu = pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt;

    /* velocity of IMU */
    vel_imu = vel_imu + acc_imu * dt;

    /* save the poses at each IMU measurements */
    angvel_last = angvel_avr;
    acc_s_last  = acc_imu;
    double &&offs_t = tail->header.stamp.toSec() - pcl_beg_time;
    // std::cout<<"acc "<<acc_imu.transpose()<<"vel "<<acc_imu.transpose()<<"vel "<<pos_imu.transpose()<<std::endl;
    IMUpose.push_back(set_pose6d(offs_t, acc_imu, angvel_avr, vel_imu, pos_imu, R_imu));
  }

```

最后把每个 imu 的姿态保存到了 pose 队列中。  
（这里的协方差没用到，怎么用？）

最后还有一个细节：  
pcl 的最后一个点的时间和 imu 的最后一个时间往往不是精准对齐的，这里根据最后一个 imu 的姿态，计算了最后一个点云的姿态，这个是我们其他所有其他时刻的点云要对准的坐标系：

```
/*** calculated the pos and attitude prediction at the frame-end ***/
  dt = pcl_end_time - imu_end_time;
  state_inout.vel_end = vel_imu + acc_imu * dt;
  state_inout.rot_end = R_imu * Exp(angvel_avr, dt);
  state_inout.pos_end = pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt;

```

### 2、点云补偿

基本过程是把点云的 pose，倒序处理，每两个为一对，在队列前面叫做 head，下一个叫做 tail，对于点云数据，也是倒序处理，找大于 head 时间戳的点来处理，点云处理的位置用一个变量来维护。

这个过程的详细描述为：  
首先将 pcl 的指针 it_pcl 指向最后一个点云的位置；  
取出 imu pose 中最后面的两个 pose，时间小的叫做 head，时间大的叫做 tail；  
以 it_pcl 为起点，倒序处理每个点，对于某个点，假设对应的时刻为 i，判断 i 是否大于 head 的时间，如果大于，则以这个 head 对应的姿态为起点、pcl 点与 head 的时间差为 dt，计算 head 到 i 时刻的姿态。然后对比最后一个点云（结束时刻）的姿态，计算出 i 时刻的这个点，在结束时刻的 lidar 坐标系下的坐标。

P_at_lidar_i=Pi; //i 时刻 lidar 坐标系下的点  
P_at_imu_i=Tbl_P_at_lidar_i;//i 时刻 lidar 点在 imu 坐标系的坐标  
P_at_world_i=Twi_P_at_imu_i;// 转换到 world 坐标系

P_at_imu_e=Twe.inverse()*P_at_world_i;// 转换到结束帧时刻的 imu 坐标  
P_at_lidar_e=Tbl.inverse()*P_at_imu_e;// 有 imu 再转到 lidar 坐标

由于作者用的 imu 与 lidar，坐标系的朝向是一样的，只有平移量，所以不是像上面这样写。  
但如果要改造为自己的装置，则要这样来处理。  
在代码中也对比了这种写法与作者的补偿后的点的坐标，是一致的。

```
auto pos_liD_e = state_inout.pos_end + state_inout.rot_end * Lidar_offset_to_IMU;
  // auto R_liD_e   = state_inout.rot_end * Lidar_R_to_IMU;
  //pos_liD_e的计算假设是imu与lidar的坐标系之间没有旋转变换，只有平移量，这个是livox集成了bmi088
  //它们的坐标系都是前左上，只有平移
  //如果是用自己的装配，需要根据条件调整
  //本质上就是Pw=Tib*Tbl*Pl
  //Pl是雷达坐标系的原点，在雷达坐标系就是(0,0,0)
  //Tib表示imu body坐标系到全局坐标系的转换，这里就是计算出来的rot_end,pos_end
  //Tbl表示lidar坐标系到imu body坐标系的转换，这里旋转为单位阵，平移就是Lidar_offset_to_IMU，即lidar原点在imu坐标系下的表示

  #ifdef DEBUG_PRINT
    std::cout<<"[ IMU Process ]: vel "<<state_inout.vel_end.transpose()<<" pos "<<state_inout.pos_end.transpose()<<" ba"<<state_inout.bias_a.transpose()<<" bg "<<state_inout.bias_g.transpose()<<std::endl;
    std::cout<<"propagated cov: "<<state_inout.cov.diagonal().transpose()<<std::endl;
  #endif

  /*** undistort each lidar point (backward propagation) ***/
  auto it_pcl = pcl_out.points.end() - 1;
  for (auto it_kp = IMUpose.end() - 1; it_kp != IMUpose.begin(); it_kp--)
  {
    bool comp=false;
    if(it_kp==IMUpose.begin()+1){
      comp=true;
    }
    auto head = it_kp - 1;
    auto tail = it_kp;
    R_imu<<MAT_FROM_ARRAY(head->rot);
    acc_imu<<VEC_FROM_ARRAY(head->acc);
    // std::cout<<"head imu acc: "<<acc_imu.transpose()<<std::endl;
    vel_imu<<VEC_FROM_ARRAY(head->vel);
    pos_imu<<VEC_FROM_ARRAY(head->pos);
    angvel_avr<<VEC_FROM_ARRAY(head->gyr);

    int pc_cnt=0;
    for(; it_pcl->curvature / double(1000) > head->offset_time; it_pcl --)
    {
      ++pc_cnt;
      dt = it_pcl->curvature / double(1000) - head->offset_time;

      /* Transform to the 'end' frame, using only the rotation
       * Note: Compensation direction is INVERSE of Frame's moving direction
       * So if we want to compensate a point at timestamp-i to the frame-e
       * P_compensate = R_imu_e ^ T * (R_i * P_i + T_ei) where T_ei is represented in global frame */
      Eigen::Matrix3d R_i(R_imu * Exp(angvel_avr, dt));
      Eigen::Vector3d T_ei(pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt + R_i * Lidar_offset_to_IMU - pos_liD_e);
      //T_ei表示i时刻的lidar坐标原点到end时刻的坐标平移量，用的全局坐标系的衡量
      //T_ei=Ti-Te
      //为什么是Ti-Te，在global坐标系下原点为o，Ti是向量oi,Te是向量oe,
      //Ti-Te就是向量oi-oe，得到的是向量ei,
      //即以e指向i的向量，以e为原点的向量
      //end时刻已经计算出来了就是pos_liD_e
      //i时刻的坐标计算方式与pos_liD_e一样
      //pos_liD_i=Pos_i+rot_i*Lidar_offset_to_IMU
      //而Pos_i=Pos_head+velocity_head*dt+0.5*acc_head*dt*dt

      Eigen::Vector3d P_i(it_pcl->x, it_pcl->y, it_pcl->z);
      Eigen::Vector3d P_compensate = state_inout.rot_end.transpose() * (R_i * P_i + T_ei);
      //本质上是将P_i转换到全局坐标系，再转换到end的坐标系下
      //验证以下：

      static long test_cnt=0;
     if(comp==true && pc_cnt%100){
        ++test_cnt;

        //lidar 到imu的转换矩阵
        Eigen::Matrix4d Tbl;
        Tbl<<1,0,0,Lidar_offset_to_IMU(0),
            0,1,0,Lidar_offset_to_IMU(1),
            0,0,1,Lidar_offset_to_IMU(2),
            0,0,0,1;
        std::cout<<"Tbl=\n"<<Tbl<<std::endl;
        //i时刻imu到global的转换
        Eigen::Matrix4d Twi;
        Twi.block<3,3>(0,0)=R_i;
        Twi.block<3,1>(0,3)=pos_imu + vel_imu * dt + 0.5 * acc_imu * dt * dt;
        Twi(3,0)=0;
        Twi(3,1)=0;
        Twi(3,2)=0;
        Twi(3,3)=1;
        std::cout<<"Twi=\n"<<Twi<<std::endl;

        //e时刻imu到global的转换
        Eigen::Matrix4d Twe;
        Twe.block<3,3>(0,0)=(state_inout.rot_end);
        Twe.block<3,1>(0,3)=(state_inout.pos_end);
        Twe(3,0)=0;
        Twe(3,1)=0;
        Twe(3,2)=0;
        Twe(3,3)=1;
        std::cout<<"Twe=\n"<<Twe<<std::endl;

        //i时刻 lidar原点在global下的值
        Eigen::Vector4d P_i_lidar_at_imu;
        P_i_lidar_at_imu.block<3,1>(0,0)=Lidar_offset_to_IMU;
        P_i_lidar_at_imu(3,0)=1;
        Eigen::Vector4d P_i_lidar_at_global=Twi*P_i_lidar_at_imu;
        std::cout<<"P_i_lidar_at_global=\n"<<P_i_lidar_at_global<<std::endl;


        //e时刻 lidar原点在global下的值
        Eigen::Vector4d P_e_lidar_at_imu;
        P_e_lidar_at_imu.block<3,1>(0,0)=Lidar_offset_to_IMU;
        P_e_lidar_at_imu(3,0)=1;
        Eigen::Vector4d P_e_lidar_at_global=Twe*P_e_lidar_at_imu;
        std::cout<<"P_e_lidar_at_global=\n"<<P_e_lidar_at_global<<std::endl;

        //e->i
        Eigen::Vector4d Tei_2=P_i_lidar_at_global-P_e_lidar_at_global;
        std::cout<<"Tei_2=\n"<<Tei_2
        <<"\nT_ei=\n"<<T_ei<<std::endl;

        //i时刻的lidar点转到i时刻的imu坐标系，再转到global坐标系，再转到e时刻的imu坐标系，再转到e时刻的lidar坐标系
        Eigen::Vector4d P_i_h;
        P_i_h.block<3,1>(0,0)=P_i;
        P_i_h(3)=1;

        Eigen::Vector4d Pw_i=Twi*Tbl*P_i_h;//lidar坐标系下的点，转到imu坐标系，转到global坐标系
        Eigen::Vector4d Pe=Tbl.inverse()*Twe.inverse()*Pw_i;//global坐标系的点，转到e时刻imu坐标系，转到lidar坐标系
        std::cout<<"Point at e =\n"<<Pe<<std::endl;
        std::cout<<"P_compensate = \n"<<P_compensate<<std::endl;
        std::cout<<"P_compensate-pe="<<(P_compensate-Pe.block<3,1>(0,0))<<std::endl;
        std::cout<<"pos_liD_e=\n"<<pos_liD_e<<std::endl;
        }

        /// save Undistorted points and their rotation
        it_pcl->x = P_compensate(0);
        it_pcl->y = P_compensate(1);
        it_pcl->z = P_compensate(2);

        if (it_pcl == pcl_out.points.begin()) break;
      }

```

对比的结果：

```
Tbl=
      1       0       0 0.04165
      0       1       0 0.02326
      0       0       1 -0.0284
      0       0       0       1
Twi=
           1 -0.000388064  0.000560378   -0.0502916
 0.000388001            1  0.000113026   -0.0282829
-0.000560422 -0.000112809            1    0.0225139
           0            0            0            1
Twe=
           1 -0.000500308  0.000576076   -0.0501736
 0.000500295            1  2.28191e-05   -0.0279155
-0.000576087 -2.25309e-05            1    0.0224003
           0            0            0            1
P_i_lidar_at_global=
-0.00866659
-0.00500993
 -0.0059121
          1
P_e_lidar_at_global=
-0.00855164
-0.00463531
-0.00602425
          1
Tei_2=
 -0.00011495
-0.000374617
  0.00011215
           0
T_ei=
 -0.00011495
-0.000374617
  0.00011215
Point at e =
 14.6811
-6.30971
 3.42791
       1
P_compensate = 
 14.6811
-6.30971
 3.42791
P_compensate-pe=-5.68434e-14
 1.15463e-14
-2.17604e-14
pos_liD_e=
-0.00855164
-0.00463531
-0.00602425


```