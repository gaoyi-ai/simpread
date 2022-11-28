> 本文由 [简悦 SimpRead](http://ksria.com/simpread/) 转码， 原文地址 [blog.csdn.net](https://blog.csdn.net/brightming/article/details/118250783?spm=1001.2101.3001.6650.12&utm_medium=distribute.pc_relevant.none-task-blog-2%7Edefault%7ECTRLIST%7ERate-12-118250783-blog-127252203.pc_relevant_multi_platform_whitelistv3&depth_1-utm_source=distribute.pc_relevant.none-task-blog-2%7Edefault%7ECTRLIST%7ERate-12-118250783-blog-127252203.pc_relevant_multi_platform_whitelistv3&utm_relevant_index=13)

利用 imu 在短时间 100ms 内的运动，补偿雷达的运动畸变，只进行旋转方向的补偿。

1、找配对数据  
以 lidar 为基准，找到包含 lidar 一帧的点的起始、结束时刻的 imu 数据。

2、对这些 imu 数据，进行积分处理  
以第一个 imu 为基准坐标，依次积分所有的 imu 的数据，每个 imu 都得到一个对应 pose。

3、计算结束时刻的 lidar 的相对于第一个 imu 的姿态  
lidar 不一定和 imu 的时间对齐，因此，要进行[插值](https://so.csdn.net/so/search?q=%E6%8F%92%E5%80%BC&spm=1001.2101.3001.7020)。  
找到包含这个结束时刻的最近的左边、右边的 imu，得到其姿态和时间。  
然后利用插值的方法，插值出这个 lidar 时刻的姿态，注意，这个姿态是 imu 坐标系的，要转换成 lidar 坐标系相对于第一个 imu 坐标系的姿态。

4、将整个点云的点按照第 3 步处理方式，计算出其 lidar 坐标系相对于第一个 imu 的姿态，再转换到最后一个时刻的 lidar 坐标系下。

5、速度的考虑  
1）考虑到点云很多，几万个点，没有必要每个点的时刻都计算一次姿态，将 100ms，划分成 10 个段，计算 10 个段的姿态，落在某一个段内的，就用对应的姿态进行计算。  
2）并行化  
这些处理的逻辑是简单的，且是无相关的，可以用 omp 的方式进行加速处理

6、之前参考的 lidar 补偿的一些问题  
参考的 livox 的运动补偿，是用时间比例进行插值的方式进行补偿，这个处理，在 lidar 与 imu 有平移量的时候，是不能精准插值的。

7、代码

流程代码：

```
void ImuProcess::Process(const MeasureGroup &meas) {
  if(meas.imu.empty() || meas.lidar==nullptr){
    return;
  }
  // ROS_ASSERT(!meas.imu.empty());
  // ROS_ASSERT(meas.lidar != nullptr);
  // ROS_DEBUG("Process lidar at time: %.4f, %lu imu msgs from %.4f to %.4f",
  //           meas.lidar->header.stamp.toSec(), meas.imu.size(),
  //           meas.imu.front()->header.stamp.toSec(),
  //           meas.imu.back()->header.stamp.toSec());

  auto pcl_in_msg = meas.lidar;

  /**
  if (b_first_frame_) {
    /// The very first lidar frame

    /// Reset
    // Reset();

    /// Record first lidar, and first useful imu
    last_lidar_ = pcl_in_msg;
    last_imu_ = meas.imu.back();

    ROS_WARN("The very first lidar frame");

    /// Do nothing more, return
    b_first_frame_ = false;
    return;
  }
  */

  /// Integrate all input imu message
  Reset();

  pcl::fromROSMsg(*pcl_in_msg, *cur_pcl_in_);

  double lidar_beg_time=cur_pcl_in_->points[0].timestamp;
  double lidar_end_time=cur_pcl_in_->points.back().timestamp;
  double imu_beg_time=meas.imu.front()->header.stamp.toSec();
  double imu_end_time=meas.imu.back()->header.stamp.toSec();
  //找到包含lidar begin and lidar end的两个imu
  uint imu_idx_for_lidar_beg_1=0;
  uint imu_idx_for_lidar_beg_2=0;
  uint imu_idx_for_lidar_end_1=0;
  uint imu_idx_for_lidar_end_2=0;

  IntegrateGyr(meas.imu,
  lidar_beg_time,
  lidar_end_time,
  imu_idx_for_lidar_beg_1,
  imu_idx_for_lidar_beg_2,
  imu_idx_for_lidar_end_1,
  imu_idx_for_lidar_end_2
  );//积分出来的是imu最后相对于最前的变化，如果与pcl的时间不一致，还不能直接用
  
  double t_lidar_beg_to_imu_beg=lidar_beg_time-imu_beg_time;
  double t_imu_time_range=imu_end_time-imu_beg_time;
  double t_lidar_time_range=lidar_end_time-lidar_beg_time;
  double t_lidar_end_to_imu_end=imu_end_time-lidar_end_time;

  

  //求出lidar起始时刻、结束时刻的imu的姿态
  //包含此lidar的imu的时间差
  double imu_time_contain_beg_lidar=meas.imu[imu_idx_for_lidar_beg_2]->header.stamp.toSec()-meas.imu[imu_idx_for_lidar_beg_1]->header.stamp.toSec();
  //lidar start time to 包含此ldiar的开始的imu的时间
  double lidar_start_time_to_contain_imu_start=lidar_beg_time-meas.imu[imu_idx_for_lidar_beg_1]->header.stamp.toSec();
  std::vector<Sophus::SO3d> imu_poses=gyr_int_.GetRots();
  Sophus::SO3d imu_pose_1=imu_poses[imu_idx_for_lidar_beg_1];
  Sophus::SO3d imu_pose_2=imu_poses[imu_idx_for_lidar_beg_2];
  Sophus::SO3d lidar_beg_so3=GetInterpolateSO3(
  imu_pose_1,imu_pose_2,
  imu_time_contain_beg_lidar,
  lidar_start_time_to_contain_imu_start
  );

  //结束时刻的lidar姿态
  double imu_time_contain_end_lidar=meas.imu[imu_idx_for_lidar_end_2]->header.stamp.toSec()-meas.imu[imu_idx_for_lidar_end_1]->header.stamp.toSec();
  //lidar end time to 包含此ldiar的开始的imu的时间
  double lidar_end_time_to_contain_imu_start=lidar_end_time-meas.imu[imu_idx_for_lidar_end_1]->header.stamp.toSec();
  Sophus::SO3d imu_pose_3=imu_poses[imu_idx_for_lidar_end_1];
  Sophus::SO3d imu_pose_4=imu_poses[imu_idx_for_lidar_end_2];
  Sophus::SO3d lidar_end_so3=GetInterpolateSO3(
  imu_pose_3,imu_pose_4,
  imu_time_contain_end_lidar,
  lidar_end_time_to_contain_imu_start
  );

  std::cout<<"t_imu_time_range="<<t_imu_time_range<<"\n"
           <<"lidar_beg_time="<<lidar_beg_time<<"\n"
           <<"t_lidar_beg_to_imu_beg="<<t_lidar_beg_to_imu_beg<<"\n"
           <<"t_lidar_end_to_imu_end="<<t_lidar_end_to_imu_end<<"\n"
           <<"contain lidar begtime ,imu idx["<<imu_idx_for_lidar_beg_1<<","<<imu_idx_for_lidar_beg_2<<"]\n"
           <<"contain lidar endtime ,imu idx["<<imu_idx_for_lidar_end_1<<","<<imu_idx_for_lidar_end_2<<"]\n"
           <<",lidar_start_time_to_contain_imu_start="<<lidar_start_time_to_contain_imu_start
           <<std::endl;

  std::cout
  <<"begin lidar pose:"<<(lidar_beg_so3.log()).transpose()*180.0/M_PI
  <<"\ncontained beg lidar imu start pose:"<<(imu_pose_1.log()).transpose()*180.0/M_PI
  <<"\ncontained beg lidar imu end pose:"<<(imu_pose_2.log()).transpose()*180.0/M_PI

   <<"\nend lidar pose:"<<(lidar_end_so3.log()).transpose()*180.0/M_PI
  <<"\ncontained end lidar imu start pose:"<<(imu_pose_3.log()).transpose()*180.0/M_PI
  <<"\ncontained end lidar  imu end pose:"<<(imu_pose_4.log()).transpose()*180.0/M_PI
  <<std::endl;

  
  //最终要得出的是pcl的起始时刻和结束时刻的相对位姿变化
  

  
  //计算lidar begin以及lidar end的相对于imu起始时刻的姿态
  //直接插值
  
  /// Compensate lidar points with IMU rotation
   Initial pose from IMU (with only rotation)
  //这个是imu的结束时刻相比开始时刻的姿态变化
  // SE3d T_l_c(gyr_int_.GetRot(), Eigen::Vector3d::Zero());
  // dt_l_c_ =0.1;//   pcl_in_msg->header.stamp.toSec() - last_lidar_->header.stamp.toSec();
  
  //利用上面计算的lidar 结束时刻、开始时刻的旋转，计算出lidar帧内的姿态变化
  dt_l_c_=lidar_end_time-lidar_beg_time;
  Eigen::Vector3d lidar_frame_angled=lidar_end_so3.log()-lidar_beg_so3.log();
  std::cout
  <<"lidar_frame rot angled = "<<lidar_frame_angled.transpose()*(180.00/M_PI)
  <<"\ndt_l_c_="<<dt_l_c_
  <<std::endl;
  Sophus::SO3d lidar_frame_so3=SO3d::exp(lidar_frame_angled);//imu坐标系下的
  SE3d T_l_c(lidar_frame_so3, Eigen::Vector3d::Zero());
   

      // std::cout<<"pcl_in_msg->header.stamp.toSec()="<<pcl_in_msg->header.stamp.toSec()
      // <<", last_lidar_->header.stamp.toSec()="<< last_lidar_->header.stamp.toSec()
      // <<std::endl;
   Get input pcl
  

  /// Undistort points
  //因为积分的时候，是以begin为基准的，得到其他时刻相对于begin时刻的坐标变化
  //而我们要补偿的是补偿到end时刻，所以在执行的时候要注意
  Sophus::SE3d T_l_be = T_i_l.inverse() * T_l_c * T_i_l;//end to begin的姿态变化
  pcl::copyPointCloud(*cur_pcl_in_, *cur_pcl_un_);

  //这种插值方式，在imu和雷达在有平移量时是不精确的，但是时间运算量较小
  // UndistortPcl(cur_pcl_un_, dt_l_c_, T_l_be);

  //精准的插值方式
  UndistortPclPrecise(cur_pcl_un_,meas,imu_poses,lidar_end_so3,T_i_l);
}




void ImuProcess::Reset() {
  ROS_WARN("Reset ImuProcess");

  b_first_frame_ = true;
  last_lidar_ = nullptr;
  last_imu_ = nullptr;

  gyr_int_.Reset(-1, nullptr);

  cur_pcl_in_.reset(new pcl::PointCloud<RsPointXYZIRT>());
  cur_pcl_un_.reset(new pcl::PointCloud<RsPointXYZIRT>());
}


```

积分代码：

```
void ImuProcess::IntegrateGyr(
    const std::vector<sensor_msgs::Imu::ConstPtr> &v_imu,
    double &lidar_beg_time,
    double &lidar_end_time,
    uint &imu_idx_for_lidar_beg_1,
    uint &imu_idx_for_lidar_beg_2,
    uint &imu_idx_for_lidar_end_1,
    uint &imu_idx_for_lidar_end_2
    
    
    ) {
  /// Reset gyr integrator
  //gyr_int_.Reset(last_lidar_->header.stamp.toSec(), last_imu_);
  /// And then integrate all the imu measurements
  bool has_large_for_lidarbegintime=false;
  bool has_large_for_lidarendtime=false;
  double imutime=0;
  uint idx=0;
  for (const auto &imu : v_imu) {
    imutime=imu->header.stamp.toSec();
    if(has_large_for_lidarbegintime==false && imutime<=lidar_beg_time){
      imu_idx_for_lidar_beg_1=idx;
    }
    if(has_large_for_lidarendtime==false && imutime<=lidar_end_time){
      imu_idx_for_lidar_end_1=idx;
    }
    if(has_large_for_lidarbegintime==false && imutime>lidar_beg_time){
      imu_idx_for_lidar_beg_2=idx;
      has_large_for_lidarbegintime=true;
    }
    if(has_large_for_lidarendtime==false && imutime>lidar_end_time){
      imu_idx_for_lidar_end_2=idx;
      has_large_for_lidarendtime=true;
    }

    gyr_int_.Integrate(imu);
    ++idx;
  }
  ROS_INFO("integrate time[%.6f,%.6f] rotation angle [x, y, z]: [%.2f, %.2f, %.2f]",
           v_imu.front()->header.stamp.toSec(),v_imu.back()->header.stamp.toSec(),
           gyr_int_.GetRot().angleX() * 180.0 / M_PI,
           gyr_int_.GetRot().angleY() * 180.0 / M_PI,
           gyr_int_.GetRot().angleZ() * 180.0 / M_PI);
}

void GyrInt::Integrate(const sensor_msgs::ImuConstPtr &imu) {
   //以第一个imu
  if (v_rot_.empty()) {
    v_rot_.push_back(SO3d());
    v_imu_.push_back(imu);

    return;
  }
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
  // std::cout<<"delta_angle="<<delta_angle<<std::endl;
  auto delta_r = SO3d::exp(delta_angle);

  SO3d rot = rot_last * delta_r;

  v_imu_.push_back(imu);
  v_rot_.push_back(rot);


}

```

插值代码：

```
Sophus::SO3d ImuProcess::GetInterpolateSO3(Sophus::SO3d start,Sophus::SO3d end ,double range,double time_to_beg){
  
   Eigen::Vector3d rso3_1=start.log();
   Eigen::Vector3d rso3_2=end.log();

   Eigen::Vector3d delta_rso=rso3_2-rso3_1;
   double ratio_to_beg=time_to_beg/range;

   Eigen::Vector3d delta_from_i_to_beg=delta_rso*ratio_to_beg;

   Eigen::Vector3d res_rso=rso3_1+delta_from_i_to_beg;
      
   SO3d resSO3 = SO3d::exp(res_rso);

   return resSO3;
}


void ImuProcess::UndistortPclPrecise(const pcl::PointCloud<RsPointXYZIRT>::Ptr &pcl_in_out,
const MeasureGroup &meas,
const std::vector<Sophus::SO3d> &imu_poses,//所有的pose
Sophus::SO3d &lidar_end_time_imu_pose,lidar 结束时刻的imu的pose
Sophus::SE3d &Tbl //lidar to imu
){

  //结束时刻imu的姿态，不考虑平移
  Sophus::SE3d T_lidar_endtime_imu(lidar_end_time_imu_pose,Eigen::Vector3d::Zero());
  //结束时刻，lidar相对于imu基准坐标的姿态
  Sophus::SE3d T_lidar_endtime_pose=T_lidar_endtime_imu*Tbl;
  std::cout<<"T_lidar_endtime_pose = \n"<<T_lidar_endtime_pose.matrix()<<std::endl;
  double t=0;
  for (auto &pt : pcl_in_out->points) {
    if(!pcl_isfinite(pt.x)
           ||!pcl_isfinite(pt.y)
           ||!pcl_isfinite(pt.z)
           ||pcl_isnan(pt.x)
           ||pcl_isnan(pt.y)
           ||pcl_isnan(pt.z)
        ){
          continue;
        }

        t=pt.timestamp;
        uint fi=0,fe=0;
        bool has_fe=false;
        uint idx=0;
        double t_imu_1=0,t_imu_2=0,t_tmp=0;
        for(auto &imu:meas.imu){
          if(has_fe){
            break;
          }
          t_tmp=imu->header.stamp.toSec();
          if(t_tmp<=t){
            fi=idx;
            t_imu_1=t_tmp;
          }else if(t_tmp>t){
            fe=idx;
            t_imu_2=t_tmp;
            has_fe=true;
            break;
          }


          ++idx;
        }
        auto imu_pose_3=imu_poses[fi];
        auto imu_pose_4=imu_poses[fe];
        double imu_time_contain_end_lidar=t_imu_2-t_imu_1;
        double lidar_end_time_to_contain_imu_start = t-t_imu_1;
        //插值姿态,imu的
        Sophus::SO3d lidar_i_so3=GetInterpolateSO3(
                                  imu_pose_3,imu_pose_4,
                                  imu_time_contain_end_lidar,
                                  lidar_end_time_to_contain_imu_start
                                  );
        Sophus::SE3d T_curr_imu(lidar_i_so3,Eigen::Vector3d::Zero());
        Sophus::SE3d T_curr_lidar=T_curr_imu*Tbl;

        // Sophus::SE3d T_curr_to_end_lidar=T_lidar_endtime_pose*T_curr_lidar.inverse();

        Eigen::Vector3d pi(pt.x,pt.y,pt.z);
        // auto pe=T_curr_to_end_lidar*pi;
        auto pe=T_lidar_endtime_pose.inverse()*T_curr_lidar*pi;
        // std::cout<<"lidar_i_so3="<<lidar_i_so3.angleX()*180./M_PI<<","<<lidar_i_so3.angleY()*180./M_PI<<","<<lidar_i_so3.angleZ()*180./M_PI<<std::endl;
        std::cout<<"\n\n\n--------\nt="<<t<<",t_imu_1="<<t_imu_1<<",t_imu_2="<<t_imu_2<<std::endl;
        std::cout<<"T_curr_lidar=\n"<<T_curr_lidar.matrix()<<std::endl;
        
        std::cout<<"aft precision compensation,\norig p = "<<pi.transpose()
        <<"\npe="<<pe.transpose()<<std::endl<<std::endl;

        //这个时刻的lidar姿态相对于
        // Sophus::SE3d T_l_be = T_i_l.inverse() * T_l_c * T_i_l;//end to begin的姿态变化
  }

}

```