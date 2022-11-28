> 本文由 [简悦 SimpRead](http://ksria.com/simpread/) 转码， 原文地址 [blog.csdn.net](https://blog.csdn.net/brightming/article/details/118049819)

指定[欧拉角](https://so.csdn.net/so/search?q=%E6%AC%A7%E6%8B%89%E8%A7%92&spm=1001.2101.3001.7020)，是指按照指定的顺序，按照右乘的方式构建旋转矩阵。

验证一：指定[旋转矩阵](https://so.csdn.net/so/search?q=%E6%97%8B%E8%BD%AC%E7%9F%A9%E9%98%B5&spm=1001.2101.3001.7020)，得到欧拉角，按照轴角的方式重新构建矩阵：
-------------------------------------------------------------------------------------------------------------------------------

```
///---------------------------//
    Eigen::Matrix3d rot_cl;
    rot_cl<<-0.00382504437037171  ,   0.190218909453634	,0.981734248929839,\
        -0.999948197804418,	 0.00853270673893375,	-0.00554929035141790,\
        -0.00943243040057437   ,  -0.981704619222084	,0.190176417714608;
    rot_cl.transposeInPlace();//上面是matlab出来的列为先输出的矩阵

    std::cout<<"rot_cl=\n"<<rot_cl<<std::endl;
    Eigen::Vector3d eulerAngle_lidar_to_camera_1=rot_cl.eulerAngles(2,1,0);
    std::cout<<"eulerAngle_lidar_to_camera_1 :"<<eulerAngle_lidar_to_camera_1[0]
    <<","<<eulerAngle_lidar_to_camera_1[1]
    <<","<<eulerAngle_lidar_to_camera_1[2]
    <<std::endl;

    Eigen::Matrix3d chk_mat = (AngleAxisd(eulerAngle_lidar_to_camera_1[0], Vector3d::UnitZ())
     * AngleAxisd(eulerAngle_lidar_to_camera_1[1], Vector3d::UnitY())
     * AngleAxisd(eulerAngle_lidar_to_camera_1[2], Vector3d::UnitX())).toRotationMatrix(); 

    //compare rotation construct by euler angles
    std::cout<<"chk_mat=\n"<<chk_mat<<std::endl
    <<"diff=\n"<<(chk_mat-rot_cl)
    <<std::endl;

```

输出：

```
rot_cl=
-0.00382504   -0.999948 -0.00943243
   0.190219  0.00853271   -0.981705
   0.981734 -0.00554929    0.190176
eulerAngle_lidar_to_camera_1 :1.5909,-1.37937,-0.0291714
chk_mat=
-0.00382504   -0.999948 -0.00943243
   0.190219  0.00853271   -0.981705
   0.981734 -0.00554929    0.190176
diff=
-7.84962e-17  2.22045e-16 -2.94903e-17
 6.66134e-16  2.75821e-16 -2.22045e-16
-2.22045e-16 -3.46945e-17 -8.32667e-17
x_lidar_in_cam=-0.00382504    0.190219    0.981734
y_lidar_in_cam=  -0.999948  0.00853271 -0.00554929
z_lidar_in_cam=-0.00943243   -0.981705    0.190176

```

以上是测试的 lidar 和 camera 的旋转矩阵。  
camera 的坐标系是右下前，lidar 是前左上。  
直接看旋转矩阵，可以知道，lidar 的 x 大约为 camera 的 z，lidar 的 y 大约为 camera 坐标系的 - x,lidar 的 z 大约为 camera 坐标系的 - y。

得到的欧拉角：1.5909,-1.37937,-0.0291714，根据获取时候用的 rot_cl.eulerAngles(2,1,0); 表示先获取 Z 轴的转角，再 Y，再 X，那么按照得到的角度，先绕 Z 轴旋转大概 90 度（逆时针），再按转动后的 Y 轴旋转大约 - 90（顺时针），再按转动后的 X 旋转约 0 度，就可以从 camera 的 “右下前” 得到 lidar 的坐标系“前左上”。

![](https://img-blog.csdnimg.cn/20210619110811364.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L2JyaWdodG1pbmc=,size_16,color_FFFFFF,t_70)  
验证二：指定旋转矩阵，得到欧拉角，按照 rotz_roty_rotx 重新构建矩阵：
------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

```
//指定euler构造rotation
    yaw= 1.5909 ;//rotate around z
    pitch= -1.3794 ;//rotate around y
    roll=  -0.0292;//rotate around x

    Eigen::Matrix3d rot_z;
    rot_z<<cos(yaw),-sin(yaw),0,\
    sin(yaw),cos(yaw),0,\
    0,0,1;
    Eigen::Matrix3d rot_y;
    rot_y<<cos(pitch),0,sin(pitch),\
                            0,1,0,\
                            -sin(pitch),0,cos(pitch);
    Eigen::Matrix3d rot_x;
    rot_x<<1,0,0,\
    0,cos(roll),-sin(roll),\
    0,sin(roll),cos(roll);

    Eigen::Matrix3d res_zyx=rot_z*rot_y*rot_x;
    std::cout<<"res_zyx=\n"<<res_zyx
    <<std::endl;
    //重新获取rotation
    Eigen::Vector3d ang2=res_zyx.eulerAngles(2,1,0);
    std::cout<<"ang2 = "<<ang2.transpose()<<std::endl;

```

输出：

```
res_zyx=
-0.00382406   -0.999948 -0.00946312
   0.190191  0.00856318    -0.98171
    0.98174 -0.00555392    0.190149
ang2 =  1.5909 -1.3794 -0.0292

```

同上。

验证三：用另一张顺序获取 euler angle
------------------------

```
 ///-------------换一个顺序- x->y->z --------------//
    Eigen::Vector3d angle_xyz=res_zyx.eulerAngles(0,1,2);
    std::cout<<"angle_xyz="<<angle_xyz.transpose()<<std::endl;

    roll=angle_xyz[0];
    pitch=angle_xyz[1];
    yaw=angle_xyz[2];

    Eigen::Matrix3d rot_z2;
    rot_z2<<cos(yaw),-sin(yaw),0,\
    sin(yaw),cos(yaw),0,\
    0,0,1;
    Eigen::Matrix3d rot_y2;
    rot_y2<<cos(pitch),0,sin(pitch),\
                            0,1,0,\
                            -sin(pitch),0,cos(pitch);
    Eigen::Matrix3d rot_x2;
    rot_x2<<1,0,0,\
    0,cos(roll),-sin(roll),\
    0,sin(roll),cos(roll);

    Eigen::Matrix3d res_xyz=rot_x2*rot_y2*rot_z2;
    std::cout<<"res_xyz=\n"<<res_xyz
    <<std::endl;

```

输出：

```
angle_xyz=    1.37947 -0.00946326     1.57462
res_xyz=
-0.00382406   -0.999948 -0.00946312
   0.190191  0.00856318    -0.98171
    0.98174 -0.00555392    0.190149

```

可以看到，得到的旋转矩阵是一样的。

验证三：rpy
-------

```
//测试rpy，旋转矩阵->四元数->rpy
    Eigen::Quaterniond quater(res_xyz);
    Rot3 gtsam_rot(quater);
    gtsam::Vector3 gtsam_rpy=gtsam_rot.rpy();
    std::cout<<"gtsam_rpy ("<<gtsam_rpy[0]<<","<<gtsam_rpy[1]<<","<<gtsam_rpy[2]<<std::endl;



```

输出：

```
gtsam_rpy (-0.0292,-1.3794,1.5909

```

可以看到，与上面的欧拉角是反过来的。  
这个含义是绕固定轴 x\y\z（camera 坐标系）旋转多少度后，可以到达 lidar 的坐标系。  
绕 x 旋转 - 0.0292(roll)，再绕固定轴 y 轴旋转 - 1.3794(pitch)，再绕固定轴 z 轴旋转 1.5909(yaw)  
在乘法上是：  
rot_z_rot_y_rot_x (rot_z <- rot_y <- rot_x)  
即将后续的姿态变换放在左边。  
rot(1.5909)_rot(-1.3794)_rot(-0.0292)  
在数值上，与上面的用欧拉角，右乘的数值是一样的。上面欧拉角右乘是：  
先绕 z 旋转，再绕 y 旋转，再绕 x 旋转，且后续旋转放在右边：  
rot_z_rot_y_rot_x (rot_z -> rot_y -> rot_x)

在坐标上旋转也是可以直观看到的：  
![](https://img-blog.csdnimg.cn/20210619134731984.png?x-oss-process=image/watermark,type_ZmFuZ3poZW5naGVpdGk,shadow_10,text_aHR0cHM6Ly9ibG9nLmNzZG4ubmV0L2JyaWdodG1pbmc=,size_16,color_FFFFFF,t_70)

### 这里的 rpy 对应的 roll(-0.0292)、pitch(-1.3794)、yaw(1.5909)，与上面的得到对应 zyx 对应的角度： 1.5909（绕 z 轴） -1.3794（绕 y 轴） -0.0292（绕 x 轴）是一样的含义。

所以，如果我们说，经过 RPY 旋转（定轴，外旋）可以到达另一个坐标系，等价于经过绕动轴 ZYX 旋转（内旋）到达另一个坐标系。  
所以，ZYX 欧拉角旋转组合就与我们常说 RPY 旋转是对应的。

### 总结

给定一个旋转矩阵，代表了某种变换，可以有很多种不同的旋转组合得到这个变换，这个就是那 12 种欧拉角旋转方式。  
There are six possibilities of choosing the rotation axes for proper Euler angles. In all of them, the first and third rotation axes are the same. The six possible sequences are:

```
z1-x′-z2″ (intrinsic rotations) or z2-x-z1 (extrinsic rotations)
x1-y′-x2″ (intrinsic rotations) or x2-y-x1 (extrinsic rotations)
y1-z′-y2″ (intrinsic rotations) or y2-z-y1 (extrinsic rotations)
z1-y′-z2″ (intrinsic rotations) or z2-y-z1 (extrinsic rotations)
x1-z′-x2″ (intrinsic rotations) or x2-z-x1 (extrinsic rotations)
y1-x′-y2″ (intrinsic rotations) or y2-x-y1 (extrinsic rotations)

```

There are six possibilities of choosing the rotation axes for Tait–Bryan angles. The six possible sequences are:

```
x-y′-z″ (intrinsic rotations) or z-y-x (extrinsic rotations)
y-z′-x″ (intrinsic rotations) or x-z-y (extrinsic rotations)
z-x′-y″ (intrinsic rotations) or y-x-z (extrinsic rotations)
x-z′-y″ (intrinsic rotations) or y-z-x (extrinsic rotations)
z-y′-x″ (intrinsic rotations) or x-y-z (extrinsic rotations): the intrinsic rotations are known as: yaw, pitch and roll
y-x′-z″ (intrinsic rotations) or z-x-y (extrinsic rotations)

```

以一种方式组合方式构建的旋转矩阵，可以以另一种不同顺序的方式得到不同欧拉角。但只要对应顺序去乘，就能得到一致的旋转矩阵。