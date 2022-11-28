> 本文由 [简悦 SimpRead](http://ksria.com/simpread/) 转码， 原文地址 [blog.csdn.net](https://blog.csdn.net/brightming/article/details/127473326?spm=1001.2014.3001.5502)

这次好好的梳理一下。  
先从两种旋转开始说起。

** 参考：https://blog.51cto.com/xxpcb/2903395**

1、坐标轴不动，向量绕原点旋转后的关系

旋转前坐标为 (x,y,z)，旋转后为 (x',y',z')。

旋转情况为：

绕 z 轴旋转 yaw，绕 y 轴旋转 pitch，绕 x 轴旋转 roll。

这里明显可知道，每次都是绕的是固定的原坐标系轴，即可以说是外旋。这个也是这里可以统一概念的地方。

那么旋转矩阵为：

绕 z 轴旋转后的情况是：![](https://latex.codecogs.com/gif.latex?%5Cbegin%7Bbmatrix%7D%20x%27%5C%5C%20y%27%5C%5C%20z%27%20%5Cend%7Bbmatrix%7D%3D%5Cbegin%7Bbmatrix%7D%20cos%28yaw%29%20%26%20-sin%28yaw%29%20%26%200%5C%5C%20sin%28yaw%29%20%26%20cos%28yaw%29%20%26%200%5C%5C%200%20%26%200%20%26%201%20%5Cend%7Bbmatrix%7D%20*%20%5Cbegin%7Bbmatrix%7D%20x%5C%5C%20y%5C%5C%20z%20%5Cend%7Bbmatrix%7D)

在这个基础上，绕 y 轴旋转：

![](https://latex.codecogs.com/gif.latex?%5Cbegin%7Bbmatrix%7D%20x%27%27%5C%5C%20y%27%27%5C%5C%20z%27%27%20%5Cend%7Bbmatrix%7D%3D%5Cbegin%7Bbmatrix%7D%20cos%28pitch%29%20%26%200%20%26%20sin%28pitch%29%5C%5C%200%26%201%20%26%200%5C%5C%20-sin%28pitch%29%260%20%26cos%28pitch%29%20%5Cend%7Bbmatrix%7D*%5Cbegin%7Bbmatrix%7D%20x%27%5C%5C%20y%27%5C%5C%20z%27%20%5Cend%7Bbmatrix%7D%3D%5Cbegin%7Bbmatrix%7D%20cos%28pitch%29%20%26%200%20%26%20sin%28pitch%29%5C%5C%200%26%201%20%26%200%5C%5C%20-sin%28pitch%29%260%20%26cos%28pitch%29%20%5Cend%7Bbmatrix%7D*%5Cbegin%7Bbmatrix%7D%20cos%28yaw%29%20%26%20-sin%28yaw%29%20%26%200%5C%5C%20-sin%28yaw%29%20%26%20cos%28yaw%29%20%26%200%5C%5C%200%20%26%200%20%26%201%20%5Cend%7Bbmatrix%7D*%5Cbegin%7Bbmatrix%7D%20x%5C%5C%20y%5C%5C%20z%20%5Cend%7Bbmatrix%7D)

在这个基础上，绕 x 轴旋转：

![](https://latex.codecogs.com/gif.latex?%5Cbegin%7Bbmatrix%7D%20x%27%27%27%5C%5C%20y%27%27%27%5C%5C%20z%27%27%27%20%5Cend%7Bbmatrix%7D%3D%20%5Cbegin%7Bbmatrix%7D%201%20%26%200%20%26%200%5C%5C%200%20%26%20cos%28roll%29%20%26%20-sin%28roll%29%20%5C%5C%200%20%26%20si%28roll%29%20%26%20cos%28roll%29%20%5Cend%7Bbmatrix%7D*%20%5Cbegin%7Bbmatrix%7D%20cos%28pitch%29%20%26%200%20%26%20sin%28pitch%29%5C%5C%200%26%201%20%26%200%5C%5C%20-sin%28pitch%29%260%20%26cos%28pitch%29%20%5Cend%7Bbmatrix%7D*%5Cbegin%7Bbmatrix%7D%20x%27%5C%5C%20y%27%5C%5C%20z%27%20%5Cend%7Bbmatrix%7D%3D%5Cbegin%7Bbmatrix%7D%201%20%26%200%20%26%200%5C%5C%200%20%26%20cos%28roll%29%20%26%20-sin%28roll%29%20%5C%5C%200%20%26%20si%28roll%29%20%26%20cos%28roll%29%20%5Cend%7Bbmatrix%7D*%5Cbegin%7Bbmatrix%7D%20cos%28pitch%29%20%26%200%20%26%20sin%28pitch%29%5C%5C%200%26%201%20%26%200%5C%5C%20-sin%28pitch%29%260%20%26cos%28pitch%29%20%5Cend%7Bbmatrix%7D*%5Cbegin%7Bbmatrix%7D%20cos%28yaw%29%20%26%20-sin%28yaw%29%20%26%200%5C%5C%20-sin%28yaw%29%20%26%20cos%28yaw%29%20%26%200%5C%5C%200%20%26%200%20%26%201%20%5Cend%7Bbmatrix%7D*%5Cbegin%7Bbmatrix%7D%20x%5C%5C%20y%5C%5C%20z%20%5Cend%7Bbmatrix%7D)

把矩阵乘起来，就得到完整的矩阵。从等式可以看出来，是 Rot_old_to_new 的。

2、坐标轴旋转，原来的向量在新的坐标系下的表示

这个本质上，要计算出新的坐标轴的基向量在原坐标轴的表示。

在上一步已经知道了向量的旋转前后的表示关系。如果把这个向量变成坐标轴，再根据以上变化矩阵计算出新的坐标轴的位置，放在转换矩阵中，可以得到 rot_new_to_old。没错是，new_to_old，因为是把新的坐标轴的基向量在原坐标系下的值放进去的。

还是按照 z->y->x 的旋转顺序，可以一次得到 (1,0,0),(0,1,0),(0,0,1) 旋转后分别是：

![](https://latex.codecogs.com/gif.latex?%5Cbegin%7Bbmatrix%7D%20cp*cy%5C%5C%20cr*sy&plus;sr*sp*cy%5C%5C%20sr*sy-cr*sp*cy%20%5Cend%7Bbmatrix%7D) ===》新的 x 坐标轴在原坐标系的表示

![](https://latex.codecogs.com/gif.latex?%5Cbegin%7Bbmatrix%7D%20-cp*sy%5C%5C%20cr*cy-sr*sp*sy%5C%5C%20sr*cy&plus;cr*sp*sy%20%5Cend%7Bbmatrix%7D) ===》新的 y 坐标轴在原坐标系的表示

![](https://latex.codecogs.com/gif.latex?%5Cbegin%7Bbmatrix%7D%20sp%5C%5C%20-sr*cp%5C%5C%20cr*cp%20%5Cend%7Bbmatrix%7D) ===》新的 z 坐标轴在原坐标系的表示。

构建一个 Rot3x3 矩阵，把这三列，放进去，就得到了 Rot_new_to_old 矩阵：

![](https://img-blog.csdnimg.cn/1850792c41ae4d7b979dfd66f1fadce0.jpeg)

回到标题的问题，坐标轴旋转后，原来的向量是在新的坐标系怎么表示呢？

就是这样了，取个逆，得到 Rot_old_to_new，再乘上原来的向量，就得到了在新坐标系下的表示。

3、利用这些信息来计算静止状态下的 imu 的 roll、pitch 角度

目标是求出，imu 的坐标系，相对于 Enu 坐标系的转换，即 Rot_body_to_enu。

已知道在 imu 下测量到的重力情况为：grav_body=[acc_x,acc_y,acc_z]，这个在 body 坐标系下；

在 Enu 坐标系下，grav_enu 重力的情况为：[0,0,-g] （重力方向向下，enu 的 z 轴向上）。

按照上面的说法，R_new_to_old，可以直接获取，利用关系计算：

grav_enu=R_new_to_old*grav_body

得到：

0=gx=cp*cy*ax-cp*sy*ay+sp*az

0=(cr*sy+sr*sp*cy)*ax+(cr*cy-sr*sp*sy)*ay+sr*cp*az

-g=(sr*sy-cr*sp*cy)*ax+(sr*cy+cr*sp*sy)*ay+cr*cp*az

这个不是很好算。

转换一下思路：

以 grav_body 为开始坐标，计算 grav_enu 相对于它的旋转情况，那就比较简单:

grav_body(当做是 old) = R_new_to_old * grav_enu(当做是 new) 

算出这个 R_new_to_old 后，求个逆，就得到了 grav_body->grav_enu 的变化。

因为 grav_enu 中的 x\y 都是 0，所以可以得：

ax=sp*gz

ay=-sr*cp*gz

az=cr*cp*gz

那么：pitch=asin(ax/gz) roll=-atan(ay/az)

将角度带入那个 R_new_to_old（这里的 new、old 都是相对来说的，哪个都可以作为 new 和 old），就可以得到这个 rot 矩阵。此时的含义是 env_to_body。

求逆以后，得到 body_to_env，再获里面的 roll、pitch，就是 body 相对于 enu 旋转的角度。

（会不会太绕了，起码思路清晰连贯了）

```
 
Eigen::Matrix3d calc_rot_body_to_enu(Eigen::Vector3d grav_in_old,Eigen::Vector3d grav_in_body){
    
    Eigen::Vector3d acc_mea_in_body=grav_in_body;
    // std::cout<<"grav_in_old = "<<grav_in_old.transpose()<<",acc_mea_in_body = "<<acc_mea_in_body.transpose()
    // <<std::endl;
 
    //反向计算出角度
    //为了计算方便，以acc_mea_in_body所在的body为起点，计算到old坐标系的roll,pitch,yaw，计算出T，然后求逆，得到的矩阵应该等价于rot_old_to_new
    double roll_enu_to_body=-atan(acc_mea_in_body(1)/acc_mea_in_body(2));
    double pitch_enu_to_body=asin(acc_mea_in_body(0)/grav_in_old(2));
    double yaw_enu_to_body=0;
    Eigen::Matrix3d R_enu_to_body;
    double cp=cos(pitch_enu_to_body);
    double sp=sin(pitch_enu_to_body);
    double cy=cos(yaw_enu_to_body);
    double sy=sin(yaw_enu_to_body);
    double cr=cos(roll_enu_to_body);
    double sr=sin(roll_enu_to_body);
 
    R_enu_to_body<<cp*cy,           -cp*sy,         sp,
                    cr*sy+sr*sp*cy,cr*cy-sr*sp*sy,-sr*cp,
                    sr*sy-cr*sp*cy,sr*cy+cr*sp*sy,cr*cp;
    Eigen::Matrix3d R_body_to_enu=R_enu_to_body.inverse();
 
    return R_body_to_enu;
}
 
void test_calc_vector_matrix(){
    float roll=-10.0/180*M_PI;
    float pitch=60.0/180*M_PI;
    float yaw=0.0/180*M_PI;
    std::cout<<"init yaw="<<yaw * (180 / M_PI)<<",pitch="<<pitch* (180 / M_PI)<<",roll="<<roll* (180 / M_PI)<<std::endl;
 
    
    //利用单个旋转构造
    Eigen::Matrix3d rot_x,rot_y,rot_z;
    rot_x<<1,0,0,
            0,cos(roll),-sin(roll),
            0,sin(roll),cos(roll);
    
    rot_y<<cos(pitch),0,sin(pitch),
            0,1,0,
            -sin(pitch),0,cos(pitch);
 
    rot_z<<cos(yaw),-sin(yaw),0,
          sin(yaw),cos(yaw),0,
          0,0,1;
 
    //得到的rot是新的坐标轴到旧坐标轴的转换。
    //以向量旋转的角度来看，相当于是得到的了新的坐标轴的x\y\z轴在老坐标轴的表示
    //即T_new_to_old，用这个，就可以把新坐标系下的数值，转换为老坐标系下的值
    //要得到T_old_to_new，取逆就行
    Eigen::Matrix3d rot_new_to_old=rot_z*rot_y*rot_x;
    std::cout<<"rot_new_to_old(body to enu)=\n"<<rot_new_to_old<<std::endl;
 
    Eigen::Matrix3d rot_old_to_new=rot_new_to_old.inverse();//为了做验证运算
    
    
        
    //--测试效果----- 
    Eigen::Vector3d grav_in_old(0,0,-9.81);
    Eigen::Vector3d grav_aft=rot_old_to_new*grav_in_old;
    
    Eigen::Vector3d acc_mea_in_body=grav_aft;
    std::cout<<"grav_in_old = "<<grav_in_old.transpose()<<",acc_mea_in_body = "<<acc_mea_in_body.transpose()
    <<std::endl;
 
    Eigen::Matrix3d R_body_to_enu =  calc_rot_body_to_enu(grav_in_old,acc_mea_in_body);
 
  
 
    std::cout<<",R_body_to_enu=\n"<<R_body_to_enu<<std::endl
    <<"\nresult of (R_body_to_enu-rot_new_to_old)=\n"<<(R_body_to_enu-rot_new_to_old)
    <<std::endl;
 
    gtsam::Rot3 rot2=gtsam::Rot3(R_body_to_enu);
    Eigen::Vector3d calc_ypr=rot2.ypr();
    std::cout<<"calc yaw="<<calc_ypr[0]* (180 / M_PI)
            <<",pitch="<<calc_ypr[1]* (180 / M_PI)
            <<",roll="<<calc_ypr[2]* (180 / M_PI)<<std::endl;
    
}
 
 
输出：
init yaw=0,pitch=60,roll=-10
rot_new_to_old(body to enu)=
       0.5  -0.150384   0.852869
         0   0.984808   0.173648
 -0.866025 -0.0868241   0.492404
grav_in_old =     0     0 -9.81,acc_mea_in_body =  8.49571 0.851744 -4.83048
,R_body_to_enu=
       0.5  -0.150384   0.852869
         0   0.984808   0.173648
 -0.866025 -0.0868241   0.492404
 
result of (R_body_to_enu-rot_new_to_old)=
 2.32568e-08  3.56927e-09 -2.02423e-08
           0  2.24331e-08  3.95556e-09
  4.0282e-08 -6.01628e-09    3.412e-08
calc yaw=-1.37722e-15,pitch=60,roll=-10
```