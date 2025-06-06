#include "ekf.h"

#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
// #include <geometry_msgs/Accel.h>
#include "conversion.h"
#include <std_msgs/Float64.h>
#include "lowpass_filter.hpp"
#include <chrono>
#include <std_msgs/Int32.h>
#include <quadrotor_msgs/SimpleOdom.h>


using namespace std;
using namespace Eigen;
//20200531: time synchronization
//20200105: ekf_node_vio.cpp and ekf_node_mocap.cpp merge into one (ekf_node_vio.cpp) and the differences between them is the odom format
//X_state: p q v gb ab   with time stamp aligned between imu and img
/*
    EKF model
    prediction:
    xt~ = xt-1 + dt*f(xt-1, ut, 0)
    sigmat~ = Ft*sigmat-1*Ft' + Vt*Qt*Vt'
    Update:
    Kt = sigmat~*Ct'*(Ct*sigmat~*Ct' + Wt*Rt*Wt')^-1
    xt = xt~ + Kt*(zt - g(xt~,0))
    sigmat = sigmat~ - Kt*Ct*sigmat~
*/
/*
   -pi ~ pi crossing problem:
   1. the model prpagation: X_state should be limited to [-pi,pi] after predicting and updating
   2. inovation crossing: (measurement - g(X_state)) should also be limited to [-pi,pi] when getting the inovation.
   z_measurement is normally in [-pi~pi]
*/


//odom: pose px,py pz orientation qw qx qy qz
//imu: acc: x y z gyro: wx wy wz

// 20250409 ： Read by Arc
// TODO : add linear velocity compensate for output
// TODO : add angular velocity subscribtion，and add it in so-called sys_seq
//! 注意/mavors/imu/data话题的角速度是在imu的body系下的
// TODO : delete the cam related code , not use it
// TODO : clean up the code, too chaos

//!  imu frame is imu body frame in FLU frame



#define TimeSync 0 //time synchronize or not
#define RePub 0 //re publish the odom when repropagation

ros::Publisher odom_pub;
ros::Publisher cam_odom_pub;
ros::Publisher acc_filtered_pub;
ros::Publisher yaw_rad_pub;
ros::Publisher odom_filtered_pub;
ros::Publisher test_freq_pub;
ros::Publisher simple_odom_pub;

//state
// no use
geometry_msgs::Pose pose;
Vector3d position, orientation, velocity;
// imu filtered angular velocity
Vector3d imu_angular_velocity_;

// Now set up the relevant matrices
//states X [p q pdot]  [px,py,pz, wx,wy,wz, vx,vy,vz]
size_t stateSize;                                // x = [p q pdot bg ba]
size_t stateSize_pqv;                                // x = [p q pdot]
size_t measurementSize;                          // z = [p q]
size_t inputSize;                                // u = [w a]
VectorXd X_state(stateSize);                     // x (in most literature)
VectorXd u_input;
VectorXd Z_measurement;                          // z
MatrixXd StateCovariance;                        // sigma
MatrixXd Kt_kalmanGain;                          // Kt
VectorXd X_state_correct(stateSize);                     // x (in most literature)
MatrixXd StateCovariance_correct;                        // sigma
// MatrixXd Ct_stateToMeasurement;                  // Ct
//  VectorXd innovation;                         // z - Hx

MatrixXd Qt;
MatrixXd Rt;
Vector3d u_gyro;
Vector3d u_acc;
Vector3d gravity(0., 0., -9.8); //need to estimate the bias 9.8099
Vector3d bg_0(0., 0., 0); //need to estimate the bias
Vector3d ba_0(0., 0., 0); //need to estimate the bias  0.1
Vector3d ng(0., 0., 0.);
Vector3d na(0., 0., 0.);
Vector3d nbg(0., 0., 0.);
Vector3d nba(0., 0., 0.);

Vector3d q_last;
Vector3d bg_last;
Vector3d ba_last;

// 刚体质心在imu坐标系下的坐标
double imu_trans_x = 0.0;
double imu_trans_y = 0.0;
double imu_trans_z = 0.0;

//Qt imu covariance matrix  smaller believe system(imu) more
double gyro_cov = 0.01;
double acc_cov = 0.01;
//Rt visual odomtry covariance smaller believe measurement more
double position_cov = 0.1;
double q_rp_cov = 0.1;
double q_yaw_cov = 0.1;

double dt = 0.005; //second
double t_last, t_now;  
bool first_frame_imu = true;
bool first_frame_tag_odom = true;
bool test_odomtag_call = false;
bool odomtag_call = false;

double time_now, time_last;
double time_odom_tag_now;
// double diff_time;

ros::Time last_vio_update_time_ ;

string world_frame_id = "world";

// imu 队列
#define seqsize 60
deque<pair<VectorXd, sensor_msgs::Imu>> sys_seq; // 存储接收到imu数据时的状态和imu测量值，该状态利用imu测量值向前积分一次得到该imu测量时间下的状态
deque<MatrixXd> cov_seq;
double dt_0_rp; // 用来从找到的状态-imu对中，还原到imu测量值对应的状态

// 低通滤波器，平滑输出速度
LowPassFilter2ndOrder lpf_vel_x_(30.0);
LowPassFilter2ndOrder lpf_vel_y_(30.0);
LowPassFilter2ndOrder lpf_vel_z_(30.0);

// 坐标系yaw整体旋转
double frame_rot_yaw = 0.0;

// 向状态-imu序列添加当前状态及其对应的imu测量值到最后
void seq_keep(const sensor_msgs::Imu::ConstPtr &imu_msg)
{
    if(sys_seq.size() < seqsize)
    {
        sys_seq.push_back(make_pair(X_state, *imu_msg));  //X_state before propagation and imu at that time
        cov_seq.push_back(StateCovariance);
    }
    else
    {
        sys_seq.pop_front();
        sys_seq.push_back(make_pair(X_state, *imu_msg));
        cov_seq.pop_front();
        cov_seq.push_back(StateCovariance);
    }
    // ensure that the later frame time > the former one
}

// 找到和vio odom的时间戳对应的imu测量值，找到后把它之前的imu测量值pop掉，只剩下该时间对应的测量值在第一个
//choose the coordinate frame imu for the measurement
bool search_proper_frame(double odom_time)
{
    if(sys_seq.size() <= 1)
    {
        dt_0_rp = 0;
        return false;
    }
        
    size_t rightframe = sys_seq.size()-1;
    bool find_proper_frame = false;
    for(size_t i = 1; i < sys_seq.size(); i++)  // TODO: it better to search from the middle instead in the front
    {
        // 每次取出前后两个imu测量值的时间戳，计算和当前odom时间的差值
        double time_before = odom_time - sys_seq[i-1].second.header.stamp.toSec();  
        double time_after  = odom_time - sys_seq[i].second.header.stamp.toSec();  
        if((time_before >= 0) && (time_after < 0))
        {
            // odom时间戳落在了前后两个imu测量值之间，选择更近的一个
            if(abs(time_before) > abs(time_after))
            {
                rightframe = i;
            }
            else
            {
                rightframe = i-1;
            }

            if(rightframe != 0)
            {
                // 更新dt为当前imu测量值和前一个imu测量值之间的时间差
                dt_0_rp = sys_seq[rightframe].second.header.stamp.toSec() - sys_seq[rightframe-1].second.header.stamp.toSec();
            }
            else
            {   
                // if rightframe is the first frame in the seq, set dt_0_rp as the next dt
                // 如果是队列的第一个，没办法取前一个，只能取下一个dt
                dt_0_rp = sys_seq[rightframe+1].second.header.stamp.toSec() - sys_seq[rightframe].second.header.stamp.toSec();
            }
            
            find_proper_frame = true;
            break;
        }
    }
    if(!find_proper_frame)
    {
        if((odom_time - sys_seq[0].second.header.stamp.toSec()) <= 0)  //if odom time before the first frame, set first frame
        {
            rightframe = 0;
            // if rightframe is the first frame in the seq, set dt_0_rp as the next dt
            dt_0_rp = sys_seq[rightframe+1].second.header.stamp.toSec() - sys_seq[rightframe].second.header.stamp.toSec();
        }
        if((odom_time - sys_seq[sys_seq.size()-1].second.header.stamp.toSec()) >= 0)  //if odom time after the last frame, set last frame
        {
            rightframe = sys_seq.size()-1;
            dt_0_rp = sys_seq[rightframe].second.header.stamp.toSec() - sys_seq[rightframe-1].second.header.stamp.toSec();
        }
        //no process, set the latest one
    }

    //set the right frame as the first frame in the queue
    // pop掉找到的imu测量值之前的imu测量值
    for(size_t i = 0; i < rightframe; i++)  
    {
        sys_seq.pop_front();
        cov_seq.pop_front();
    }
    
    if(find_proper_frame)
    {
        return true;
    }
    else
    {
        return false;
    }
}

// 从序列的头开始向前传播，传播到结尾得到当前的状态
void re_propagate()
{
    // 从第二个开始取，因为第一个已经用来做update前的状态还原了
    for(size_t i=1; i < sys_seq.size(); i++)
    {
        //re-prediction for the rightframe 
        dt = sys_seq[i].second.header.stamp.toSec() - sys_seq[i-1].second.header.stamp.toSec();

        u_gyro(0) = sys_seq[i].second.angular_velocity.x;
        u_gyro(1) = sys_seq[i].second.angular_velocity.y;
        u_gyro(2) = sys_seq[i].second.angular_velocity.z;
        u_acc(0)  = sys_seq[i].second.linear_acceleration.x;
        u_acc(1)  = sys_seq[i].second.linear_acceleration.y;
        u_acc(2)  = sys_seq[i].second.linear_acceleration.z;

        MatrixXd Ft;
        MatrixXd Vt;

        q_last = sys_seq[i].first.segment<3>(3);  // last X2
        bg_last = sys_seq[i].first.segment<3>(9);  //last X4
        ba_last = sys_seq[i].first.segment<3>(12);  //last X5
        Ft = MatrixXd::Identity(stateSize, stateSize) + dt*diff_f_diff_x(q_last, u_gyro, u_acc, bg_last, ba_last);
        
        Vt = dt*diff_f_diff_n(q_last);

        X_state += dt*F_model(u_gyro, u_acc);
        if(X_state(3) > PI)  X_state(3) -= 2*PI;
        if(X_state(3) < -PI) X_state(3) += 2*PI;
        if(X_state(4) > PI)  X_state(4) -= 2*PI;
        if(X_state(4) < -PI) X_state(4) += 2*PI;
        if(X_state(5) > PI)  X_state(5) -= 2*PI;
        if(X_state(5) < -PI) X_state(5) += 2*PI;
        StateCovariance = Ft*StateCovariance*Ft.transpose() + Vt*Qt*Vt.transpose();

        #if RePub
        system_pub(sys_seq[i].second.header.stamp);  // choose to publish the repropagation or not
        #endif

    }
}

// 接受到一个imu raw测量值
void imu_raw_callback(const sensor_msgs::Imu::ConstPtr &msg)
{
    // seq_keep(msg);
    // nav_msgs::Odometry odom_fusion;
    // your code for propagation
    // std::cout << "imu: (" << msg->linear_acceleration.x << ", " << msg->linear_acceleration.y << ")\n";
    if(!first_frame_tag_odom)
    { //get the initial pose and orientation in the first frame of measurement 
        if(first_frame_imu)
        {
            first_frame_imu = false;
            time_now = msg->header.stamp.toSec();
            time_last = time_now;
            #if TimeSync
            seq_keep(msg);//keep before propagation
            #endif

            system_pub(msg->header.stamp);
        }
        else
        {
            #if TimeSync
            // 将当前状态和该imu测量值添加到队列中，也就是存起来
            // 所以测量值的时间对应的状态实际上需要imu积分一次后的状态
            seq_keep(msg);//keep before propagation 
            #endif

            time_now = msg->header.stamp.toSec();
            dt = time_now - time_last;

            // 检查当前imu测量值和最新的vio测量值的时间差，并cout出来
            if(odomtag_call)
            {
                odomtag_call = false;
                // diff_time = time_now - time_odom_tag_now;
                // if(diff_time<0) // 处理这个imu的时候来了个vio odom
                // {
                //     cout << "diff time: " << diff_time << endl;  //???!!! exist !!!???
                //     cout << "timeimu: " << time_now - 1.60889e9 << " time_odom: " << time_odom_tag_now - 1.60889e9 << endl;
                //     // cout << "diff time: " << diff_time << endl;  //about 30ms
                // }
            }
            MatrixXd Ft;
            MatrixXd Vt;

            u_gyro(0) = msg->angular_velocity.x; // imu原始角速度
            u_gyro(1) = msg->angular_velocity.y;
            u_gyro(2) = msg->angular_velocity.z;
            u_acc(0)  = msg->linear_acceleration.x; // imu原始加速度
            u_acc(1)  = msg->linear_acceleration.y;
            u_acc(2)  = msg->linear_acceleration.z;

            q_last = X_state.segment<3>(3);  // last X2
            bg_last = X_state.segment<3>(9);  //last X4
            ba_last = X_state.segment<3>(12);  //last X5

            // 更新协方差
            Ft = MatrixXd::Identity(stateSize, stateSize) + dt*diff_f_diff_x(q_last, u_gyro, u_acc, bg_last, ba_last);
            Vt = dt*diff_f_diff_n(q_last);

            acc_f_pub(u_acc, msg->header.stamp);
            X_state += dt*F_model(u_gyro, u_acc);
            if(X_state(3) > PI)  X_state(3) -= 2*PI;
            if(X_state(3) < -PI) X_state(3) += 2*PI;
            if(X_state(4) > PI)  X_state(4) -= 2*PI;
            if(X_state(4) < -PI) X_state(4) += 2*PI;
            if(X_state(5) > PI)  X_state(5) -= 2*PI;
            if(X_state(5) < -PI) X_state(5) += 2*PI;
            StateCovariance = Ft*StateCovariance*Ft.transpose() + Vt*Qt*Vt.transpose();
 
            time_last = time_now;
            
            // if(test_odomtag_call) //no frequency boost
            // {
            //     test_odomtag_call = false;
            //     system_pub(msg->header.stamp);
            // }

            // 每个imu测量值来了就发布一次ekf输出
            system_pub(msg->header.stamp);

        }
    }
  
}

//Rotation from the camera frame to the IMU frame
Matrix3d Rc_i;     
Vector3d tc_i;  //  cam in imu frame
int cnt = 0;
Vector3d INNOVATION_;

//  vio odom in imu frame
Matrix3d Rr_i;  
Vector3d tr_i;  


//msg is imu in world
VectorXd get_pose_from_mocap(const geometry_msgs::PoseStamped::ConstPtr &msg) 
{
    Matrix3d Rr_w;    //rigid body in world
    Vector3d tr_w;
    Matrix3d Ri_w;  
    Vector3d ti_w;
    Vector3d p_temp;
    p_temp(0) = msg->pose.position.x;
    p_temp(1) = msg->pose.position.y;
    p_temp(2) = msg->pose.position.z;

    //quaternion2euler:  ZYX  roll pitch yaw
    Quaterniond q;
    q.w() = msg->pose.orientation.w;
    q.x() = msg->pose.orientation.x;
    q.y() = msg->pose.orientation.y;
    q.z() = msg->pose.orientation.z;

    // Quaterniond q_mid1, q_mid2;
    // q_mid1 = Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d::UnitX());
    // q_mid2 = Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d::UnitX());
    // p_temp = q_mid1 * p_temp;
    // q = q_mid1 * q * q_mid2;
    
    //Euler transform
    Rr_w = q.toRotationMatrix();
    tr_w = p_temp;
    Ri_w = Rr_w * Rr_i.inverse();
    ti_w = tr_w - Ri_w * tr_i;
    Vector3d euler = mat2euler(Ri_w);

    VectorXd pose = VectorXd::Random(6);
    pose.segment<3>(0) = ti_w;
    pose.segment<3>(3) = euler;

    return pose;
}
VectorXd get_pose_from_VIOodom(const nav_msgs::Odometry::ConstPtr &msg) 
{
    Matrix3d Rr_w;    //rigid body in world
    Vector3d tr_w;
    Matrix3d Ri_w;  
    Vector3d ti_w;
    Vector3d p_temp;
    p_temp(0) = msg->pose.pose.position.x;
    p_temp(1) = msg->pose.pose.position.y;
    p_temp(2) = msg->pose.pose.position.z;
    Quaterniond q;
    q.w() = msg->pose.pose.orientation.w;
    q.x() = msg->pose.pose.orientation.x;
    q.y() = msg->pose.pose.orientation.y;
    q.z() = msg->pose.pose.orientation.z;
    
    //Euler transform
    // Ri_w = q.toRotationMatrix();
    // ti_w = p_temp;
    Rr_w = q.toRotationMatrix(); // 这里假定了vio odom的输入是在刚体坐标系下的
    tr_w = p_temp;
    Ri_w = Rr_w * Rr_i.inverse();
    ti_w = tr_w - Ri_w*tr_i;
    Vector3d euler = mat2euler(Ri_w);     //quaternion2euler:  ZYX  roll pitch yaw

    VectorXd pose = VectorXd::Random(6);
    pose.segment<3>(0) = ti_w;
    pose.segment<3>(3) = euler;

    return pose;
}
void vioodom_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{//assume that the odom_tag from camera is sychronized with the imus and without delay. !!!

    last_vio_update_time_ = ros::Time::now();
    //your code for update
    if(first_frame_tag_odom)
    {//system begins in first odom frame
        first_frame_tag_odom = false;
        time_odom_tag_now = msg->header.stamp.toSec();

        VectorXd odom_pose = get_pose_from_mocap(msg);
        // std::cout << "msg: (" << msg->pose.position.x << ", " << msg->pose.position.y << ")\n";
        X_state.segment<3>(0) = odom_pose.segment<3>(0);
        X_state.segment<3>(3) = odom_pose.segment<3>(3);

        world_frame_id = msg->header.frame_id;

    }
    else
    {
        time_odom_tag_now = msg->header.stamp.toSec();
        //    double t = clock();
        MatrixXd Ct;
        MatrixXd Wt;

        VectorXd odom_pose = get_pose_from_mocap(msg);
        // std::cout << "msg: (" << msg->pose.position.x << ", " << msg->pose.position.y << ")\n";

        #if TimeSync
        //call back to the proper time
        // 找到该vio odom时间戳对应的imu测量值，把它放在队列的最前面
        search_proper_frame(time_odom_tag_now);
        #endif

        Z_measurement.segment<3>(0) = odom_pose.segment<3>(0);
        Z_measurement.segment<3>(3) = odom_pose.segment<3>(3);
        // cam_system_pub(msg->header.stamp);
       
        #if !TimeSync //no aligned
        Ct = diff_g_diff_x();
        Wt = diff_g_diff_v();

        Kt_kalmanGain = StateCovariance*Ct.transpose() * (Ct*StateCovariance*Ct.transpose() + Wt*Rt*Wt.transpose()).inverse();
        VectorXd gg = g_model();
        VectorXd innovation = Z_measurement - gg;
        VectorXd innovation_t = gg;
    
        //Prevent innovation changing suddenly when euler from -Pi to Pi
        
        if(innovation(3) > 6)  innovation(3) -= 2*PI;
        if(innovation(3) < -6) innovation(3) += 2*PI;
        if(innovation(4) > 6)  innovation(4) -= 2*PI;
        if(innovation(4) < -6) innovation(4) += 2*PI;
        if(innovation(5) > 6)  innovation(5) -= 2*PI;
        if(innovation(5) < -6) innovation(5) += 2*PI;
        INNOVATION_ = innovation_t.segment<3>(3);
        X_state += Kt_kalmanGain*(innovation);
        if(X_state(3) > PI)  X_state(3) -= 2*PI;
        if(X_state(3) < -PI) X_state(3) += 2*PI;
        if(X_state(4) > PI)  X_state(4) -= 2*PI;
        if(X_state(4) < -PI) X_state(4) += 2*PI;
        if(X_state(5) > PI)  X_state(5) -= 2*PI;
        if(X_state(5) < -PI) X_state(5) += 2*PI;
        StateCovariance = StateCovariance - Kt_kalmanGain*Ct*StateCovariance;

        // ROS_INFO("time cost: %f\n", (clock() - t) / CLOCKS_PER_SEC);
        // cout << "z " << Z_measurement(2) << " k " << Kt_kalmanGain(2) << " inn " << innovation(2) << endl;
        
        cam_system_pub(msg->header.stamp);
        test_odomtag_call = true;
        odomtag_call = true;
        
        if(INNOVATION_(0)>6 || INNOVATION_(1)>6 || INNOVATION_(2)>6)
        cout << "\ninnovation: \n" << INNOVATION_ << endl; 
        if(INNOVATION_(0)<-6 || INNOVATION_(1)<-6 || INNOVATION_(2)<-6)
        cout << "\ninnovation: \n" << INNOVATION_ << endl; 
        //monitor the position changing
        if((innovation(0)>1.5) || (innovation(1)>1.5) || (innovation(2)>1.5) ||
           (innovation(0)<-1.5) || (innovation(1)<-1.5) || (innovation(2)<-1.5))
           ROS_ERROR("posintion diff too much between measurement and model prediction!!!");
        if(cnt == 10||cnt==50||cnt==90)
        {
            // cout << "Ct: \n" << Ct << "\nWt:\n" << Wt << endl; 
            // cout << "Kt_kalmanGain: \n" << Kt_kalmanGain << endl; 
            // cout << "\ninnovation: \n" << Kt_kalmanGain*innovation  << "\ndt:\n" << dt << endl;
            // cout << "\ninnovation: \n" << Kt_kalmanGain*innovation  << endl;
            // cout << "\ninnovation: \n" << INNOVATION_ << endl; 
        } 
        cnt++;
        if(cnt>100) cnt=101;

        #else  // time sync

        //re-prediction for the rightframe 
        // 往前进一步，前进一步的状态就是imu测量值对应的状态了，可用来进行update     
        dt = dt_0_rp;

        u_gyro(0) = sys_seq[0].second.angular_velocity.x;
        u_gyro(1) = sys_seq[0].second.angular_velocity.y;
        u_gyro(2) = sys_seq[0].second.angular_velocity.z;
        u_acc(0)  = sys_seq[0].second.linear_acceleration.x;
        u_acc(1)  = sys_seq[0].second.linear_acceleration.y;
        u_acc(2)  = sys_seq[0].second.linear_acceleration.z;

        MatrixXd Ft;
        MatrixXd Vt;

        X_state = sys_seq[0].first; // 状态取自队列的第一个，也就是该vio odom对应的imu测量值时的状态
        StateCovariance  = cov_seq[0];

        q_last = sys_seq[0].first.segment<3>(3);  // last X2
        bg_last = sys_seq[0].first.segment<3>(9);  //last X4
        ba_last = sys_seq[0].first.segment<3>(12);  //last X5
        Ft = MatrixXd::Identity(stateSize, stateSize) + dt*diff_f_diff_x(q_last, u_gyro, u_acc, bg_last, ba_last);
        
        Vt = dt*diff_f_diff_n(q_last);

        X_state += dt*F_model(u_gyro, u_acc); 
        if(X_state(3) > PI)  X_state(3) -= 2*PI;
        if(X_state(3) < -PI) X_state(3) += 2*PI;
        if(X_state(4) > PI)  X_state(4) -= 2*PI;
        if(X_state(4) < -PI) X_state(4) += 2*PI;
        if(X_state(5) > PI)  X_state(5) -= 2*PI;
        if(X_state(5) < -PI) X_state(5) += 2*PI;
        StateCovariance = Ft*StateCovariance*Ft.transpose() + Vt*Qt*Vt.transpose();

        // 使用vio odom测量值进行update
        //re-update for the rightframe 
        Ct = diff_g_diff_x();
        Wt = diff_g_diff_v();

        Kt_kalmanGain = StateCovariance*Ct.transpose() * (Ct*StateCovariance*Ct.transpose() + Wt*Rt*Wt.transpose()).inverse();

        VectorXd gg = g_model();
        VectorXd innovation = Z_measurement - gg;
        VectorXd innovation_t = gg;
        //Prevent innovation changing suddenly when euler from -Pi to Pi
        if(innovation(3) > 6)  innovation(3) -= 2*PI;
        if(innovation(3) < -6) innovation(3) += 2*PI;
        if(innovation(4) > 6)  innovation(4) -= 2*PI;
        if(innovation(4) < -6) innovation(4) += 2*PI;
        if(innovation(5) > 6)  innovation(5) -= 2*PI;
        if(innovation(5) < -6) innovation(5) += 2*PI;
        INNOVATION_ = innovation_t.segment<3>(3);

        // update
        X_state += Kt_kalmanGain*(innovation);
        if(X_state(3) > PI)  X_state(3) -= 2*PI;
        if(X_state(3) < -PI) X_state(3) += 2*PI;
        if(X_state(4) > PI)  X_state(4) -= 2*PI;
        if(X_state(4) < -PI) X_state(4) += 2*PI;
        if(X_state(5) > PI)  X_state(5) -= 2*PI;
        if(X_state(5) < -PI) X_state(5) += 2*PI;
        StateCovariance = StateCovariance - Kt_kalmanGain*Ct*StateCovariance;

        system_pub(sys_seq[0].second.header.stamp);  // choose to publish the repropagation or not

        // 用队列里的imu数据向前积分回到当前时间到状态
        re_propagate();

        #endif
    }
    
}

// ground truth ，used to examine the accuracy of the system
Quaterniond q_gt, q_gt0;
bool first_gt = true;
void gt_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    q_gt.w() = msg->pose.pose.orientation.w;
    q_gt.x() = msg->pose.pose.orientation.x;
    q_gt.y() = msg->pose.pose.orientation.y;
    q_gt.z() = msg->pose.pose.orientation.z;

    if(first_gt&&!first_frame_tag_odom)
    {
        first_gt = false;
        q_gt0 = q_gt;
        // q_gt0 = q_gt0.normalized();
    }
}

void imu_att_callback(const sensor_msgs::Imu::ConstPtr &msg)
{
    imu_angular_velocity_(0) = msg->angular_velocity.x;
    imu_angular_velocity_(1) = msg->angular_velocity.y;
    imu_angular_velocity_(2) = msg->angular_velocity.z;
}

void timerCallback(const ros::TimerEvent &event)
{
    nav_msgs::Odometry test_freq;
    test_freq.header.stamp = ros::Time(0);
    test_freq.header.frame_id = "world";
    test_freq_pub.publish(test_freq);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ekf");
    ros::NodeHandle n("~");
    ros::Subscriber imu_raw_sub = n.subscribe("imu", 1000, imu_raw_callback, ros::TransportHints().tcpNoDelay());
    // 怪不得叫body odometry，就是假设该odom描述刚体坐标系下，位置代表其质心坐标，速度代表其质心速度
    ros::Subscriber body_odom_sub = n.subscribe<geometry_msgs::PoseStamped>("bodyodometry", 40, vioodom_callback, ros::TransportHints().tcpNoDelay());  
    ros::Subscriber imu_att_sub = n.subscribe("/mavros/imu/data", 1000, imu_att_callback, ros::TransportHints().tcpNoDelay());

    // ground truth
    ros::Subscriber s4 = n.subscribe("gt_", 40, gt_callback, ros::TransportHints().tcpNoDelay()); 
    odom_pub = n.advertise<nav_msgs::Odometry>("ekf_odom", 1000);   //freq = imu freq
    yaw_rad_pub = n.advertise<std_msgs::Float64>("yaw_rad", 4); //pub_yaw_radius value
    cam_odom_pub = n.advertise<nav_msgs::Odometry>("cam_ekf_odom", 100);
    acc_filtered_pub = n.advertise<geometry_msgs::PoseStamped>("acc_filtered", 1000);
    odom_filtered_pub = n.advertise<nav_msgs::Odometry>("ekf_odom_filtered", 1000); 
    test_freq_pub = n.advertise<nav_msgs::Odometry>("test_freq", 100);
    simple_odom_pub = n.advertise<quadrotor_msgs::SimpleOdom>("simple_odom", 1000);
    

    ros::Timer timer = n.createTimer(ros::Duration(0.005), timerCallback);
    
    n.getParam("gyro_cov", gyro_cov);
    n.getParam("acc_cov", acc_cov);
    n.getParam("position_cov", position_cov);
    n.getParam("q_rp_cov", q_rp_cov);
    n.getParam("q_yaw_cov", q_yaw_cov);
    n.getParam("imu_trans_x", imu_trans_x);
    n.getParam("imu_trans_y", imu_trans_y);
    n.getParam("imu_trans_z", imu_trans_z);
    n.getParam("frame_rot_yaw", frame_rot_yaw);

    cout << "Q:" << gyro_cov << " " << acc_cov << " R: " << position_cov << " " << q_rp_cov << " " << q_yaw_cov << endl;
    
    std::vector<double> Rri, tri;
    n.getParam("Rr_i", Rri);
    n.getParam("tr_i", tri);
    Rr_i = Quaterniond(Rri.at(0), Rri.at(1), Rri.at(2), Rri.at(3)).toRotationMatrix();
    tr_i << tri.at(0), tri.at(1), tri.at(2);
    cout << "Rr_i: " << endl << Rr_i << endl;
    cout << "tr_i: " << endl << tr_i << endl;

    initsys();
    cout << "initsys" << endl;

    // cout << "======================" << endl;
    // double r = atan2(1,-100);
    // double p = asin(-0.707);
    // double y = atan2(-1, -100);   
    // cout << "r: " << r << " p: " << p << " y: " << y << endl;
    // cout << "======================" << endl;

    last_vio_update_time_ = ros::Time(0);

    // 2. 创建 AsyncSpinner（8个线程）
    ros::AsyncSpinner spinner(8);  // 使用 8 个线程处理回调
    spinner.start();  // 启动异步 spinner
    
    // 保持节点运行
    ros::Rate rate(1000);  // 设置循环频率为 1000Hz
    while (ros::ok())
    {
        rate.sleep();  // 等待下一个循环
    }

    return 0;
}

void acc_f_pub(Vector3d acc, ros::Time stamp)
{
    geometry_msgs::PoseStamped Accel_filtered;
    Accel_filtered.header.frame_id = "world";
    Accel_filtered.header.stamp = stamp;
    // Accel_filtered.header.stamp = ros::Time::now();
    Vector3d Acc_ = get_filtered_acc(acc);
    Accel_filtered.pose.position.x = Acc_[0];
    Accel_filtered.pose.position.y = Acc_[1];
    Accel_filtered.pose.position.z = Acc_[2];

    Quaterniond q;
    q = euler2quaternion(X_state.segment<3>(3));
    // q = q.normalized();
    Accel_filtered.pose.orientation.w = q.w();
    Accel_filtered.pose.orientation.x = q.x();
    Accel_filtered.pose.orientation.y = q.y();
    Accel_filtered.pose.orientation.z = q.z();

    // cout << "q_gt0: " << quaternion2euler(q_gt0) << endl;
    // cout << "q_gt: " << quaternion2euler(q_gt) << endl;
    // cout << "q_vio: " << mat2euler(q_gt0.toRotationMatrix() * euler2quaternion(X_state.segment<3>(3)).toRotationMatrix()) << endl;
    // cout << "q_gt0*vio != q_gt: " << quaternion2euler(q_gt0 * q) << endl;   
    // cout << "q_gt0*vio*q_gt0^-1 = q_gt: " << quaternion2euler(q_gt0 * q * q_gt0.inverse()) << endl;   //q1*euler2quaternion(V)*q1.inverse()  = q1.toRotationMatrix() * V  TODO why?

    acc_filtered_pub.publish(Accel_filtered);
}
void system_pub(ros::Time stamp)
{
    // auto tic_begin_timer = std::chrono::steady_clock::now();

    if ((ros::Time::now() - last_vio_update_time_).toSec() > 0.1) {
        ROS_ERROR_THROTTLE(1.0,"vio/gps message timeout. Stopping ekf_odom publishing.");
        return;
    }

    nav_msgs::Odometry odom_fusion;
    odom_fusion.header.stamp = stamp;
    // odom_fusion.header.frame_id = world_frame_id;
    // odom_fusion.header.frame_id = "map";
    odom_fusion.header.frame_id = "world";
    // odom_fusion.header.frame_id = "imu";
    
    // attitude
    Quaterniond q;
    q = euler2quaternion(X_state.segment<3>(3));
    odom_fusion.pose.pose.orientation.w = q.w();
    odom_fusion.pose.pose.orientation.x = q.x();
    odom_fusion.pose.pose.orientation.y = q.y();
    odom_fusion.pose.pose.orientation.z = q.z();

    // body linear velocity
    Vector3d imu_vel_in_world = X_state.segment<3>(6); // we get the imu velocity in world frame

    // 假定imu的角速度和刚体的角速度是相同的，假定imu_angular_velocity_的角速度是当前的角速度
    // 已知imu处的线速度，imu的角速度以及质心到imu的坐标Vector3d(-imu_trans_x,-imu_trans_y,-imu_trans_z)
    // imu相对于质心的线速度 等于 imu_angular_velocity_ 叉乘 Vector3d(-imu_trans_x,-imu_trans_y,-imu_trans_z)
    Vector3d imu_pos_rel_body_in_body =  Vector3d(-imu_trans_x,-imu_trans_y,-imu_trans_z);
    Vector3d imu_vel_rel_body_in_body =  imu_angular_velocity_.cross(imu_pos_rel_body_in_body);
    // transform this offset to world frame
    Vector3d imu_vel_rel_body_in_world = q.toRotationMatrix() * imu_vel_rel_body_in_body;
    // 质心的线速度 = imu的线速度 + 刚体相对于imu的线速度 = imu的线速度 - imu相对于刚体的线速度
    // velocity of the center of mass in world frame = velocity of imu in world frame - velocity of imu in body frame
    Vector3d body_vel_in_world = imu_vel_in_world - imu_vel_rel_body_in_world;

    odom_fusion.twist.twist.linear.x = body_vel_in_world(0);
    odom_fusion.twist.twist.linear.y = body_vel_in_world(1);
    odom_fusion.twist.twist.linear.z = body_vel_in_world(2);

    // body position
    // pos_conter: imu在世界坐标系下的坐标， pos_center2: 质心在世界坐标系下的坐标
    Vector3d imu_pos(X_state(0),X_state(1),X_state(2)),body_pos;

    body_pos = imu_pos + q.toRotationMatrix() * Vector3d(imu_trans_x,imu_trans_y,imu_trans_z);
    odom_fusion.pose.pose.position.x = body_pos(0);
    odom_fusion.pose.pose.position.y = body_pos(1);
    odom_fusion.pose.pose.position.z = body_pos(2);

    // std::cout << "odom: (" << pos_center2(0) << ", " << pos_center2(1) << ")\n";


    // static ros::Time last_pub_time = ros::Time(0);
    // ros::Time now_time = ros::Time::now();
    // double pub_dt = (now_time - last_pub_time).toSec();
    // last_pub_time = now_time;
    // std::cout << "pub_dt: " << pub_dt << std::endl;
    // odom_pub.publish(odom_fusion);


    // lpf
    static ros::Time last_stamp = ros::Time(0);
    double dt = last_stamp.isZero() ? 0.01 : (stamp - last_stamp).toSec();

    // limit dt to 100 to 300 hz , warn when exceed
    if(dt > 0.01)
    {
        ROS_WARN_THROTTLE(1.0,"dt is too large: %f", dt);
        dt = 0.01;
    }
    if(dt < 0.003)
    {
        ROS_WARN_THROTTLE(1.0,"dt is too small: %f", dt);
        dt = 0.003;
    }

    last_stamp = stamp;
    double body_vel_x_filtered = lpf_vel_x_.update(body_vel_in_world.x(),dt);
    double body_vel_y_filtered = lpf_vel_y_.update(body_vel_in_world.y(),dt);
    double body_vel_z_filtered = lpf_vel_z_.update(body_vel_in_world.z(),dt);

    odom_fusion.twist.twist.linear.x = body_vel_x_filtered;
    odom_fusion.twist.twist.linear.y = body_vel_y_filtered;
    odom_fusion.twist.twist.linear.z = body_vel_z_filtered;

    odom_fusion.twist.twist.angular.x = imu_angular_velocity_(0);
    odom_fusion.twist.twist.angular.y = imu_angular_velocity_(1);
    odom_fusion.twist.twist.angular.z = imu_angular_velocity_(2);

    // odom_filtered_pub.publish(odom_fusion);
    // odom_pub.publish(odom_fusion);

    nav_msgs::Odometry odom_fusion_rot;
    odom_fusion_rot.header.stamp = stamp;
    odom_fusion_rot.header.frame_id = "world";
    // 对odom_fusion进行绕着世界系z轴的旋转
    // 角度转换为弧度
    double frame_rot_yaw_rad = frame_rot_yaw * M_PI / 180.0;
    Quaterniond q_rot = euler2quaternion(Vector3d(0, 0, frame_rot_yaw_rad));
    Quaterniond q_rot_odom = q_rot * q;
    odom_fusion_rot.pose.pose.orientation.w = q_rot_odom.w();
    odom_fusion_rot.pose.pose.orientation.x = q_rot_odom.x();
    odom_fusion_rot.pose.pose.orientation.y = q_rot_odom.y();
    odom_fusion_rot.pose.pose.orientation.z = q_rot_odom.z();

    odom_fusion_rot.pose.pose.position.x = body_pos(0) * cos(frame_rot_yaw_rad) - body_pos(1) * sin(frame_rot_yaw_rad);
    odom_fusion_rot.pose.pose.position.y = body_pos(0) * sin(frame_rot_yaw_rad) + body_pos(1) * cos(frame_rot_yaw_rad);
    odom_fusion_rot.pose.pose.position.z = body_pos(2);

    odom_fusion_rot.twist.twist.linear.x = body_vel_x_filtered * cos(frame_rot_yaw_rad) - body_vel_y_filtered * sin(frame_rot_yaw_rad);
    odom_fusion_rot.twist.twist.linear.y = body_vel_x_filtered * sin(frame_rot_yaw_rad) + body_vel_y_filtered * cos(frame_rot_yaw_rad);
    odom_fusion_rot.twist.twist.linear.z = body_vel_z_filtered;

    odom_fusion_rot.twist.twist.angular.x = imu_angular_velocity_(0);
    odom_fusion_rot.twist.twist.angular.y = imu_angular_velocity_(1);
    odom_fusion_rot.twist.twist.angular.z = imu_angular_velocity_(2);

    // static ros::Time last_pub_time = ros::Time(0);
    // ros::Time now_time = ros::Time::now();
    // std::cout<<" dt = "<<(now_time - last_pub_time).toSec()<<std::endl;
    // last_pub_time = now_time;
    odom_pub.publish(odom_fusion_rot);

    // quadrotor_msgs::SimpleOdom simple_odom;
    // simple_odom.header.stamp = stamp;
    // simple_odom.header.frame_id = "world";
    // simple_odom.pose = odom_fusion_rot.pose.pose;
    // simple_odom.twist = odom_fusion_rot.twist.twist;
    // simple_odom_pub.publish(simple_odom);

    std_msgs::Float64 yaw_rad;
    double qua_w = 1.0, qua_x = 0.0, qua_y = 0.0, qua_z = 0.0;
    qua_w = q.w();
    qua_x = q.x();
    qua_y = q.y();
    qua_z = q.z();
    yaw_rad.data = atan2(2*(qua_w*qua_z+qua_x*qua_y),1-2*(qua_y*qua_y+qua_z*qua_z));
    // yaw_rad_pub.publish(yaw_rad);

    // auto toc_now = std::chrono::steady_clock::now();
    // std::cout << "till now costs in timer: " << (toc_now - tic_begin_timer).count() << "nanoseconds" << std::endl;
}
void cam_system_pub(ros::Time stamp)
{
    nav_msgs::Odometry odom_fusion;
    odom_fusion.header.stamp = stamp;
    odom_fusion.header.frame_id = world_frame_id;
    // odom_fusion.header.frame_id = "imu";
    // odom_fusion.pose.pose.position.x = Z_measurement(0);
    // odom_fusion.pose.pose.position.y = Z_measurement(1);
    // odom_fusion.pose.pose.position.z = Z_measurement(2);
    odom_fusion.pose.pose.position.x = Z_measurement(0); // vio的位置
    odom_fusion.pose.pose.position.y = Z_measurement(1);
    odom_fusion.pose.pose.position.z = Z_measurement(2);
    Quaterniond q;
    q = euler2quaternion(Z_measurement.segment<3>(3)); // vio的欧拉角
    odom_fusion.pose.pose.orientation.w = q.w();
    odom_fusion.pose.pose.orientation.x = q.x();
    odom_fusion.pose.pose.orientation.y = q.y();
    odom_fusion.pose.pose.orientation.z = q.z();
    odom_fusion.twist.twist.linear.x = Z_measurement(3); // vio的线速度
    odom_fusion.twist.twist.linear.y = Z_measurement(4);
    odom_fusion.twist.twist.linear.z = Z_measurement(5);
    
    // odom_fusion.twist.twist.angular.x = INNOVATION_(0);
    // odom_fusion.twist.twist.angular.y = INNOVATION_(1);
    // odom_fusion.twist.twist.angular.z = INNOVATION_(2);
    Vector3d pp, qq, v, bg, ba;
    getState(pp, qq, v, bg, ba);
    odom_fusion.twist.twist.angular.x = ba(0); // kidding me?? seems like just to fill the data, whatever it is
    odom_fusion.twist.twist.angular.y = ba(1);///??????why work??????????//////
    odom_fusion.twist.twist.angular.z = ba(2); 
    // cam_odom_pub.publish(odom_fusion);
}

//process model
void initsys()
{
    //  camera position in the IMU frame = (0.05, 0.05, 0)
    // camera orientaion in the IMU frame = Quaternion(0, 1, 0, 0); w x y z, respectively
    //					   RotationMatrix << 1, 0, 0,
    //							             0, -1, 0,
    //                                       0, 0, -1;
    //set the cam2imu params
    Rc_i = Quaterniond(0, 1, 0, 0).toRotationMatrix();
    // cout << "R_cam" << endl << Rc_i << endl;
    tc_i << 0.05, 0.05, 0; 
    
    //  rigid body position in the IMU frame = (0, 0, 0.04)
    // rigid body orientaion in the IMU frame = Quaternion(1, 0, 0, 0); w x y z, respectively
    //					   RotationMatrix << 1, 0, 0,
    //						 	             0, 1, 0,
    //                                       0, 0, 1; 

    //states X [p q pdot bg ba]  [px,py,pz, wx,wy,wz, vx,vy,vz bgx,bgy,bgz bax,bay,baz]
    stateSize = 15;                                                                 // x = [p q pdot bg ba]
    stateSize_pqv = 9;                                                              // x = [p q pdot]
    measurementSize = 6;                                                            // z = [p q]
    inputSize = 6;                                                                  // u = [w a]
    X_state = VectorXd::Zero(stateSize);                                            // x 
    //velocity
    X_state(6) = 0;
    X_state(7) = 0;
    X_state(8) = 0; 
    //bias
    X_state.segment<3>(9) = bg_0;
    X_state.segment<3>(12) = ba_0;
    u_input = VectorXd::Zero(inputSize);
    Z_measurement = VectorXd::Zero(measurementSize);                                // z
    StateCovariance = MatrixXd::Identity(stateSize, stateSize);                     // sigma
    Kt_kalmanGain = MatrixXd::Identity(stateSize, measurementSize);                 // Kt
    // Ct_stateToMeasurement = MatrixXd::Identity(stateSize, measurementSize);         // Ct
    X_state_correct = X_state;
    StateCovariance_correct = StateCovariance;

    Qt = MatrixXd::Identity(inputSize, inputSize);  //6x6 input [gyro acc]covariance
    Rt = MatrixXd::Identity(measurementSize, measurementSize); //6x6 measurement [p q]covariance
    // MatrixXd temp_Rt = MatrixXd::Identity(measurementSize, measurementSize);

    // You should also tune these parameters
    // Q imu covariance matrix; Rt visual odomtry covariance matrix
    // //Rt visual odomtry covariance smaller believe measurement more
    Qt.topLeftCorner(3, 3) = gyro_cov * Qt.topLeftCorner(3, 3);
    Qt.bottomRightCorner(3, 3) = acc_cov * Qt.bottomRightCorner(3, 3);
    Rt.topLeftCorner(3, 3) = position_cov * Rt.topLeftCorner(3, 3);
    Rt.bottomRightCorner(3, 3) = q_rp_cov * Rt.bottomRightCorner(3, 3);
    Rt.bottomRightCorner(1, 1) = q_yaw_cov * Rt.bottomRightCorner(1, 1);
}

void getState(Vector3d& p, Vector3d& q, Vector3d& v, Vector3d& bg, Vector3d& ba)
{
	p = X_state.segment<3>(0);
	q = X_state.segment<3>(3);
	v = X_state.segment<3>(6);
    bg = X_state.segment<3>(9);
    ba = X_state.segment<3>(12);
}

VectorXd get_filtered_acc(Vector3d acc)
{
    Vector3d q, ba;
    q = X_state.segment<3>(3);
    ba = X_state.segment<3>(12);

    // return (euler2mat(q)*(acc-ba-na));
    // return (q_gt.toRotationMatrix()*(acc-ba-na));  //false
    return ((acc-ba-na)); // 补偿对加速度偏置和加速度噪声的估计
    // return ((acc-na));
    // return (euler2mat(q)*(acc));
}

VectorXd F_model(Vector3d gyro, Vector3d acc)
{
    // IMU is in FLU frame
    // Transform IMU frame into "world" frame whose original point is FLU's original point and the XOY plain is parallel with the ground and z axis is up
    VectorXd f(VectorXd::Zero(stateSize));
    Vector3d p, q, v, bg, ba;
    getState(p, q, v, bg, ba);
    f.segment<3>(0) = v;
    f.segment<3>(3) = w_Body2Euler(q)*(gyro-bg-ng);
    f.segment<3>(6) = gravity + euler2mat(q)*(acc-ba-na);
    f.segment<3>(9) = nbg;
    f.segment<3>(12) = nba;

    return f;
}

VectorXd g_model()
{
    VectorXd g(VectorXd::Zero(measurementSize));

    g.segment<6>(0) = X_state.segment<6>(0);

    // if(g(3) > PI)  g(3) -= 2*PI;
    // if(g(3) < -PI) g(3) += 2*PI;
    // if(g(4) > PI)  g(4) -= 2*PI;
    // if(g(4) < -PI) g(4) += 2*PI;
    // if(g(5) > PI)  g(5) -= 2*PI;
    // if(g(5) < -PI) g(5) += 2*PI;

    return g;
}

//F_model G_model Jocobian
//diff_f()/diff_x (x_t-1  ut  noise=0)   At     Ft = I+dt*At
MatrixXd diff_f_diff_x(Vector3d q_last, Vector3d gyro, Vector3d acc, Vector3d bg_last, Vector3d ba_last)
{
    double cr = cos( q_last(0));
    double sr = sin( q_last(0));
    double cp = cos( q_last(1));
    double sp = sin( q_last(1));
    double cy = cos( q_last(2));
    double sy = sin( q_last(2));
 
    // ng na = 0 nbg nba = 0
    double Ax = acc(0) - ba_last(0);
    double Ay = acc(1) - ba_last(1);
    double Az = acc(2) - ba_last(2);
    // double Wx = gyro(0) - bg_last(0);
    double Wy = gyro(1) - bg_last(1);
    double Wz = gyro(2) - bg_last(2);

    MatrixXd diff_f_diff_x_jacobian(MatrixXd::Zero(stateSize, stateSize));
    MatrixXd diff_f_diff_x_jacobian_pqv(MatrixXd::Zero(stateSize_pqv, stateSize_pqv));

    diff_f_diff_x_jacobian_pqv <<  0, 0, 0, 0, 0, 0, 1, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 1, 0, 
      0, 0, 0, 0, 0, 0, 0, 0, 1,  
      0, 0, 0, (sp*(Wy*cr - Wz*sr))/cp, (Wz*cr + Wy*sr)/(cp*cp), 0, 0, 0, 0,  
      0, 0, 0, (- Wz*cr - Wy*sr), 0, 0, 0, 0, 0, 
      0, 0, 0, (Wy*cr - Wz*sr)/cp, (sp*(Wz*cr + Wy*sr))/(cp*cp), 0, 0, 0, 0, 
      0, 0, 0, (Ay*(sr*sy + cr*cy*sp) + Az*(cr*sy - cy*sp*sr)), (Az*cp*cr*cy - Ax*cy*sp + Ay*cp*cy*sr), (Az*(cy*sr - cr*sp*sy) - Ay*(cr*cy + sp*sr*sy) - Ax*cp*sy), 0, 0, 0, 
      0, 0, 0, (- Ay*(cy*sr - cr*sp*sy) - Az*(cr*cy + sp*sr*sy)), (Az*cp*cr*sy - Ax*sp*sy + Ay*cp*sr*sy), (Az*(sr*sy + cr*cy*sp) - Ay*(cr*sy - cy*sp*sr) + Ax*cp*cy), 0, 0, 0,
      0, 0, 0, (Ay*cp*cr - Az*cp*sr), (- Ax*cp - Az*cr*sp - Ay*sp*sr), 0, 0, 0, 0 ;
    
    diff_f_diff_x_jacobian.block<9, 9>(0, 0) = diff_f_diff_x_jacobian_pqv;
    diff_f_diff_x_jacobian.block<3, 3>(3, 9) = -w_Body2Euler(q_last);
    diff_f_diff_x_jacobian.block<3, 3>(6, 12) = -euler2mat(q_last);

    return diff_f_diff_x_jacobian;

    // cp != 0 pitch != 90° !!!!!!!!!
}
//diff_f()/diff_n (x_t-1  ut  noise=0)  Ut    Vt = dt*Ut
MatrixXd diff_f_diff_n(Vector3d q_last)
{
    MatrixXd diff_f_diff_n_jacobian(MatrixXd::Zero(stateSize, inputSize));
    diff_f_diff_n_jacobian.block<3,3>(3,0) = -w_Body2Euler(q_last);
    diff_f_diff_n_jacobian.block<3,3>(6,3) = -euler2mat(q_last);

    return diff_f_diff_n_jacobian;
}
//diff_g()/diff_x  (xt~ noise=0)  Ct 
MatrixXd diff_g_diff_x()
{
    MatrixXd diff_g_diff_x_jacobian(MatrixXd::Zero(measurementSize, stateSize));
    diff_g_diff_x_jacobian.block<3,3>(0,0) = MatrixXd::Identity(3,3);
    diff_g_diff_x_jacobian.block<3,3>(3,3) = MatrixXd::Identity(3,3);

    return diff_g_diff_x_jacobian;
}
//diff_g()/diff_v  (xt~ noise=0) Wt
MatrixXd diff_g_diff_v()
{
    MatrixXd diff_g_diff_v_jacobian(MatrixXd::Identity(measurementSize, measurementSize));

    return diff_g_diff_v_jacobian;
}
