#include "KalmanFilter.h"

KalmanFilterNode::KalmanFilterNode()
{
  //更新频率以及步长的初始化
  publishFreq=20;
  delta_t=1/publishFreq;

  //系统状态变量初始化标志位
  targetstate_is_init=false;
  //系统观测值获取成功标志位
  targetstate_is_meas=false;
  //是否赋值给X_evlt
  X_evlt_is_assign=false;
  //系统状态变量尺寸resize


  //【订阅】目标的状态
  target_state_sub=nh_.subscribe("/target_state",2,&KalmanFilterNode::targetstateCb,this);
  //【变量】
  target_found=false;

}
KalmanFilterNode::~KalmanFilterNode()
{

}

//考虑到简化任务考虑无人机只在一个方向运动
void KalmanFilterNode::targetstateCb(const geometry_msgs::Pose::ConstPtr msg)
{
  target_found=msg->orientation.w;
  if(!targetstate_is_init)
  {
    if (target_found)
    {
      target_state[0]=msg->position.x;
      target_state[1]=0;
      targetstate_is_init=true;
    }
  }
  else
  {
    if(target_found)
    {
      targetstate_is_meas=true;
      target_state_meas[0]=msg->position.y;
    }
  }
}

void KalmanFilterNode::Loop()
{
  ros::Rate pub_rate(publishFreq);

  //x_dot=A*x+B*u+w; 为系统状态方程
  Eigen::MatrixXd A(2,2);
  /*赋值方式1:
  A(0,0)=1;
  A(1,0)=0;
  A(0,1)=delta_t;
  A(1,1)=1;
  */
  //赋值方式2
  A<<    1   ,0,
      delta_t,1;


  Eigen::MatrixXd B(2,1);
  /*这里是匀加速运动的小车的B
  B(0,0)=pow(delta_t,2)/2;
  B(1,0)=delta_t;
  */
  //在针对匀速运动模型的时候，则应该为无输入
  B<< 0,0;

  //z=Hx+v系统的观测方程
  Eigen::MatrixXd H(2,1);
  //由于观测量只有位置
  H<<1,0;

  Eigen::MatrixXd Q(2,2); //过程激励噪声协方差
  //原代码上讲的是 假设系统的噪声向量只存在速度分量上，且速度噪声的方差是一个常量0.01,位移分量上的噪声协方差为0
  // !!!具体选取方式待定 (TBD)
  Q<<0,0,0,1;

  Eigen::MatrixXd R(1,1); //观测噪声协方差，测量值只有位移，它的协方差矩阵大小是1X1，就是测量噪声的方差本身
  // !!!具体选值待定 (TBD)
  R(0,0)=0.001;


  //【初始化】目标状态变量
  target_state.resize(2);

  //【变量定义】状态预测值
  Eigen::MatrixXd X_pdct=Eigen::MatrixXd::Constant(2,1,0);
  //【变量定义】状态估计值
  Eigen::MatrixXd X_evlt=Eigen::MatrixXd::Constant(2,1,0);
  //【变量定义】状态测量值
  Eigen::MatrixXd Z_meas=Eigen::MatrixXd::Constant(1,1,0);
  //【变量定义】估计状态和真实状态的协方差矩阵
  Eigen::MatrixXd Pk=Eigen::MatrixXd::Constant(2,2,0);
  //【变量定义】预测状态和真实状态的协方差矩阵
  Eigen::MatrixXd Pk_p=Eigen::MatrixXd::Constant(2,2,0);
  //【变量定义】增益
  Eigen::MatrixXd K=Eigen::MatrixXd::Constant(2,1,0);

  double acc=0.0;



  while (ros::ok())
  {
    ros::spinOnce();

    if (targetstate_is_init==true)
    {
      //判断是否赋值给X_evlt
      if(!X_evlt_is_assign)
      {
       // X_evlt[0]=target_state[0];
       // X_evlt[1]=target_state[1];
        X_evlt_is_assign=true;
      }

      if(targetstate_is_meas)
      {
        if(target_found)
        {
          //进行预测更新
          X_pdct=A*X_evlt+B*acc;
          //预测状态与真实状态的协方差矩阵，Pk'
          Pk_p=A*Pk*A.transpose()+Q;
          //K:2X1
          Eigen::MatrixXd tmp(1,1);
          tmp =H*Pk_p*H.transpose()+R;
          K=Pk_p*H.transpose()*tmp.inverse();
          //测量值z
          Z_meas=target_state_meas;
          //估计值
          X_evlt=X_pdct+K*(Z_meas-H*X_pdct);
          //估计状态和真实状态的协方差矩阵 ，Pk
          Pk=(Eigen::MatrixXd::Identity(2,2)-K*H)*Pk_p;

        }
        else
        {
          //只进行预测
           X_evlt=A*X_evlt+B*acc;

        }
      }

    }

    pub_rate.sleep();
  }

}
