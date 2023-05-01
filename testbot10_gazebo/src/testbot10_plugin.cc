#include "testbot10_plugin.hpp"
#include "Walkingpattern_generator.hpp"
#define deg2rad		0.017453292519943
#define rad2deg		57.295779513082323
namespace gazebo
{
void testbot10::Load(ModelPtr _model, sdf::ElementPtr)
  {
    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "testbot10");
    ROS_INFO("JAEMIN");
    IdleMotion();
    sub = n.subscribe("turn_angle", 1, &testbot10::PositionCallback, this);
    sub_motion_selector = n.subscribe("motion_selector", 1, &testbot10::SelectMotion, this);
    this->model = _model;
    jc0 = new physics::JointController(model);
    GetLinks();
    GetJoints();
    IdleMotion();
    InitROSPubSetting();
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
    boost::bind(&testbot10::OnUpdate, this, _1));
    time = 0;
    indext = 0;
    angle = 0;

  }

void testbot10::InitROSPubSetting()
{
    pub_joint_state = n.advertise<sensor_msgs::JointState>("testbot10/joint_states", 100);
}

void testbot10::ROSMsgPublish()
{
 sensor_msgs::JointState joint_state_msg;

 joint_state_msg.header.stamp = ros::Time::now();
 for (uint8_t i = 0; i < 12; i++)
 {
   joint_state_msg.name.push_back((std::string)joint_names.at(i));
   joint_state_msg.position.push_back((float)(sensor_th[i]));
 }
 pub_joint_state.publish(joint_state_msg);
}

void testbot10::GetLinks()
  {
    this->base_link  = this->model->GetLink("base_link");
    this->RL_link1 = this->model->GetLink("RL_link1");
    this->RL_link2 = this->model->GetLink("RL_link2");
    this->RL_link3 = this->model->GetLink("RL_link3");
    this->RL_link4 = this->model->GetLink("RL_link4");
    this->RL_link5 = this->model->GetLink("RL_link5");
    this->RL_link6 = this->model->GetLink("RL_link6");
    this->LL_link1 = this->model->GetLink("LL_link1");
    this->LL_link2 = this->model->GetLink("LL_link2");
    this->LL_link3 = this->model->GetLink("LL_link3");
    this->LL_link4 = this->model->GetLink("LL_link4");
    this->LL_link5 = this->model->GetLink("LL_link5");
    this->LL_link6 = this->model->GetLink("LL_link6");
  }

void testbot10::GetJoints()
  {
    this->RL_j1 = this->model->GetJoint("RL_j1");
    this->RL_j2 = this->model->GetJoint("RL_j2");
    this->RL_j3 = this->model->GetJoint("RL_j3");
    this->RL_j4 = this->model->GetJoint("RL_j4");
    this->RL_j5 = this->model->GetJoint("RL_j5");
    this->RL_j6 = this->model->GetJoint("RL_j6");

    this->LL_j1 = this->model->GetJoint("LL_j1");
    this->LL_j2 = this->model->GetJoint("LL_j2");
    this->LL_j3 = this->model->GetJoint("LL_j3");
    this->LL_j4 = this->model->GetJoint("LL_j4");
    this->LL_j5 = this->model->GetJoint("LL_j5");
    this->LL_j6 = this->model->GetJoint("LL_j6");
  }

void testbot10::OnUpdate(const common::UpdateInfo &)
  { 
    PostureGeneration();
    SetJointPosition();
  }

void testbot10::SelectMotion(const std_msgs::Float32Ptr &msg){
    mode = msg ->data;
    ROS_INFO("mode(%f)",mode);
    Motions motion;
    if (indext == 0){
    if (mode == 0 ){
    motion.Motion0();
    ref_LL_th = motion.Return_Motion0_LL();
    ref_RL_th = motion.Return_Motion0_RL();
    }
    else if (mode == 1){
    motion.Motion1();
    ref_LL_th = motion.Return_Motion1_LL();
    ref_RL_th = motion.Return_Motion1_RL();
    }
    else if (mode == 2){
    motion.Motion2();
    ref_LL_th = motion.Return_Motion2_LL();
    ref_RL_th = motion.Return_Motion2_RL();
    }
    else if (mode == 3){
    motion.Motion3();
    ref_LL_th = motion.Return_Motion3_LL();
    ref_RL_th = motion.Return_Motion3_RL();
    }
    else {
    motion.Motion0();
    ref_LL_th = motion.Return_Motion0_LL();
    ref_RL_th = motion.Return_Motion0_RL();
    }
    }
}

void testbot10::SetJointPosition()
  {
    RL_j1->SetPosition(0, RL_th(0));
    RL_j2->SetPosition(0, RL_th(1));
    RL_j3->SetPosition(0, RL_th(2));
    RL_j4->SetPosition(0, RL_th(3));
    RL_j5->SetPosition(0, RL_th(4));
    RL_j6->SetPosition(0, RL_th(5));
    LL_j1->SetPosition(0, LL_th(0));
    LL_j2->SetPosition(0, LL_th(1));
    LL_j3->SetPosition(0, LL_th(2));
    LL_j4->SetPosition(0, LL_th(3));
    LL_j5->SetPosition(0, LL_th(4));
    LL_j6->SetPosition(0, LL_th(5));
  }

void testbot10::GetJointPosition()
  {
    sensor_th[0] = this->RL_j1->Position(2);
    sensor_th[1] = this->RL_j2->Position(1);
    sensor_th[2] = this->RL_j3->Position(1);
    sensor_th[3] = this->RL_j4->Position(1);
    sensor_th[4] = this->RL_j5->Position(1);
    sensor_th[5] = this->RL_j6->Position(0);
    sensor_th[6] = this->LL_j1->Position(2);
    sensor_th[7] = this->LL_j2->Position(1);
    sensor_th[8] = this->LL_j3->Position(1);
    sensor_th[9] = this->LL_j4->Position(1);
    sensor_th[10] = this->LL_j5->Position(1);
    sensor_th[11] = this->LL_j6->Position(0);
 }
 
void testbot10::PositionCallback(const std_msgs::Float32Ptr &msg)
  {
  angle = msg->data;
  ROS_INFO("angle callback(%f)",angle);
  TurningTrajectory();
  };

void testbot10::PostureGeneration()
 {
    time +=1;
    if (time>=30){
    RL_th(0) = ref_RL_th(indext, 0);
    RL_th(1) = ref_RL_th(indext, 1);
    RL_th(2) = ref_RL_th(indext, 2);
    RL_th(3) = ref_RL_th(indext, 3);
    RL_th(4) = ref_RL_th(indext, 4);
    RL_th(5) = ref_RL_th(indext, 5);
    LL_th(0) = ref_LL_th(indext, 0);
    LL_th(1) = -ref_LL_th(indext, 1);
    LL_th(2) = -ref_LL_th(indext, 2);
    LL_th(3) = -ref_LL_th(indext, 3);
    LL_th(4) = -ref_LL_th(indext, 4);
    LL_th(5) = ref_LL_th(indext, 5);
    indext += 1;
    time = 0;
    }
    if (indext >=923)
    indext = 0;
 }  
  
 void testbot10::IdleMotion()
  { 
    Motions motion;
    motion.Motion0();
    ref_LL_th = motion.Return_Motion0_LL();
    ref_RL_th = motion.Return_Motion0_RL();
    // ref_RL_th = 0;
    // ref_RL_th.col(1) = 0;
    // ref_RL_th.col(2) = VectorXd::Zero(924);// -15. * deg2rad * VectorXd::Ones(924);
    // ref_RL_th.col(3) = VectorXd::Zero(924);//30. * deg2rad * VectorXd::Ones(924);
    // ref_RL_th.col(4) = VectorXd::Zero(924);//-15. * deg2rad * VectorXd::Ones(924);
    // ref_RL_th.col(5) = VectorXd::Zero(924);
    // ref_LL_th.col(0) = VectorXd::Zero(924);
    // ref_LL_th.col(1) = VectorXd::Zero(924);
    // ref_LL_th.col(2) = VectorXd::Zero(924);//-15. * deg2rad * VectorXd::Ones(924);
    // ref_LL_th.col(3) = VectorXd::Zero(924);//30. * deg2rad * VectorXd::Ones(924);
    // ref_LL_th.col(4) = VectorXd::Zero(924);//-15. * deg2rad * VectorXd::Ones(924);
    // ref_LL_th.col(5) = VectorXd::Zero(924);
 } 

void testbot10::TurningTrajectory(){
  if (angle < 0){
    if (indext < 296)
    {
      for (int i = 0; i < 55; i++){
      ref_LL_th(296 + i,0) = angle/55*i;
      ref_LL_th(390 + i, 0) = angle - angle/55*i;
      }
      for (int k = 0; k < 39; k++)
      {
      ref_LL_th(351+k,0) = angle;
      }
    }
    else if (indext < 481)
    {
      for (int j = 0; j < 55; j++){ 
      ref_LL_th(481 + j, 0) = angle/55*j;
      ref_LL_th(575 + j, 0) = angle - angle/55*(j+1);    
      }
      for (int l = 0; l< 39; l++)
      {
      ref_LL_th(536 + l , 0) = angle;
      }
    }

  };

if (angle >0){
   if (indext < 204)
    {
      for (int i = 0; i < 55; i++){ 
      ref_RL_th(204 + i,0) = angle/55*i;
      ref_RL_th(298 + i, 0) = angle - angle/55*i;
      }
      for (int k = 0; k < 39; k++)
      {
      ref_RL_th(259+k,0) = angle;
      }
    }
    else if (indext < 389)
    {
      for (int j = 0; j < 55; j++){ 
      ref_RL_th(389 + j, 0) = angle/55*j;
      ref_RL_th(483 + j, 0) = angle -angle/55*j;    
      }
      for (int l = 0; l< 39; l++)
      {
      ref_RL_th(444 + l , 0) = angle;
      }
    }
    else if (indext < 573)
    {
      for (int m = 0; m < 55; m++){ 
      ref_RL_th(573 + m, 0) = angle/55*m;
      ref_RL_th( 667+ m, 0) = angle -angle/55*(m+1);    
      }
      for (int n = 0; n< 39; n++)
      {
      ref_RL_th(628 + n , 0) = angle;
      }

    }

};
};
  double testbot10::Turn(double t){
  double X = turn(0) + turn(1) * t + turn(2) * pow(t, 2) + turn(3) * pow(t, 3) + turn(4) * pow(t, 4) + turn(5) * pow(t, 5);
	return X;
  }
  double testbot10::Back(double t){
  double X = back(0) + back(1) * t + back(2) * pow(t, 2) + back(3) * pow(t, 3) + back(4) * pow(t, 4) + back(5) * pow(t, 5);
	return X;
  }
}

