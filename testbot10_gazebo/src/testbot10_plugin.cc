#include "testbot10_plugin.hpp"
#include "Walkingpattern_generator.hpp"
namespace gazebo
{
  void testbot10::Load(ModelPtr _model, sdf::ElementPtr)
  {
    int argc = 0;
    char **argv = NULL;
    ros::init(argc, argv, "testbot10");
    ROS_INFO("JAEMIN");
    //sub = n.subscribe("motion_select", 0, &testbot10::PositionCallback, this);
    this->model = _model;
    jc0 = new physics::JointController(model);
    GetLinks();
    GetJoints();
    SelectMotion();
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&testbot10::OnUpdate, this, _1));
    time = 0;
    indext = 0;  
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
    SetJointPosition();
  }

  void testbot10::SelectMotion(){
    Motions motion;
    motion.Motion2();
    ref_LL_th = motion.Return_Motion2_LL();
    // std::cout << "ref_LL_th" << ref_LL_th << std::endl;
    ref_RL_th = motion.Return_Motion2_RL();
    std::cout << "ref_RL_th" << ref_RL_th << std::endl;
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
}