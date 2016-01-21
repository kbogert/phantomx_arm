#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>

namespace gazebo
{
  class grasp_hack : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
   
      //printf("Hello World!\nHello World\nHello World!!!!!!!!!!!!!!!!!!!!");  

      // Store the pointer to the model
      this->model = _parent;
      this->flag = false;

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&grasp_hack::OnUpdate, this, _1));
    }

    // Called by the world update start event
    public: void OnUpdate(const common::UpdateInfo & /*_info*/)
    {
	 
      math::Pose _pose;
      if(this->model->GetWorld()->GetModelCount() == 8 && this->flag == false)
      {
        const std::string &_nameModel = "phantomx_arm";
    	this->phantomxModel = this->model->GetWorld()->GetModel(_nameModel);
        const std::string &_name = "cube_1";
        this->cube_1 = this->model->GetLink(_name); 
        const std::string &_name1 = "link_7";
        this->link_5 = this->phantomxModel->GetLink(_name1); 
	this->flag = true;
	
	_pose.Set(4.54, 4.0, 0.9,  0.0, 0.0, 0.0);
	this->cube_1->SetWorldPose(_pose);
	this->increment = 4.54;
      }


      //printf("Model Count: %d\n", this->model->GetWorld()->GetModelCount());

      //printf("Hello World!\nHello World\nHello World!!!!!!!!!!!!!!!!!!!!");   

      // Apply a small linear velocity to the model.
      //this->model->SetLinearVel(math::Vector3(-.03, 0, 0));
      // create the joint for the given model

    math::Vector3 axis;
    
    bool asdf = false;
    if(flag == true)
    {
	//this->increment = this->increment + 0.0001;
	_pose.Set(this->link_5->GetWorldPose().pos.x, this->link_5->GetWorldPose().pos.y, this->link_5->GetWorldPose().pos.z,  0.0, 0.0, 0.0);
	this->cube_1->SetWorldPose(_pose);

	//printf("For LINK_6 ---> X Pos: %f Y Pos: %f Z Pos: %f\n", this->link_5->GetWorldPose().pos.x, this->link_5->GetWorldPose().pos.y, this->link_5->GetWorldPose().pos.z); 
	//printf("For CUBE ---> X Pos: %f Y Pos: %f Z Pos: %f\n", this->cube_1->GetWorldPose().pos.x, this->cube_1->GetWorldPose().pos.y, this->cube_1->GetWorldPose().pos.z);     
    }
    //if(asdf == true)
	//printf("sdfa\n");
	

      
    }
  
    private: bool checkCollision()
    {
	physics::LinkPtr _Link;
	
	std::vector<physics::Contact*> _contact;
	
	//_Link = NULL;
	printf("Hello World!\nHello World\nHello World!!!!!!!!!!!!!!!!!!!! ---> %d\n", this->phantomxModel->GetWorld()->GetPhysicsEngine()->GetContactManager()->GetContactCount());

	if(this->phantomxModel->GetWorld()->GetPhysicsEngine()->GetContactManager()->GetContactCount() > 0)
	{
		printf("Hello World!\nHello World\nHello World!!!!!!!!!!!!!!!!!!!!");  
	
		for(int i=0; i<this->phantomxModel->GetWorld()->GetPhysicsEngine()->GetContactManager()->GetContactCount(); i++)
		{
			if(this->phantomxModel->GetWorld()->GetPhysicsEngine()->GetContactManager()->GetContact(i)->collision1->GetLink() == this->cube_1)
			{
				_Link = this->model->GetWorld()->GetPhysicsEngine()->GetContactManager()->GetContact(i)->collision1->GetLink();				
				printf("Hello World!\nHello World\nHello World!!!!!!!!!!!!!!!!!!!! --------> %d \n", i);  
	
			}
		}		
		
	}
	
	if(_Link == this->cube_1)
	{
		return true;
	}
	else 
	{
		return false;
	}
    }

    private: bool flag;

    private: double increment;

    // Pointer to the Cube model
    private: physics::ModelPtr model;

    // Pointer to the Phantomx model
    private: physics::ModelPtr phantomxModel;

    // Pointer to the New model
    private: physics::JointPtr myJoint;

    // Pointer to the Phantomx link
    private: physics::LinkPtr link_5;

    // Pointer to the Cube model
    private: physics::LinkPtr cube_1;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(grasp_hack)
}
