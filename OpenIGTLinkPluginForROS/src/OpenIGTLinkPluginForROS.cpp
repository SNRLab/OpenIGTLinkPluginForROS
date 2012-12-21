/* 
 * Copyright 2011 Nate Koenig & Andrew Howard
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#include <boost/bind.hpp>
#include "physics/physics.h"
#include "transport/Node.hh"
#include "transport/TransportTypes.hh"
#include "msgs/MessageTypes.hh"
#include "common/Time.hh"
#include "common/Plugin.hh"
#include "common/Events.hh"
#include "common/PID.hh"

// for own thread
#include "ros/callback_queue.h"
#include <boost/thread.hpp>
#include "boost/thread/mutex.hpp"

// for OpenIGTLink Receiver
#include <iostream>
#include <math.h>
#include <cstdlib>
#include <cstring>

#include "igtlOSUtil.h"
#include "igtlMessageHeader.h"
#include "igtlTransformMessage.h"
#include "igtlImageMessage.h"
#include "igtlServerSocket.h"
#include "igtlStatusMessage.h"
#include "igtlPositionMessage.h"

// comments

namespace gazebo
{
  class MoveModel : public ModelPlugin
  {
    public: void Load( physics::ModelPtr _parent, sdf::ElementPtr _sdf )
    {
      // Get the world name.
      this->world = _parent->GetWorld();

      // Get a pointer to the model
      this->model = _parent;

      this->node = transport::NodePtr(new transport::Node());
      this->node->Init(this->world->GetName());
      this->statsSub = this->node->Subscribe("~/world_stats", &MoveModel::OnStats, this);
      
      // initialize a PID class
      for(int i=0; i<40; i++)
      {
      this->target_position_[i] = 0.0;
      this->pid[i].Init(10, 0, 2, 0, 0, 100, -100);
      this->pid[i].SetCmd(this->target_position_[i]);
      this->pid1[i].Init(10, 0, 2, 0, 0, 100, -100);
      this->pid1[i].SetCmd(this->target_position_[i]);
      }

      this->last_update_time_ = this->model->GetWorld()->GetSimTime();

      // Load parameters for this plugin
      if (this->LoadParams(_sdf))
      {
        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        this->updateConnection = event::Events::ConnectWorldUpdateStart(
            boost::bind(&MoveModel::OnUpdate, this));
      }

     // start custom queue for joint trajectory plugin ros topics
     this->callback_queue_thread_ = boost::thread( boost::bind( &MoveModel::OpenIGTLinkReceiverThread,this ) );

    }

    public: bool LoadParams(sdf::ElementPtr _sdf) 
    {
      if (
          this->FindJointByParam(_sdf, this->controlled_joint_[0],
                             "joint1") &&
          this->FindJointByParam(_sdf, this->controlled_joint_[1],
                             "joint2") &&
          this->FindJointByParam(_sdf, this->controlled_joint_[2],
                             "joint3") &&
          this->FindJointByParam(_sdf, this->controlled_joint_[3],
                             "joint4") &&
          this->FindJointByParam(_sdf, this->controlled_joint_[4],
                             "joint5") && 
          this->FindJointByParam(_sdf, this->controlled_joint_[5],
                             "joint6") &&
          this->FindJointByParam(_sdf, this->controlled_joint_[6],
                             "joint7") &&
          this->FindJointByParam(_sdf, this->controlled_joint_[7],
                             "joint8") &&
          this->FindJointByParam(_sdf, this->controlled_joint_[8],
                             "joint9") &&
          this->FindJointByParam(_sdf, this->controlled_joint_[9],
                             "joint10") &&
          this->FindJointByParam(_sdf, this->controlled_joint_[10],
                             "joint11") &&
          this->FindJointByParam(_sdf, this->controlled_joint_[11],
                             "joint12") &&
          this->FindJointByParam(_sdf, this->controlled_joint_[12],
                             "joint13") &&
          this->FindJointByParam(_sdf, this->controlled_joint_[13],
                             "joint14") &&
          this->FindJointByParam(_sdf, this->controlled_joint_[14],
                             "joint15") &&
          this->FindJointByParam(_sdf, this->controlled_joint_[15],
                             "joint16") &&
          this->FindJointByParam(_sdf, this->controlled_joint_[16],
                             "joint17") &&
          this->FindJointByParam(_sdf, this->controlled_joint_[17],
                             "joint18") &&
          this->FindJointByParam(_sdf, this->controlled_joint_[18],
                             "joint19") &&
          this->FindJointByParam(_sdf, this->controlled_joint_[19],
                             "joint20") &&
          this->FindJointByParam(_sdf, this->controlled_joint_[20],
                             "joint21") &&
          this->FindJointByParam(_sdf, this->controlled_joint_[21],
                             "joint22") &&
          this->FindJointByParam(_sdf, this->controlled_joint_[22],
                             "joint23") &&
          this->FindJointByParam(_sdf, this->controlled_joint_[23],
                             "joint24") &&
          this->FindJointByParam(_sdf, this->controlled_joint_[24],
                             "joint25") &&
          this->FindJointByParam(_sdf, this->controlled_joint_[25],
                             "joint26") &&
          this->FindJointByParam(_sdf, this->controlled_joint_[26],
                             "joint27") &&
          this->FindJointByParam(_sdf, this->controlled_joint_[27],
                             "joint28") &&
          this->FindJointByParam(_sdf, this->controlled_joint_[28],
                             "joint29") &&
          this->FindJointByParam(_sdf, this->controlled_joint_[29],
                             "joint30") &&
          this->FindJointByParam(_sdf, this->controlled_joint_[30],
                             "joint31") &&
          this->FindJointByParam(_sdf, this->controlled_joint_[31],
                             "joint32") &&
          this->FindJointByParam(_sdf, this->controlled_joint_[32],
                             "joint33") &&
          this->FindJointByParam(_sdf, this->controlled_joint_[33],
                             "joint34") &&
          this->FindJointByParam(_sdf, this->controlled_joint_[34],
                             "joint35") &&
          this->FindJointByParam(_sdf, this->controlled_joint_[35],
                             "joint36") &&
          this->FindJointByParam(_sdf, this->controlled_joint_[36],
                             "joint37") &&
          this->FindJointByParam(_sdf, this->controlled_joint_[37],
                             "joint38") &&
          this->FindJointByParam(_sdf, this->controlled_joint_[38],
                             "joint39") &&
          this->FindJointByParam(_sdf, this->controlled_joint_[39],
                             "joint40") 
    )

        return true;
      else
        return false;
    }

    public: bool FindJointByParam(sdf::ElementPtr _sdf,
                                  physics::JointPtr &_joint,
                                  std::string _param)
    {
      if (!_sdf->HasElement(_param))
      {
        gzerr << "param [" << _param << "] not found\n";
        return false;
      }
      else
      {
        _joint = this->model->GetJoint(
          _sdf->GetElement(_param)->GetValueString());

        if (!_joint)
        {
          gzerr << "joint by name ["
                << _sdf->GetElement(_param)->GetValueString()
                << "] not found in model\n";
          return false;
        }
      }
      return true;
    }

    // Called by the world update start event
    public: void OnUpdate()
    {
      // mutex lock
      boost::mutex::scoped_lock lock(this->update_mutex);

      // PID controller
      this->target_position_[0] = 3.14*this->matrix[0][3]/180.0;
      this->target_position_[1] = 3.14*this->matrix[1][3]/180.0;

      common::Time current_time = this->model->GetWorld()->GetSimTime();
      double dt    = current_time.Double()
                   - this->last_update_time_.Double();
      
      for(int i = 0; i < 20; i++)
      {
      this->pid[i].SetCmd(this->target_position_[0]);
      this->pid1[i].SetCmd(this->target_position_[1]);

      this->error[i] = this->controlled_joint_[i]->GetAngle(0).GetAsRadian()
                   - target_position_[0];
      this->pid[i].Update(this->error[i], dt);

      this->error[i] = this->controlled_joint_[i]->GetAngle(0).GetAsRadian()
                   - target_position_[1];
      this->pid1[i].Update(this->error[i], dt);
      this->controlled_joint_[i]->SetForce(0, this->pid[i].GetCmd()+this->pid1[i].GetCmd());
      }
      for(int i = 20; i < 40; i++)
      {
      this->pid[i].SetCmd(this->target_position_[1]);
      this->error[i] = this->controlled_joint_[i]->GetAngle(0).GetAsRadian()
                   - target_position_[1];
      this->pid[i].Update(this->error[i], dt);
      this->controlled_joint_[i]->SetForce(0, this->pid[i].GetCmd());
      }
      
      this->last_update_time_ = current_time;
      //gzdbg << "error [" << error
      //      << "] cmd [" << this->pid.GetCmd() << "]\n";
      //gzdbg << "error1 [" << error1
      //      << "] cmd1 [" << this->pid1.GetCmd() << "]\n";
      
      //std::cerr << "matrix[0][3]=" << this->matrix[0][3] << std::endl;
      //std::cerr << "matrix[1][3]=" << this->matrix[1][3] << std::endl;
      //std::cerr << "matrix[2][3]=" << this->matrix[2][3] << std::endl;
      //std::cerr << "matrix[3][3]=" << this->matrix[3][3] << std::endl;

    }

	public: int ReceiveTransform(igtl::Socket * socket, igtl::MessageHeader * header)
	{
	  std::cerr << "Receiving TRANSFORM data type." << std::endl;
  
	  // Create a message buffer to receive transform data
	  //igtl::TransformMessage::Pointer transMsg;
	  //transMsg = igtl::TransformMessage::New();
	  this->transMsg->SetMessageHeader(header);
	  this->transMsg->AllocatePack();
  
	  // Receive transform data from the socket
	  socket->Receive(this->transMsg->GetPackBodyPointer(), this->transMsg->GetPackBodySize());
  
	  // Deserialize the transform data
	  // If you want to skip CRC check, call Unpack() without argument.
	  int c = this->transMsg->Unpack(1);
  
	  if (c & igtl::MessageHeader::UNPACK_BODY) // if CRC check is OK
	    {
            // mutex lock
	    boost::mutex::scoped_lock lock(this->update_mutex);

	    // Retrive the transform data
	    //igtl::Matrix4x4 matrix;
	    this->transMsg->GetMatrix(this->matrix);
	    //igtl::PrintMatrix(this->matrix);
	    
	    this->controlFlag=1;

	    return 1;
	    }

	  return 0;

	}



    public: void OnStats( const boost::shared_ptr<msgs::WorldStatistics const> &_msg)
    {
      this->simTime  = msgs::Convert( _msg->sim_time() );

    }

    // own thread
    private: void OpenIGTLinkReceiverThread()
    {
     static const double timeout = 0.01;

      // OpenIGTLink
      int    port     = 18945;
      double fps      = 10;
      this->interval = (int) (1000.0 / fps);

      this->communicationFlag = 0;
      this->testCount = 0;
      this->ccount = 1;

      // for receiving message from Slicer
      this->transMsg = igtl::TransformMessage::New();
      this->transMsg->SetDeviceName("MarkersPositionTransform");

      // for sending message to Slicer
      this->positionMsg = igtl::PositionMessage::New();
      this->positionMsg->SetDeviceName("MRIRobotTransform");
      this->positionMsg->SetPackType(igtl::PositionMessage::ALL); // default
     
      this->serverSocket = igtl::ServerSocket::New();
      this->r = serverSocket->CreateServer(port);

      if (this->r < 0)
      {
	std::cerr << "Cannot create a server socket." << std::endl;
        exit(0);
      }else{
        std::cerr << "Could create a server socket." << std::endl;
      }


	while(1){

	  this->socket = this->serverSocket->WaitForConnection(1);

          if (this->socket.IsNotNull()) // if client connected
          {

            // receive states of the robot from the model
            //float angle = this->controlled_joint_[0]->GetAngle(0).GetAsRadian();

	    // Create a message buffer to receive header
            igtl::MessageHeader::Pointer headerMsg;
            headerMsg = igtl::MessageHeader::New();

	    // Initialize receive buffer
            headerMsg->InitPack();

            this->r = this->socket->Receive(headerMsg->GetPackPointer(), headerMsg->GetPackSize());
      
            std::cerr << "matrix[0][3]=" << this->matrix[0][3] << std::endl;
            std::cerr << "matrix[1][3]=" << this->matrix[1][3] << std::endl;
            std::cerr << "matrix[2][3]=" << this->matrix[2][3] << std::endl;
            std::cerr << "matrix[3][3]=" << this->matrix[3][3] << std::endl;

	    // Deserialize the header
            headerMsg->Unpack();
	    this->ReceiveTransform(this->socket, headerMsg);

	    // Send message to Slicer
            float position[3];
            float quaternion[4];
        
            //GetRandomTestVectors(position, quaternion);
	    position[0]=this->matrix[0][3];
	    position[1]=this->matrix[1][3];
	    position[2]=this->matrix[2][3];
	
	    quaternion[0]=1.0;
	    quaternion[1]=1.0;
	    quaternion[2]=1.0;
	    quaternion[3]=1.0;
             
	    this->positionMsg->SetPosition(position);
            this->positionMsg->SetQuaternion(quaternion);
            this->positionMsg->Pack();
            this->socket->Send(this->positionMsg->GetPackPointer(), this->positionMsg->GetPackSize());
            igtl::Sleep(this->interval); // wait

	    if (this->r == 0)
            {
          	this->socket->CloseSocket();
            }

	  }
	} // while loop

    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    // subscribe to world stats
    private: transport::NodePtr node;
    private: transport::SubscriberPtr statsSub;
    private: common::Time simTime;
    private: physics::WorldPtr world;

    private: physics::JointPtr controlled_joint_[40];

    // for own thread
    private: boost::mutex update_mutex;
    private: ros::CallbackQueue queue_;
    private: boost::thread callback_queue_thread_;

    // for OpenIGTLink
    int interval;
    int r;
    igtl::ServerSocket::Pointer serverSocket;
    igtl::Socket::Pointer socket;
    //igtl::PositionMessage::Pointer positionMsg;
    igtl::TransformMessage::Pointer transMsg;
    igtl::PositionMessage::Pointer positionMsg;

    // communication matrix between OpenIGTLink receiver and creating force vectors
    igtl::Matrix4x4 matrix;

    int communicationFlag;
    int testCount;

    // PID control
    int ccount;
    int controlFlag;

    common::PID pid[40];
    common::PID pid1[40];

    double target_position_[40];
    double error[40];

    physics::JointPtr joint_;
    event::ConnectionPtr update_connection_;
    common::Time last_update_time_;

  // Constructer
  public: MoveModel(){}

  // Destructer
  public: virtual ~MoveModel()
  {
  this->queue_.clear();
  this->queue_.disable();
  this->callback_queue_thread_.join();

  }


  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(MoveModel)
}
