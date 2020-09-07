#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <sstream>
#include <thread>
#include <tf/transform_broadcaster.h>

#include "nuturtlebot/WheelCommands.h"
#include "nuturtlebot/SensorData.h"
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
#include "std_msgs/String.h"

namespace gazebo {
	
	// See the link below for how to integrate ROS with a plugin
	// http://gazebosim.org/tutorials?tut=guided_i6&cat=
	class ModelPush : public ModelPlugin {
		public: 
			void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {
			
				std::cout << "\n\nPlug-in load is running\n\n" << std::endl;		
					
				// Store the pointer to the model
				this->model = _parent;
			
				if (!ros::isInitialized()) {
					ROS_FATAL("A ROS node for Gazebo has not been initialized."
							"Unable to load plugin. Load the Gazebo system plugin"
							"'libgazebo_ros_api_plugin.so' in the gazebo_ros package");
					return;
				}
				

				// Listen to the update event. This event is broadcast every
				// simulation iteration.
				this->updateConnection = event::Events::ConnectWorldUpdateBegin(
						std::bind(&ModelPush::OnUpdate, this));
		
				
				n.reset(new ros::NodeHandle("gazebo_client"));
				std::cout << "\n\n\n\nPlugin is setting up its publishers and subscribers\n\n\n\n" << std::endl;
					
	
					
				sensor_data_pub = this->n->advertise<nuturtlebot::SensorData>("/sensor_data", 1);  	
				wheel_cmd_sub = this->n->subscribe("/wheel_cmd", 1, &ModelPush::wheel_cmd_callback, this);	
			
				n->getParam("/left_wheel_joint", left_wheel_joint);
       				n->getParam("/right_wheel_joint", right_wheel_joint);			
				n->getParam("/wheel_base", wheel_base);
		        	n->getParam("/wheel_radius", wheel_radius);
        			n->getParam("/motor_torque", max_motor_torque);
        			n->getParam("/motor_power", max_cmd);
        			n->getParam("/encoder_ticks_per_revolution", encoder_ticks_rotation);
				n->getParam("/motor_limit", max_motor_rot);	
				
				frequency = _sdf->GetElement("sensor_frequency")->Get<double>();

				// Set the intial wheel velocity to be 0
				left_wheel_vel = 0.0;
				right_wheel_vel = 0.0;
				
				initial_encoder_left = 0; 
				initial_encoder_right = 0;
			
				// Make wheels be at 0 rads/s
			        this->model->GetJoint(left_wheel_joint)->SetParam("fmax", 0 , max_motor_torque);
      				this->model->GetJoint(left_wheel_joint)->SetParam("vel", 0 , 0.0);
   				this->model->GetJoint(right_wheel_joint)->SetParam("fmax", 0 , max_motor_torque);
     				this->model->GetJoint(right_wheel_joint)->SetParam("vel", 0 , 0.0);
				
				last_time = model->GetWorld()->SimTime();	
			}

		// Called by the world update start event
		public: void OnUpdate() {
 				// publish wheel odometry and control the velocity
				
				double period = 1.0/frequency;
                                
				gazebo::common::Time current_time = model->GetWorld()->SimTime();
                                double delta_t = (current_time - last_time).Double();

				if (delta_t >= period) {
					last_time = current_time;
					 				
					// We know we rotate at current_wheel_cmd for delta_t seconds
					// Calculate the new/nearest encoder value
					nuturtlebot::SensorData newData;
					
					// Convert from motor command to					
					double m = (encoder_ticks_rotation) / (2.0  * 3.14);
    					double b = (encoder_ticks_rotation - 2.0 * 3.14 * m);
    					newData.left_encoder = this->model->GetJoint(left_wheel_joint)->Position() * m + b;
   					newData.right_encoder = this->model->GetJoint(right_wheel_joint)->Position() * m + b;

					sensor_data_pub.publish(newData);		

					if (updateVelocity) {	

						updateVelocity = false;	
						// the vel is set in rad/s - fmax is the max force/torque possible at this motor
						this->model->GetJoint(left_wheel_joint)->SetParam("vel", 0, left_wheel_vel);
						this->model->GetJoint(left_wheel_joint)->SetParam("fmax", 0, max_motor_torque);

                                		this->model->GetJoint(right_wheel_joint)->SetParam("vel", 0, right_wheel_vel);
						this->model->GetJoint(right_wheel_joint)->SetParam("fmax", 0, max_motor_torque);
					}
				}

			}
		

		public: void wheel_cmd_callback(nuturtlebot::WheelCommands next_wheel_command) {
			
			// Convert a wheel command (-256, 256) to a velocity (rad/s)
			//double m = (max_motor_rot * 2.0) / (max_cmd * 2.0);
  			//double b = (max_motor_rot - max_cmd * m);
			float m = (max_cmd * 2.0) / (max_motor_rot * 2.0);
		        float b = (max_cmd - max_motor_rot * m);

  			left_wheel_vel = ((double)(next_wheel_command.left_velocity) - b) / m;
  			right_wheel_vel = ((double)(next_wheel_command.right_velocity) - b) / m;
			

			if (left_wheel_vel > max_motor_rot) {
                		left_wheel_vel = max_motor_rot;
		        }
		        else if (left_wheel_vel < (-1 * max_motor_rot) ) {
                		left_wheel_vel = -1 * max_motor_rot;
        		}

        		if (right_wheel_vel > max_motor_rot) {
                		right_wheel_vel = max_motor_rot;
        		}
        		else if (right_wheel_vel < (-1 * max_motor_rot) ) {
               			right_wheel_vel = -1 * max_motor_rot;
        		}

			updateVelocity = true;

			//std::cout << "(" << left_wheel_vel << ", " << right_wheel_vel << ")" << std::endl;		
		}

		// Pointer to the model
		private: physics::ModelPtr model;

		// Pointer to the update event connection
		private: event::ConnectionPtr updateConnection;
	
		private: ros::Publisher sensor_data_pub;
		private: ros::Subscriber wheel_cmd_sub;
		private: std::unique_ptr<ros::NodeHandle> n;

		private: std::string left_wheel_joint;
		private: std::string right_wheel_joint;
		private: double max_motor_torque, max_cmd, max_motor_rot, encoder_ticks_rotation;
		private: double wheel_base, wheel_radius, frequency;
		

		private: double left_wheel_vel, right_wheel_vel;
		private: gazebo::common::Time last_time;
		private: int initial_encoder_left, initial_encoder_right;
		private: bool updateVelocity;

	};


	// Register this plugin with the simulator
	GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}
