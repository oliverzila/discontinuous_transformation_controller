/******************************************************************************
	            	ROS Discontinuous Transformation Controller Package
		        			Discontinuous Transformation Controller
          Copyright (C) 2019 Carlos Eduardo Pedroso de Oliveira and
		  					Ernesto Dickel Saraiva

        This program is free software: you can redistribute it and/or modify
        it under the terms of the GNU General Public License as published by
        the Free Software Foundation, either version 3 of the License, or
        (at your option) any later version.

        This program is distributed in the hope that it will be useful, but
        WITHOUT ANY WARRANTY; without even the implied warranty of
        MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
        General Public License for more details.

        You should have received a copy of the GNU General Public License
        along with this program.  If not, see
        <http://www.gnu.org/licenses/>.
        
*******************************************************************************/

#include <ros/ros.h>

#include <geometry_msgs/Accel.h>
#include <geometry_msgs/PoseStamped.h>
#include <discontinuous_transformation_controller_msgs/DiscontinuousTransformationControllerStatus.h>
#include <nav_msgs/Odometry.h>
#include <gazebo_msgs/ModelStates.h>
#include <linearizing_controllers_msgs/DynamicsLinearizingControllerStatus.h>

#include <eigen3/Eigen/Dense>
#include <tf/transform_datatypes.h>
#include <control_toolbox/pid.h>
#include <math.h>
#include <iostream>

Eigen::Vector2d u; //linear and angular velocity
Eigen::Vector3d x; //pose - x,y,theta
Eigen::Vector3d xr; //pose reference

Eigen::Vector3d x_real;
Eigen::Vector2d u_real;
Eigen::Vector2d torque;

void realPoseCB(const gazebo_msgs::ModelStates::ConstPtr &pose_real)
{       //change names to use on plot later
        x_real[0]=pose_real->pose[1].position.x;
        x_real[1]=pose_real->pose[1].position.y;
        x_real[2]=tf::getYaw(pose_real->pose[1].orientation);
        u_real[0]=sqrt(pow(pose_real->twist[1].linear.x,2)+pow(pose_real->twist[1].linear.y,2));
        u_real[1]=pose_real->twist[1].angular.z;
}


void statusCB(const linearizing_controllers_msgs::DynamicsLinearizingControllerStatus::ConstPtr &status_)
{
    u[0]=status_->process_value.linear.x;
    u[1]=status_->process_value.angular.z;
    torque[0]=status_->command[0];
    torque[1]=status_->command[1];
}

void poseCB(const nav_msgs::Odometry::ConstPtr &pose_)
{
    x[0]=pose_->pose.pose.position.x;
    x[1]=pose_->pose.pose.position.y;
    x[2]=tf::getYaw(pose_->pose.pose.orientation);
}

void referenceCB(const geometry_msgs::Pose2D::ConstPtr &ref_)
{
    xr[0]=ref_->x;
    xr[1]=ref_->y;
    xr[2]=ref_->theta;
}

int main(int argc,char* argv[])
{
    ros::init(argc,argv,"dtc");
    ros::NodeHandle node;

    std::vector<double> lambdaVec;
    if(!node.getParam("lambda",lambdaVec))
    {
        ROS_ERROR("No 'lambda' in controller %s", node.getNamespace().c_str());
        return false;
    }
    Eigen::Vector3d lambda=Eigen::Map<Eigen::Vector3d>(lambdaVec.data()).transpose();

    std::vector<double> gammaVec;
    if(!node.getParam("gamma",gammaVec))
    {
        ROS_ERROR("No 'gamma' in controller %s", node.getNamespace().c_str());
        return false;
    }
    Eigen::Vector2d gamma=Eigen::Map<Eigen::Vector2d>(gammaVec.data()).transpose();

    std::vector<double> KpVec;
    if(!node.getParam("Kp",KpVec))
    {
        ROS_ERROR("No 'Kp' in controller %s", node.getNamespace().c_str());
        return false;
    }
    Eigen::Vector2d Kp=Eigen::Map<Eigen::Vector2d>(KpVec.data()).transpose();

    std::vector<double> KiVec;
    if(!node.getParam("Ki",KiVec))
    {
        ROS_ERROR("No 'Ki' in controller %s", node.getNamespace().c_str());
        return false;
    }
    Eigen::Vector2d Ki=Eigen::Map<Eigen::Vector2d>(KiVec.data()).transpose();

    ros::Subscriber sub_realPose=node.subscribe("/gazebo//model_states",1,&realPoseCB);
    ros::Subscriber sub_status=node.subscribe("/dynamics_linearizing_controller/status",1,&statusCB);
    ros::Subscriber sub_odom=node.subscribe("/dynamics_linearizing_controller/odom",100,&poseCB);
    ros::Subscriber sub_ref=node.subscribe("reference",1,&referenceCB);
    ros::Publisher pub_command=node.advertise<geometry_msgs::Accel>("/dynamics_linearizing_controller/command",1);
    ros::Publisher pub_cstatus=node.advertise<discontinuous_transformation_controller_msgs::DiscontinuousTransformationControllerStatus>("/dcc_status",1);

    geometry_msgs::Accel accel;
    discontinuous_transformation_controller_msgs::DiscontinuousTransformationControllerStatus status_;
    accel.linear.x=0.0;
    accel.angular.z=0.0;

    Eigen::MatrixXd R;
    Eigen::Vector3d xhat;
    Eigen::Vector2d ur;
    Eigen::Vector2d error;
    Eigen::Vector2d last_error;
    last_error.setZero();
    double e;
    double alpha;
    double psi;
    double last_v1=0.0;
    double last_v2=0.0;

    //set initial pose and pose reference to zero
    x.setZero();
    u.setZero();

    std::vector<control_toolbox::Pid> pid(2);
    for(int i=0;i<2;i++)
    {
        pid[i].initPid(Kp[i],Ki[i],0.0,0.0,-0.0);
    }

    ros::Time last_time = ros::Time::now();
    ros::Rate loop(100);
    while (ros::ok())
    {
        //origin transformation
        R=(Eigen::MatrixXd(3,3)<<
                    std::cos(xr[2]), std::sin(xr[2]), 0,
                    -std::sin(xr[2]), std::cos(xr[2]), 0,
                    0, 0, 1).finished();
        xhat=R*(x-xr);
        std::cout<<"theta: "<<x[2]<<std::endl;
        //coordinate system transformation
        e=sqrt(pow(xhat[0],2)+pow(xhat[1],2));
        psi=atan2(xhat[1],xhat[0]);
        alpha=xhat[2]-psi;
        std::cout<<"e: "<<e<<"  psi: "<<psi<<"  alpha: "<<alpha<<std::endl;
        std::cout<<"xhat: \n"<<xhat<<std::endl;

        //non-linear controller
        ur[0]=-gamma[0]*e*std::cos(alpha);
        //check if alpha is too small (sin(x)/x ->1 when x->0)
        if(fabs(alpha)>DBL_EPSILON)
            ur[1]=-gamma[1]*alpha
            -gamma[0]*std::sin(alpha)*std::cos(alpha)
            +gamma[0]*(lambda[2]/lambda[1])*std::cos(alpha)*std::sin(alpha)*(psi/alpha);
        else
            ur[1]=+gamma[0]*(lambda[2]/lambda[1])*psi;

        std::cout<<"ur: \n"<<ur<<std::endl;
        std::cout<<"u: \n"<<u<<std::endl;
        //PI controller
        ros::Time time = ros::Time::now();
        error=ur-u;
        std::cout<<"u error: \n"<<error<<std::endl;
        accel.linear.x=pid[0].computeCommand(error[0],time-last_time);//last_v1+(Kp[0]+Ki[0]/200.0)*error[0]+(Ki[0]/200-Kp[0])*last_error[0];//
        accel.angular.z=pid[1].computeCommand(error[1],time-last_time);//last_v2+(Kp[1]+Ki[1]/200.0)*error[1]+(Ki[1]/200-Kp[1])*last_error[1];//
        last_time=time;
        last_error[0]=error[0];
        last_error[1]=error[1];
        last_v1=accel.linear.x;
        last_v2=accel.angular.z;
        std::cout<<"v1: "<<accel.linear.x<<" v2: "<<accel.angular.z<<std::endl;
        //publish acceleration reference for dynamics linearizing controller
        pub_command.publish(accel);
        //publish status of controller
        status_.header.stamp=ros::Time::now();
        status_.poseReference.x=xr[0];
        status_.poseReference.y=xr[1];
        status_.poseReference.theta=xr[2];
        status_.pose.x=x[0];
        status_.pose.y=x[1];
        status_.pose.theta=x[2];
        status_.ur[0]=ur[0];
        status_.ur[1]=ur[1];
        status_.v[0]=accel.linear.x;
        status_.v[0]=accel.angular.z;
        status_.torque[0]=torque[0];
        status_.torque[1]=torque[1];
        pub_cstatus.publish(status_);
        std::cout<<"------"<<std::endl;
        ros::spinOnce();
        if(!loop.sleep()) ROS_WARN("Missed deadline!");
    }
    
    sub_odom.shutdown();
    sub_ref.shutdown();
    sub_status.shutdown();
    sub_realPose.shutdown();
    pub_command.shutdown();
    pub_cstatus.shutdown();

    return 0;
}