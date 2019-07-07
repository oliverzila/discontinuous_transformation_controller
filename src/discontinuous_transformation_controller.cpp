#include <ros/ros.h>

#include <geometry_msgs/Accel.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <linearizing_controllers_msgs/DynamicsLinearizingControllerStatus.h>

#include <eigen3/Eigen/Dense>
#include <tf/transform_datatypes.h>
#include <control_toolbox/pid.h>
#include <math.h>
#include <iostream>

Eigen::Vector2d u; //linear and angular velocity
Eigen::Vector3d x; //pose - x,y,theta
Eigen::Vector3d xr; //pose reference

void statusCB(const linearizing_controllers_msgs::DynamicsLinearizingControllerStatus::ConstPtr &status_)
{
    u[0]=status_->process_value.linear.x;
    u[1]=status_->process_value.angular.z;
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

    ros::Subscriber sub_status=node.subscribe("/dynamics_linearizing_controller/status",1000,&statusCB);
    ros::Subscriber sub_odom=node.subscribe("/dynamics_linearizing_controller/odom",1000,&poseCB);
    ros::Subscriber sub_ref=node.subscribe("reference",1,&referenceCB);
    ros::Publisher pub_command=node.advertise<geometry_msgs::Accel>("/dynamics_linearizing_controller/command",1);

    geometry_msgs::Accel accel;

    Eigen::MatrixXd R;
    Eigen::Vector3d xhat;
    Eigen::Vector2d ur;
    Eigen::Vector2d error;
    Eigen::Vector2d last_error;
    last_error[0]=0;
    last_error[1]=0;
    double e;
    double alpha;
    double psi;
    double last_v1=0;
    double last_v2=0;

    //set initial pose and pose reference to zero
    for(int i=0;i<3;i++)
    {
        x[i]=0;
        xr[i]=0;
    }

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
        std::cout<<"e: "<<e<<", psi: "<<psi<<", alpha: "<<alpha<<std::endl;
        std::cout<<"hxat: \n"<<xhat<<std::endl;

        //non-linear controller
        ur[0]=-gamma[0]*e*std::cos(alpha);
        //check if alpha is too small (sin(x)/x ->1 when x->0)
        if(fabs(atan2(std::sin(alpha),std::cos(alpha)))>DBL_EPSILON)
            ur[1]=-gamma[1]*atan2(std::sin(alpha),std::cos(alpha))
            -gamma[0]*std::sin(alpha)*std::cos(alpha)
            +gamma[0]*(lambda[2]/lambda[1])*std::cos(alpha)*std::sin(alpha)*(psi/alpha);
        else
            ur[1]=+gamma[0]*(lambda[2]/lambda[1])*psi;

        //PI controller

        ros::Time time = ros::Time::now();
        error=ur-u;
        std::cout<<"u error: \n"<<error<<std::endl;
        accel.linear.x=last_v1+184.0*error[0]-99.36*last_error[0];//pid[0].computeCommand(ur[0]-u[0],time-last_time);//
        accel.angular.z=last_v2+184.0*error[1]-99.36*last_error[1];//pid[1].computeCommand(ur[1]-u[1],time-last_time);//
        last_time=time;
        last_error[0]=error[0];
        last_error[1]=error[1];
        last_v1=accel.linear.x;
        last_v2=accel.angular.z;
        std::cout<<"Erro V: "<<ur[0]-u[0]<<std::endl;
        std::cout<<"Erro omega: "<<ur[1]-u[1]<<std::endl;

        //publish acceleration reference for dynamics linearizing controller
        pub_command.publish(accel);
        
        ros::spinOnce();
        if(!loop.sleep()) ROS_WARN("Missed deadline!");
    }
    
    return 0;
}