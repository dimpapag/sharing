#include<ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>


double x,y,theta;

double x_des;
double y_des;
double theta_des;

void msgCallback(const turtlesim::Pose& in_msg){

    x = in_msg.x;
    y = in_msg.y;
    theta = in_msg.theta;
    std::cout<<"x: "<< x<< ", y:"<< y<< ", theta:"<< theta<< std::endl;  
}

void msgCallback_des(const turtlesim::Pose& in_msg){

    x_des = in_msg.x;
    y_des = in_msg.y;
    theta_des = in_msg.theta;
    //std::cout<<"x: "<< x<< ", y:"<< y<< ", theta:"<< theta<< std::endl;  
}




int main(int argc, char **argv){


    ros::init(argc,argv,"ihu_test");

    ros::NodeHandle nh;





    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel",1000);


    ros::Subscriber sub = nh.subscribe("turtle1/pose",1000,&msgCallback);
    ros::Subscriber sub_des = nh.subscribe("desired_pose",1000,&msgCallback_des);

    srand(time(0)) ;


    ros::Rate rate(100) ;

    double k =1.0;

    while(ros::ok()){
        geometry_msgs::Twist msgDimitris;

        //msgDimitris.linear.x = double(rand( ))/double(RAND_MAX);
       // msgDimitris.linear.y
        //msgDimitris.linear.z
       // msgDimitris.angular.x
       // msgDimitris.angular.y
        //msgDimitris.angular.z = 2.0 * double(rand( ))/double(RAND_MAX) - 1.0;

        Eigen::Vector3d v, omega;


        v(0) = k * (x_des - x);
        v(1) = k * (y_des - y);
        v(2) = 0.0;
        omega(0) = 0.0;
        omega(1) = 0.0;
        omega(2) = k * (theta_des - theta);

        Eigen::Matrix3d R0r = Eigen::Matrix3d::Identity(3,3);
        R0r(0,0) = cos(theta);
        R0r(1,1) = cos(theta);
        R0r(0,1) = -sin(theta);
        R0r(1,0) = sin(theta);

        v = R0r.transpose() * v;
        omega = R0r.transpose() * omega;

        msgDimitris.linear.x = v(0);
        msgDimitris.linear.y = v(1);
        msgDimitris.angular.z = omega(2);

        pub.publish(msgDimitris);

        ros::spinOnce();

        rate.sleep();

    }



    // Initializations
    Eigen::Vector3d p = Eigen::Vector3d::Random(3);
    Eigen::Matrix3d R  = Eigen::Matrix3d::Identity(3,3);
    Eigen::MatrixXd g = Eigen::MatrixXd::Identity(4,4);


    R(0,0) = cos(1);
    R(1,1) = cos(1);
    R(0,1) = -sin(1);
    R(1,0) = sin(1);

    g.block(0,0,3,3) = R;
    g.block(0,3,3,1) = p;


    // printouts
    std::cout<<"p = "<<std::endl << p << std::endl;
    std::cout<<"R = "<<std::endl << R << std::endl;
    std::cout<<"g = "<<std::endl << g << std::endl;

    std::cout<<"g*g^(-1):"<<std::endl<< g * g.inverse() <<std::endl;


    std::cout<<"Hello world from ROS!!"<<std::endl;


}