#ifndef MACCHINA_STATI
#define MACCHINA_STATI 

#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "sensor_msgs/JointState.h"
#include <iostream>
#include <Eigen/Dense>
#include <kdl/frames.hpp>
#include <string>
#include <cmath>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include "geometry_msgs/PointStamped.h" 
#include "tf2_ros/message_filter.h"
#include "message_filters/subscriber.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geomagic_control/PhantomButtonEvent.h"
#include <geomagic_control/OmniFeedback.h>


namespace trajectory_for_grasping
{
    
    class MacchinaStati {
        
        public: 
        MacchinaStati() ; //costruttore
        virtual ~MacchinaStati();   //distruttore

        enum State { 
            INIT,
            READ_POSITION,
            CONNECT_HAPTIC_FRANKA,
            OBJECT_DEFINITION,
            TRAJECTORY_CALCULATION,
            FORCE_FIELD,
            WAIT_BUTTON
        } ;

        State state ; 

        typedef struct  {
            std::vector<double> coefficients_x ; 
            std::vector<double> coefficients_y ; 
            std::vector<double> coefficients_z ; 
        }coefficients ;

        typedef struct {
            std::vector<double> force_direction ; 
            double force_module ; 
        } Force ; 

        void initializeRos() ; 
        void get_franka_prec() ; 
        State get_state() ; 

        void state_init() ; 
        void state_read_position() ; 
        void state_connect_haptic_franka() ; 
        void state_object_definition() ; 
        void state_trajectory_calculation() ; 
        void state_force_field() ; 
        void state_wait_button() ;

        private: 

        //class variables
        ros::NodeHandle n ; 

        ros::Subscriber sub_haptic_joints  ; 
        ros::Subscriber sub_buttons ;

        ros::Publisher pub_franka_pose ; 
        ros::Publisher pub_force_feedback ; 

        tf2_ros::Buffer tfBuffer ; 
        tf2_ros::TransformListener tfListener(tf2_ros::Buffer tfBuffer) ; 

        std::vector<double> haptic_joints ; 
        std::vector<double> franka_pose ; 
        std::vector<double> franka_prec ; 
        std::vector<double> Haptic_pose ; 
        std::vector<double> Offset ; 
        std::vector<double> pos_fin ; 

        std::vector<std::vector<double>> BezierCurve ; 

        coefficients coeff_Bezier ;

        std::vector<double> point_force ; 

        Force force ; 

        State return_state ; 

        int haptic_grey;
        int haptic_white;
        
        //private methods
        void GeomagicJointsCallback(const sensor_msgs::JointState& msg) ;
        void ButtonsCallback(const geomagic_control::PhantomButtonEvent& button_msg);

        Eigen::MatrixXd MDH(double theta,double alfa,double d,double a) ; 
        std::vector<double> MappingPosition(std::vector<double> haptic_joint) ; 
        std::vector<double> OffsetPosition(std::vector<double> robot_pos,std::vector<double> haptic_pos) ; 
        std::vector<double> nearest_point(std::vector<std::vector<double>> Curve,int n_points,std::vector<double> EndEffector) ; 
        double Force_Module(std::vector<double> A,std::vector<double> B) ; 
        double isteresi(double a) ; 
        double two_points_distance(std::vector<double> A,std::vector<double> B) ; 
        void calc_coefficients() ;  
        void ComputeBezier()  ;

    } ; 
}

#endif //MACCHINA_STATI