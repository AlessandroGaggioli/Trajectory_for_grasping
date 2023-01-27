#include "trajectory_for_grasping/macchina_stati.h"

int main(int argc,char **argv) {
    
    ros::init(argc,argv,"macchina_stati") ;
    ros::Rate loop_rate(50);

    trajectory_for_grasping::MacchinaStati main_object ;

    while(ros::ok()) {

        switch(main_object.get_state()) {

        case(trajectory_for_grasping::MacchinaStati::State::INIT): {
            
            main_object.state_init() ; 
            break ; 
        }

        case(trajectory_for_grasping::MacchinaStati::State::READ_POSITION): {
            
            main_object.state_read_position() ; 
            break ; 
        }
        case(trajectory_for_grasping::MacchinaStati::State::CONNECT_HAPTIC_FRANKA): {
            
            main_object.state_connect_haptic_franka(); 
            break ; 
        }
        case(trajectory_for_grasping::MacchinaStati::State::OBJECT_DEFINITION): {
            
            main_object.state_object_definition() ; 
            break ; 
        }
        case(trajectory_for_grasping::MacchinaStati::State::TRAJECTORY_CALCULATION): {
            
            main_object.state_trajectory_calculation() ; 
            break ; 
        }
        case(trajectory_for_grasping::MacchinaStati::State::FORCE_FIELD): {
            
            main_object.state_force_field(); 
            break ; 
        }
        case(trajectory_for_grasping::MacchinaStati::WAITBUTTON): {
            main_object.state_wait_button() ; 
            break ; 
        }
    }
        main_object.get_franka_prec() ; 

        ros::spinOnce();
        loop_rate.sleep(); 
    }
}
