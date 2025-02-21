#include "trajectory_for_grasping/macchina_stati.h"
#define Npunti 200

namespace trajectory_for_grasping {

    MacchinaStati::MacchinaStati():tfListener(tfBuffer){ //costruttore
        this->state=INIT ; 
        this->return_state=WAIT_BUTTON ; 
        this->initializeRos() ; 
        this->haptic_joints.resize(6) ; 
        this->franka_pose.resize(7) ; 
        this->franka_prec.resize(7); 
        this->Haptic_pose.resize(7) ; 
        this->Offset.resize(3) ; 
        this->BezierCurve.resize(Npunti) ; 
        for(int i=0;i<Npunti;i++) {
            BezierCurve[i].resize(7) ; 
        }
        this->pos_fin.resize(7) ; 
        this->point_force.resize(7) ; 
        this->force.force_direction.resize(3) ; 
        this->stopping_force.force_direction.resize(3) ; 
        this->stopping_point_force.resize(3) ; 
        this->start_force=true  ;
        this->stop_force=true  ;
        this->count_start=0 ; 
        this->count_stop=0 ; 
        this->stop_gain=1 ; 
        this->start_gain=1 ;
        this->save_grey=true ; 
        this->save_white=true ; 
    }

    MacchinaStati::~MacchinaStati(){ //distruttore
    }

    void MacchinaStati::initializeRos() { 
        this->pub_franka_pose=this->n.advertise<geometry_msgs::PoseStamped>("/cartesian_impedance_example_controller/equilibrium_pose",1) ; //publisher definition
        this->pub_force_feedback=this->n.advertise<geomagic_control::OmniFeedback>("Geomagic/force_feedback",1) ; 
        this->sub_haptic_joints=this->n.subscribe("Geomagic/joint_states",1,&MacchinaStati::GeomagicJointsCallback,this) ; //subscriber definition
        this->sub_buttons=this->n.subscribe("Geomagic/button", 1, &MacchinaStati::ButtonsCallback, this);    
    }

    void MacchinaStati::GeomagicJointsCallback(const sensor_msgs::JointState& msg) {
        for(int i=0;i<6;i++) {
        this->haptic_joints[i]=msg.position[i] ; 
        }
    }

    void MacchinaStati::ButtonsCallback(const geomagic_control::PhantomButtonEvent& button_msg)
    {
        this->haptic_grey = button_msg.grey_button;
        this->haptic_white = button_msg.white_button;
  
    }

    MacchinaStati::State MacchinaStati::get_state() {
        return this->state;
    }

    Eigen::MatrixXd MDH(double theta,double alfa,double d,double a) {
    //-----------------------------------------------------------
    //funzione che genera la matrice cinematica sui parametri DH
    //-----------------------------------------------------------
    Eigen::MatrixXd DH(4,4) ; 
    DH(0,0) = cos(theta) ; 
    DH(0,1) = -sin(theta)*cos(alfa) ; 
    DH(0,2) = sin(theta)*sin(alfa) ; 
    DH(0,3) = a*cos(theta) ; 
    DH(1,0) = sin(theta) ; 
    DH(1,1) = cos(theta)*cos(alfa) ; 
    DH(1,2) = -cos(theta)*sin(alfa) ; 
    DH(1,3) = a*sin(theta) ; 
    DH(2,0) = 0 ; 
    DH(3,0) = 0 ;
    DH(3,1) = 0 ; 
    DH(3,2) = 0 ; 
    DH(2,1) = sin(alfa) ; 
    DH(2,2) = cos(alfa) ; 
    DH(2,3) = d ; 
    DH(3,3) = 1 ; 

    return DH ; 
    }

    std::vector<double> MappingPosition(std::vector<double> haptic_joint) {

        std::vector<double> alfa = {1.5708,0,1.5708,-1.5708,+1.5708,0} ; 
        std::vector<double> a = {0,0.13335,0,0,0,0} ; 
        std::vector<double> d = {0.11,0,0,0.13335,0,0} ;

        std::vector<Eigen::MatrixXd> DH_Matrix ; 
        DH_Matrix.resize(6) ; 

        for(int i=0;i<6;i++) {
        DH_Matrix[i]=MDH(haptic_joint[i],alfa[i],d[i],a[i]) ; 
        }

        Eigen::MatrixXd RotationMatrix(4,4) ;

        RotationMatrix = DH_Matrix[0] * DH_Matrix[1] ; 
        for(int i=2;i<6;i++) {
            RotationMatrix = RotationMatrix * DH_Matrix[i] ; 
        }

        std::vector<double> pose ; 
        pose = {RotationMatrix(0,3),RotationMatrix(1,3),RotationMatrix(2,3),0,0,0,0} ; 

        KDL::Rotation Rot ; 
        std::vector<double> R = {RotationMatrix(0,0),RotationMatrix(0,1),RotationMatrix(0,2),RotationMatrix(1,0),RotationMatrix(1,1),RotationMatrix(1,2),RotationMatrix(2,0),RotationMatrix(2,1),RotationMatrix(2,2),RotationMatrix(3,1),RotationMatrix(3,2),RotationMatrix(3,3)} ; 
        for(int i=0;i<9;i++) {
            Rot.data[i]=R[i]; 
        }

        Rot.DoRotY(3.1415926536) ; 
        Rot.GetQuaternion(pose[3],pose[4],pose[5],pose[6]) ; 

        return pose ;
    }

    std::vector<double> OffsetPosition(std::vector<double> robot_pos,std::vector<double> haptic_pos) { //calcolo offset tra haptic e franka

        std::vector<double> offset ; 
        offset.resize(3) ; 
        for(int i=0;i<3;i++) {
            offset[i] = robot_pos[i] - haptic_pos[i] ; 
        }    
    return offset ; 
    }

    double two_points_distance(std::vector<double> A,std::vector<double> B) {
        double distance; 
        distance = abs(sqrt(pow((B[0]-A[0]),2) + pow((B[1]-A[1]),2) + pow((B[2]-A[2]),2))) ; 
    return distance ; 
    }   

    double isteresi(double a) {
        if(a> 1.5) a = 1.5 ; 
        else if(a<-1.5) a = -1.5 ; 
    return a ; 
    }

    void MacchinaStati::calc_coefficients() { //calcola coefficienti della curva
         //Calcolo dei coefficienti della polinomiale 

        this->coeff_Bezier.coefficients_x.resize(3) ;   
        this->coeff_Bezier.coefficients_y.resize(3) ;   
        this->coeff_Bezier.coefficients_z.resize(3) ;   

        // POS_CTRL_1 E POS_CTRL_2 posizioni di controllo

        std::vector<double> pos_ctrl_1 ; 
        std::vector<double> pos_ctrl_2 ; 
        pos_ctrl_1.resize(7) ; 
        pos_ctrl_2.resize(7) ; 

        //posizione di controllo 1 definita come il versore della distanza per la lunghezza del prolungamento
        double proiezione = 0.05 ; //distanza del punto 1 rispetto al punto 0 
        double modulo_distanza = two_points_distance(franka_pose,franka_prec) ; //calcolo il modulo della distanza per calcolare il versore
        for(int i=0;i<3;i++) pos_ctrl_1[i] = this->franka_pose[i]+proiezione * this->franka_pose[i] / modulo_distanza ;

        //posizione di controllo 2 -- calcolata dal quaternione del punto finale
        std::vector<double> direzione_ctrl_2 ; 
        direzione_ctrl_2.resize(3) ; 
        direzione_ctrl_2[0] = 2*(pos_fin[4]*pos_fin[6]+pos_fin[5]*pos_fin[3]) ; 
        direzione_ctrl_2[1] = 2*(pos_fin[5]*pos_fin[6]-pos_fin[4]*pos_fin[3]) ; 
        direzione_ctrl_2[2] = pow(pos_fin[3],2)-pow(pos_fin[4],2)-pow(pos_fin[5],2)+pow(pos_fin[6],2) ; 
        for(int i=0;i<3;i++) pos_ctrl_2[i] = proiezione * direzione_ctrl_2[i] +pos_fin[i] ; 

        this->coeff_Bezier.coefficients_x[2] = 3.0 *(pos_ctrl_1[0] - franka_pose[0]) ; 
        this->coeff_Bezier.coefficients_x[1] = 3.0 *(pos_ctrl_2[0] - pos_ctrl_1[0]) - this->coeff_Bezier.coefficients_x[2] ; 
        this->coeff_Bezier.coefficients_x[0] = pos_fin[0] - franka_pose[0] - this->coeff_Bezier.coefficients_x[2] - this->coeff_Bezier.coefficients_x[1] ; 

        this->coeff_Bezier.coefficients_y[2] = 3.0 * (pos_ctrl_1[1] - franka_pose[1]) ; 
        this->coeff_Bezier.coefficients_y[1] = 3.0 * (pos_ctrl_2[1] - pos_ctrl_1[1]) - this->coeff_Bezier.coefficients_y[2] ; 
        this->coeff_Bezier.coefficients_y[0] = pos_fin[1] - franka_pose[1] - this->coeff_Bezier.coefficients_y[2] - this->coeff_Bezier.coefficients_y[1] ; 

        this->coeff_Bezier.coefficients_z[2] = 3.0 * (pos_ctrl_1[2] - franka_pose[2]) ; 
        this->coeff_Bezier.coefficients_z[1] = 3.0 * (pos_ctrl_2[2] - pos_ctrl_1[2]) - this->coeff_Bezier.coefficients_z[2] ; 
        this->coeff_Bezier.coefficients_z[0] = pos_fin[2] - franka_pose[2] - this->coeff_Bezier.coefficients_z[2] - this->coeff_Bezier.coefficients_z[1] ; 

    }

    void MacchinaStati::ComputeBezier() {

        /* CURVA BEZIER 
        x = ax * t^3 + bx * t^2 + cx * t + x0 
        y = ay * t^3 + by * t^2 + cy * t + y0
        z = az * t^3 + bz * t^2 + cz * t + z0  */

        double dt = 1.0/(Npunti-1) ; 

        std::vector<double> target ; 
        target.resize(7) ; 

        for(int i=0;i<Npunti;i++) {
        for(int j=3;j>0;j--) {
            target[0] += this->coeff_Bezier.coefficients_x[3-j]*pow(dt*i,j) ; 
            target[1] += this->coeff_Bezier.coefficients_y[3-j]*pow(dt*i,j) ; 
            target[2] += this->coeff_Bezier.coefficients_z[3-j]*pow(dt*i,j) ;
        }

        for(int k=0;k<3;k++) target[k] += this->franka_pose[k] ; 

        for(int k=3;k<7;k++) {
            target[k]=this->pos_fin[k] ; 
        }
        this->BezierCurve[i] = target ; 

        for(int k=0;k<7;k++) target[k] =0 ;  
        }
    }

    std::vector<double> nearest_point(std::vector<std::vector<double>> Curve,int n_points,std::vector<double> EndEffector) {
        std::vector<double> near ; 
        near.resize(7) ; 

        near = Curve[0] ; 
        for(int i=1;i<n_points;i++) {
            if(two_points_distance(EndEffector,Curve[i])<two_points_distance(EndEffector,near)) near = Curve[i] ; 
        }

    return near ; 
    }

    double Force_Module(std::vector<double> A,std::vector<double> B) {
        double force_module = 1.0/two_points_distance(A,B) ; 
       // if(force_module>2.5) force_module=2.5 ; 
    return force_module ;
    }

    /* METODI DEGLI STATI DELLA MACCHINA 
    ...
    ...
    ...
    ...
    ...
    */
    void MacchinaStati::state_init() {
            std::cout <<"\nINIT\n" ; 
            std::cout <<"Premi tasto grigio per iniziare\n" ; 
            //aspetto che si prema il tasto grigio per iniziare
            if(this->haptic_grey==1) { 
                this->state = WAIT_BUTTON ; 
                this->return_state = READ_POSITION ; 
            }

    }

    void MacchinaStati::state_wait_button() {
        std::cout <<"\nWAIT BUTTON\n" ; 
        if(haptic_grey==1 || haptic_white==1)  this->state= WAIT_BUTTON ;
        else this->state=return_state ; 
       
    }
    
    void MacchinaStati::state_read_position() { ////FARE TF2 LISTENER PER LA POSIZIONE PRECEDENTE!
            std::cout <<"\nREAD POSITION\n" ; 

            geometry_msgs::TransformStamped transformStamped ;
            try{
                    transformStamped = tfBuffer.lookupTransform("panda_link0","panda_EE",ros::Time(0)) ;   
                }
            catch (tf2::TransformException &ex) {
                    ROS_WARN("%s",ex.what()) ; 
                    ros::Duration(1.0).sleep() ; 
                    return;
            } 

                this->franka_pose[0] = transformStamped.transform.translation.x ;
                this->franka_pose[1] = transformStamped.transform.translation.y ;
                this->franka_pose[2] = transformStamped.transform.translation.z ; 
                this->franka_pose[3] = transformStamped.transform.rotation.x ;
                this->franka_pose[4] = transformStamped.transform.rotation.y  ;
                this->franka_pose[5] = transformStamped.transform.rotation.z ;
                this->franka_pose[6] = transformStamped.transform.rotation.w ; 

            //Mapping posizioni haptic
            Haptic_pose = MappingPosition(haptic_joints) ; 
            
            //Calcolo Offset
            Offset = OffsetPosition(franka_pose,Haptic_pose) ; 

            this->state=CONNECT_HAPTIC_FRANKA ; 

    }




    void MacchinaStati::state_connect_haptic_franka() {

        std::cout <<"\nCONNECT HAPTIC FRANKA\n" ; 

        Haptic_pose = MappingPosition(haptic_joints) ; 

        //aggiungo l'offset già calcolato precedentemente alla pose dell'haptic
        for(int i=0;i<3;i++) Haptic_pose[i] += Offset[i] ; 

        /* invio tf dell'haptic su Rviz
        static tf::TransformBroadcaster br ; 
        tf::Transform transform ; 
        transform.setOrigin(tf::Vector3(Haptic_pose[0],Haptic_pose[1],Haptic_pose[2])) ; 
        tf::Quaternion q(Haptic_pose[3],Haptic_pose[4],Haptic_pose[5],Haptic_pose[6]); 
        transform.setRotation(q) ; 
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),"panda_link0","Haptic_Pose")) ;  
        */

        geometry_msgs::PoseStamped send_pose ;

        send_pose.pose.position.x = Haptic_pose[0] ; 
        send_pose.pose.position.y = Haptic_pose[1] ; 
        send_pose.pose.position.z = Haptic_pose[2] ;
        send_pose.pose.orientation.x= Haptic_pose[3] ; 
        send_pose.pose.orientation.y = Haptic_pose[4] ;  
        send_pose.pose.orientation.z = Haptic_pose[5] ; 
        send_pose.pose.orientation.w = Haptic_pose[6] ; 

        pub_franka_pose.publish(send_pose) ; 

        if(this->haptic_white==1) {
            this->state=WAIT_BUTTON ; 
            this->return_state=OBJECT_DEFINITION ; 
        }
    }

    void MacchinaStati::state_object_definition() {
        std::cout <<"\nOBJECT DEFINITION\n" ; 
        // inserire qui definizione obiettivo 
        this->state=TRAJECTORY_CALCULATION ;

        this->pos_fin[0]=0.574101 ;
        this->pos_fin[1]=-0.218974 ; 
        this->pos_fin[2]=0.0277725; 
        this->pos_fin[3]=-0.999 ; 
        this->pos_fin[4]=-0.008 ; 
        this->pos_fin[5]=-0.0206 ; 
        this->pos_fin[6]=0.02161 ;

    }


    void MacchinaStati::state_trajectory_calculation() {

        std::cout <<"\nPremi tasto bianco per accendere la guida\n" ; 
        //aspetto che si prema il tasto bianco per il calcolo della traiettoria 

        std::cout <<"\nTRAJECTORY CALCULATION\n" ; 
        
        this->calc_coefficients() ; 

        this->ComputeBezier() ; 

        //mappo le posizioni della curva dallo spazio di lavoro del robot a quello dell'haptic, sottraendo l'offset

        for(int i=0;i<Npunti;i++) {
            for(int j=0;j<3;j++) {
                this->BezierCurve[i][j] -= this->Offset[j] ; 
            }
        }

        for(int i=0;i<Npunti;i++) {
            for(int j=0;j<3;j++) {
                if(j==0) BezierCurve[i][j] = BezierCurve[i][2] ; 
                else if(j==1) BezierCurve[i][j] = BezierCurve[i][0] ; 
                else if(j==2) BezierCurve[i][j] = BezierCurve[i][1]  ; 
            }
        }

        this->point_force=nearest_point(this->BezierCurve,Npunti,this->Haptic_pose) ; //calcola il punto della curva più vicino rispetto a dove si trova il robot

        this->force.force_module = Force_Module(this->Haptic_pose,this->point_force) ; //calcola modulo della forza 
        
        for(int i=0;i<3;i++) {
            this->force.force_direction[i]=(this->point_force[i]-this->Haptic_pose[i])/ two_points_distance(this->Haptic_pose,this->point_force) ; //calcola la direzione della forza 
        }

        count_start =0 ;
        start_force = true ; 
        start_gain=1 ; 

        this->state=FORCE_FIELD ; 

    }


    void MacchinaStati::state_force_field() {
        std::cout <<"\nFORCE FIELD\n" ; 

        this->Haptic_pose = MappingPosition(haptic_joints) ; 

        //aggiungo l'offset già calcolato precedentemente alla pose dell'haptic
        for(int i=0;i<3;i++) this->Haptic_pose[i] += this->Offset[i] ; 

        geometry_msgs::PoseStamped send_pose ;

        send_pose.pose.position.x = this->Haptic_pose[0] ; 
        send_pose.pose.position.y = this->Haptic_pose[1] ; 
        send_pose.pose.position.z = this->Haptic_pose[2] ;
        send_pose.pose.orientation.x= this->Haptic_pose[3] ; 
        send_pose.pose.orientation.y = this->Haptic_pose[4] ;  
        send_pose.pose.orientation.z = this->Haptic_pose[5] ; 
        send_pose.pose.orientation.w = this->Haptic_pose[6] ; 

        pub_franka_pose.publish(send_pose) ; //invio le posizioni al robot


         // invio su rviz tf punti della curva calcolati
        
        static tf::TransformBroadcaster br ; 
        tf::Transform transform ; 
        std::string str="BezierCurve" ; 
        for(int i=0;i<Npunti;i++) {
            ros::Time time=ros::Time::now() ; 
            transform.setOrigin(tf::Vector3(this->BezierCurve[i][0],this->BezierCurve[i][1],this->BezierCurve[i][2])) ; 
            tf::Quaternion q(BezierCurve[i][3],BezierCurve[i][4],BezierCurve[i][5],BezierCurve[i][6]); 
            transform.setRotation(q) ; 
            str += std::to_string(i) ; 
            br.sendTransform(tf::StampedTransform(transform,time,"panda_link0",str)) ; 
            str="BezierCurve" ; 
        }

        geomagic_control::OmniFeedback force_msg ; 

        force_msg.force.x= isteresi(this->force.force_module * this->force.force_direction[0]) ; 
        force_msg.force.y= isteresi(this->force.force_module * this->force.force_direction[1]) ; 
        force_msg.force.z= isteresi(this->force.force_module * this->force.force_direction[2]) ; 
        force_msg.position.x=this->point_force[0]  ;
        force_msg.position.x=this->point_force[1]  ;
        force_msg.position.x=this->point_force[2]  ;

        for(int i=0;i<3;i++) {
            this->stopping_force.force_direction[i] = force.force_direction[i] ; 
            this->stopping_point_force[i] = point_force[i] ; 
        }
        this->stopping_force.force_module = force.force_module ; 

       

        while(start_force) {

            force_msg.force.x= force_msg.force.x * start_gain ; 
            force_msg.force.y= force_msg.force.y * start_gain ; 
            force_msg.force.z= force_msg.force.z * start_gain ; 

            count_start++ ; 
            start_gain += 0.001 ; 
            if(count_start==999) start_force = false ; 
        }

        force_msg.lock.resize(3) ; 
        for(int i=0;i<3;i++) force_msg.lock[i] = 0 ; 

        this->pub_force_feedback.publish(force_msg) ; 

        if(haptic_white==1) {
            this->state=WAIT_BUTTON ; 
            this->return_state=STOP_FORCE ; 
            stop_force = true ; 
            stop_gain=1 ; 
            count_stop=0  ; 
            save_white = true ; 
        }
        else if(haptic_grey==1) {
            this->state=WAIT_BUTTON ; 
            this->return_state=STOP_FORCE ;
            stop_force = true ; 
            stop_gain=1 ; 
            count_stop=0 ;
            save_grey =true ; 
        }
    }

    void MacchinaStati::state_stop_force() {

        geomagic_control::OmniFeedback force_msg ; 

        while(stop_force) {
            force_msg.force.x= stop_gain*(isteresi(this->stopping_force.force_module * this->stopping_force.force_direction[0])) ; 
            force_msg.force.y= stop_gain*(isteresi(this->stopping_force.force_module * this->stopping_force.force_direction[1])) ; 
            force_msg.force.z= stop_gain*(isteresi(this->stopping_force.force_module * this->stopping_force.force_direction[2])) ; 

            force_msg.position.x=this->stopping_point_force[0]  ;
            force_msg.position.y=this->stopping_point_force[1]  ;
            force_msg.position.z=this->stopping_point_force[2]  ;
            
            force_msg.lock.resize(3) ; 
            for(int i=0;i<3;i++) force_msg.lock[i] = 0 ; 

            this->pub_force_feedback.publish(force_msg) ; 

            count_stop++ ; 
            stop_gain -= 0.001 ; 
            if(count_stop==999) stop_force =false ; 
        }

        if(!stop_force) {
            this->state=WAIT_BUTTON ; 
            if(save_white) this->return_state=CONNECT_HAPTIC_FRANKA ; 
            else if(save_grey) this->return_state=INIT ; 

            save_grey = false ; 
            save_white = false ; 
        }

    }
    
    void MacchinaStati::get_franka_prec() {


        geometry_msgs::TransformStamped transformStamped ;
            try{
                    transformStamped = tfBuffer.lookupTransform("panda_link0","panda_EE",ros::Time(0)) ;   
                }
            catch (tf2::TransformException &ex) {
                    ROS_WARN("%s",ex.what()) ; 
                    ros::Duration(1.0).sleep() ; 
                   return ; 
            } 

        this->franka_prec[0] = transformStamped.transform.translation.x ;
        this->franka_prec[1] = transformStamped.transform.translation.y ;
        this->franka_prec[2] = transformStamped.transform.translation.z ; 
        this->franka_prec[3] = transformStamped.transform.rotation.x ;
        this->franka_prec[4] = transformStamped.transform.rotation.y  ;
        this->franka_prec[5] = transformStamped.transform.rotation.z ;
        this->franka_prec[6] = transformStamped.transform.rotation.w ; 



    }
}
