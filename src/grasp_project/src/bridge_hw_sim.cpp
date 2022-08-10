#include <bridge_hw_sim.h>

std::unique_ptr<senseglove::SenseGloveSetup> build(AllowedRobot robot, int nr_of_glove, bool is_right);

brige_hw_sim::brige_hw_sim(){

    /*Topic you want to publish to simulation
    
    j4 and j3 MCP joint // j0 DIP and PIP joint
    index finger -> ff
    middle finger -> mf
    ring finger -> rf
    pinkle finger -> lf
    thumb -> th
    
    */
    pub_sim[0] = n_.advertise<std_msgs::Float64>("/sh_rh_ffj4_position_controller/command", 1);
    pub_sim[1] = n_.advertise<std_msgs::Float64>("/sh_rh_ffj0_position_controller/command", 1);
    pub_sim[2] = n_.advertise<std_msgs::Float64>("/sh_rh_ffj3_position_controller/command", 1);

    pub_sim[3] = n_.advertise<std_msgs::Float64>("/sh_rh_mfj4_position_controller/command", 1);
    pub_sim[4] = n_.advertise<std_msgs::Float64>("/sh_rh_mfj0_position_controller/command", 1); 
    pub_sim[5] = n_.advertise<std_msgs::Float64>("/sh_rh_mfj3_position_controller/command", 1);

    pub_sim[6] = n_.advertise<std_msgs::Float64>("/sh_rh_lfj4_position_controller/command", 1);
    pub_sim[7] = n_.advertise<std_msgs::Float64>("/sh_rh_lfj0_position_controller/command", 1);
    pub_sim[8] = n_.advertise<std_msgs::Float64>("/sh_rh_lfj3_position_controller/command", 1);

    pub_sim[9] = n_.advertise<std_msgs::Float64>("/sh_rh_rfj4_position_controller/command", 1);
    pub_sim[10] = n_.advertise<std_msgs::Float64>("/sh_rh_rfj0_position_controller/command", 1);
    pub_sim[11] = n_.advertise<std_msgs::Float64>("/sh_rh_rfj3_position_controller/command", 1);

    pub_sim[12] = n_.advertise<std_msgs::Float64>("/sh_rh_thj4_position_controller/command", 1);
    pub_sim[13] = n_.advertise<std_msgs::Float64>("/sh_rh_thj2_position_controller/command", 1);
    pub_sim[14] = n_.advertise<std_msgs::Float64>("/sh_rh_thj1_position_controller/command", 1);
    //Topic subscribe from hardware
    sub_hw = n_.subscribe("/senseglove/0/rh/joint_states", 1, &brige_hw_sim::jointCallback, this);
    sub_hw_init = n_.subscribe("/senseglove/0/rh/joint_states", 1, &brige_hw_sim::initJointCallback, this);

    //Subcriber for distance
    sub_dist = n_.subscribe("/rh/senseglove/finger_distances", 1, &brige_hw_sim::distCallback, this);

    //Subcriber for contact force
    // distal : tip 
    sub_force[0] = n_.subscribe("/contacts/rh_ff/distal", 1, &brige_hw_sim::forceCallback_ff, this);
    sub_force[1] = n_.subscribe("/contacts/rh_mf/distal", 1, &brige_hw_sim::forceCallback_mf, this);
    sub_force[2] = n_.subscribe("/contacts/rh_lf/distal", 1, &brige_hw_sim::forceCallback_lf, this);
    sub_force[3] = n_.subscribe("/contacts/rh_rf/distal", 1, &brige_hw_sim::forceCallback_rf, this);
    sub_force[4] = n_.subscribe("/contacts/rh_th/distal", 1, &brige_hw_sim::forceCallback_th, this);
    sub_force[5] = n_.subscribe("/contacts/rh_ff/middle", 1, &brige_hw_sim::forceCallback_ff, this);
    sub_force[6] = n_.subscribe("/contacts/rh_mf/middle", 1, &brige_hw_sim::forceCallback_mf, this);
    sub_force[7] = n_.subscribe("/contacts/rh_lf/middle", 1, &brige_hw_sim::forceCallback_lf, this);
    sub_force[8] = n_.subscribe("/contacts/rh_rf/middle", 1, &brige_hw_sim::forceCallback_rf, this);
    sub_force[9] = n_.subscribe("/contacts/rh_th/middle", 1, &brige_hw_sim::forceCallback_th, this);
    
    //calling senseglove robot from running process
    selected_robot = AllowedRobot("dk1_right");
    setup = build(selected_robot, 1, true);
    initial=true;
    
}

brige_hw_sim::~brige_hw_sim(){}

void brige_hw_sim::initJointCallback(sensor_msgs::JointState joint){
    if (initial==true){
        int command;
        std::cout<<"put four finger straight!(1 means ok)"<<std::endl;
        std::cin>>command;
        
        if (command == 1){
            int j = 0;
            for(int i=0;i<15;i++){
                if(i%3 == 0 and i!=0) j++;
                hand_relax[i] = joint.position[i+j];

                std::cout<<"Open"<<i<<":  "<<hand_relax[i]<<std::endl;
                //thumb may need debug
                }

            
        }

        else if (command == 2){
            int j = 0;
            for(int i=0;i<15;i++){
                if(i%3 == 0 and i!=0) j++;
                hand_fist[i] = joint.position[i+j];
                std::cout<<"Fist"<<i<<":  "<<hand_fist[i]<<std::endl;
                //thumb may need debug 
            }
        }
        else{
            std::cout<<"Calibration DONE!!!"<<std::endl;
            initial=false;
        }

    }
    
        
}

void brige_hw_sim::jointCallback(sensor_msgs::JointState joint){

    if(initial==false){
        std_msgs::Float64 pub_joint;
        // 2 fingers 
        if (distance<35)
        {
            for(int i=0;i<15;i++){
                if(i%3==0){
                    pub_joint.data = 0;
                }else{
                    pub_joint.data = 1.57;
                }
                pub_sim[i].publish(pub_joint);
            }
        }else{
            int j=0;
            int k=0;
            for(int i=0;i<15;i++){ // i is senseglove joint 
                if (i%3 == 0 and i!=0) j++;

                if(i == 0 or i ==3 ){
                    // index middle relax positive
                    pub_joint.data =  -(((joint.position[i+j] + abs(hand_relax[i] ))*0.35/( 2*abs(hand_fist[i]-hand_relax[i]) ) )-0.175) ;
                    // std::cout<<"sense_index_brake:"<<pub_joint.data<<std::endl;
                    pub_sim[i].publish(pub_joint);
                }
                else if(i==6 or i==9){
                    //relax negative
                    pub_joint.data = (((joint.position[i+j] + abs(hand_relax[i]))*0.35/( 2*abs(hand_fist[i]-hand_relax[i]) ) )-0.175) ;
                    //std::cout<<"sense_index_ring_pinky:"<<pub_joint.data<<std::endl;
                    pub_sim[i].publish(pub_joint);
                }
                else if(i%3==1 and i<12){
                    pub_joint.data = (joint.position[i+j] - hand_relax[i])*3.14/(hand_fist[i]-hand_relax[i]);
                    pub_sim[i].publish(pub_joint);
                }
                else if(i%3==2 and i<12){
                    pub_joint.data = (joint.position[i+j] - hand_relax[i])*1.57/(hand_fist[i]-hand_relax[i]);
                    pub_sim[i].publish(pub_joint);
                }
                else{
                    pub_joint.data = joint.position[i+j];
                    if(i == 12) pub_joint.data = (joint.position[i+j] - hand_relax[i])*1.3/(hand_fist[i]-hand_relax[i]);
                    if(i == 13) pub_joint.data = (joint.position[i+j] - hand_relax[i])*0.7/(hand_fist[i]-hand_relax[i]);
                    if(i == 14) pub_joint.data = (abs(joint.position[i+j]) - std::min(abs(hand_relax[i]), abs(hand_fist[i])) )*1.57/abs(hand_fist[i]-hand_relax[i]);

                    pub_sim[i].publish(pub_joint);
                }
            }
        }
    }
}

void brige_hw_sim::distCallback(senseglove_shared_resources::FingerDistanceFloats msg){
    distance = msg.th_ff.data;    
}

void brige_hw_sim::forceCallback_ff(gazebo_msgs::ContactsState msg){
    if(!msg.states.empty()){
        senseglove::SenseGloveRobot& robot = setup->getSenseGloveRobot(0);
        double force;
        for(int i=0;i<msg.states.size();i++){
            force += msg.states[i].total_wrench.force.x;
            force += msg.states[i].total_wrench.force.y;
            force += msg.states[i].total_wrench.force.z;
        }
        if(force>70){
            force=70;
        }else if(force<40){
            force=force*2;
        }
        robot.actuateEffort(0, force, 0, 0, 0);
        robot.actuateBuzz(0, 50, 0, 0, 0);
    }
}

void brige_hw_sim::forceCallback_mf(gazebo_msgs::ContactsState msg){
    if(!msg.states.empty()){
        senseglove::SenseGloveRobot& robot = setup->getSenseGloveRobot(0);
        double force;
        for(int i=0;i<msg.states.size();i++){
            force += msg.states[i].total_wrench.force.x;
            force += msg.states[i].total_wrench.force.y;
            force += msg.states[i].total_wrench.force.z;
        }
        if(force>70){
            force=70;
        }else if(force<40){
            force=force*2;
        }
        robot.actuateEffort(0, 0, force, 0, 0);
        robot.actuateBuzz(0, 0, 50, 0, 0);
    }
}
void brige_hw_sim::forceCallback_rf(gazebo_msgs::ContactsState msg){
    if(!msg.states.empty()){
        senseglove::SenseGloveRobot& robot = setup->getSenseGloveRobot(0);
        double force;
        for(int i=0;i<msg.states.size();i++){
            force += msg.states[i].total_wrench.force.x;
            force += msg.states[i].total_wrench.force.y;
            force += msg.states[i].total_wrench.force.z;
        }
        if(force>70){
            force=70;
        }else if(force<40){
            force=force*2;
        }
        robot.actuateEffort(0, 0, 0, force, 0);
        robot.actuateBuzz(0, 0, 0, 50, 0);
    }
}
void brige_hw_sim::forceCallback_lf(gazebo_msgs::ContactsState msg){
    if(!msg.states.empty()){
        senseglove::SenseGloveRobot& robot = setup->getSenseGloveRobot(0);
        double force;
        for(int i=0;i<msg.states.size();i++){
            force += msg.states[i].total_wrench.force.x;
            force += msg.states[i].total_wrench.force.y;
            force += msg.states[i].total_wrench.force.z;
        }
        if(force>70){
            force=70;
        }else if(force<40){
            force=force*2;
        }
        robot.actuateEffort(0, 0, 0, 0, force);
        robot.actuateBuzz(0, 0, 0, 0, 50);
    }
}
void brige_hw_sim::forceCallback_th(gazebo_msgs::ContactsState msg){
    if(!msg.states.empty()){
        senseglove::SenseGloveRobot& robot = setup->getSenseGloveRobot(0);
        double force;
        for(int i=0;i<msg.states.size();i++){
            force += msg.states[i].total_wrench.force.x;
            force += msg.states[i].total_wrench.force.y;
            force += msg.states[i].total_wrench.force.z;
        }
        if(force>70){
            force=70;
        }else if(force<40){
            force=force*2;
        }
        robot.actuateEffort(force, 0, 0, 0, 0);
        robot.actuateBuzz(50, 0, 0, 0, 0);
    }
}


int main(int argc, char** argv){
    ros::init(argc, argv, "Bridge_Node");
    brige_hw_sim bridge;
    ros::spin();

    return 0;

}

std::unique_ptr<senseglove::SenseGloveSetup> build(AllowedRobot robot, int nr_of_glove, bool is_right)
{
  HardwareBuilder builder(robot, nr_of_glove, is_right);
  try
  {
    return builder.createSenseGloveSetup();
  }
  catch (const std::exception& e)
  {
    ROS_FATAL("Hardware interface caught an exception during building hardware");
    ROS_FATAL("%s", e.what());
    std::exit(1);
  }
}