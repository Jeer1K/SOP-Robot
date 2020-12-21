/*
    This code is used to control Dynamixel servos in ROS environment.
    It subscribes /inmoov/joint_trajectory and publishes inmoov/joint_states
    joint_trajectory takes in commands for driving the servo(s) to goal position(s).
    joint_states gives out info about the servo(s)' current position(s).
*/

#include <../../include/dynamixel_workbench.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

const char* port_name = "/dev/ttyUSB0";
int baud_rate = 57600;
int dxl_id;
int num_servos = 4;
int position;
float protocol_version = 2.0;

//const char *log;
bool result = false;

int32_t get_data = 0;
int positions[num_servos];

DynamixelWorkbench dxl_wb;


void init(){
    result = dxl_wb.init(port_name, baud_rate, &log);
    if (result == false)
    {
        printf("%s\n", log);
        printf("Failed to init\n");
        return 0;
    }
    else{
        printf("Succeed to init(%d)\n", baud_rate); 
    }
    result = dxl_wb.setPacketHandler(protocol_version, &log);
}


void joint_mode(){
    result = dxl_wb.jointMode(dxl_id, 0, 0, &log);
    if (result == false)
    {
        printf("%s\n", log);
        printf("Failed to change joint mode\n");
    }
    else
    {
        printf("Succeed to change joint mode\n");
    }
}


void nod(msg){
    // Message positions
    pos1 = msg.positions[1];
    pos2 = msg.positions[2];
    pos3 = msg.positions[3];
    pos4 = msg.positions[4];
    // Message velocities
    vel1 = msg.velocities[1];
    vel2 = msg.velocities[2];
    vel3 = msg.velocities[3];
    vel4 = msg.velocities[4];
    // Message accelerations
    vel1 = msg.accelerations[1];
    vel2 = msg.accelerations[2];
    vel3 = msg.accelerations[3];
    vel4 = msg.accelerations[4];
    // Message efforts
    eff1 = msg.effort[1];
    eff2 = msg.effort[2];
    eff3 = msg.effort[3];
    eff4 = msg.effort[4];
    // Time from start
    tim_start = msg.time_from_start;
    
    for (dxl_id = 1; dxl_id < (num_servos + 1); dxl_id++){
        result = dxl_wb.goalPosition(dxl_id, msg.positions[dxl_id]);
        if (result == false)
        {
            printf("%s\n", log);
            printf("Failed to set goal position\n");
        }
    }
    
    for (dxl_id = 1; dxl_id < (num_servos + 1); dxl_id++){
        result = dxl_wb.goalPosition(dxl_id, msg.positions[dxl_id]);
        if (result == false)
        {
            printf("%s\n", log);
            printf("Failed to set goal position\n");
        }
    }
    
    
}


void shake(msg){
}


void face_left(msg){
}


void face_right(msg){
}


void trajectoryCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("I heard: [%s]", msg->data.c_str());
    
    if( msg.header == 'nod') {
      /* if condition is true then print the following */
        printf("Value is nod\n" );
        nod(msg);
    } else if( msg.header == 'shake' ) {
      /* if else if condition is true */
        printf("Value is shake\n" );
        //shake(msg);
    } else if( msg.header == 'face_left') {
      /* if else if condition is true  */
        printf("Value is left\n" );
        //face_left(msg);
    } else if( msg.header == 'face_right') {
      /* if else if condition is true  */
        printf("Value is right\n" );
        //face_right(msg);
    } else if( msg.header == 'auto') {
      /* if else if condition is true  */
      /* if(msg.joint_names == "head_pan_joint")
            dxl_id = 4;
        if(msg.joint_names == "head_tilt_joint"){
            dxl_id = 3;
        if(msg.joint_names == "head_roll2_joint"){
            dxl_id = 2;
        if(msg.joint_names == "head_roll1_joint"){
            dxl_id = 1;
      */  
        
        printf("Value is auto\n" );
    else {
      /* if none of the conditions is true */
        printf("None of the values is matching\n" );
    }    
}
   

int main(){
    dxl_wb.init();
    joint_mode();
    ros::init(argc, argv, "head");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("inmoov/joint_trajectory", 1000, trajectoryCallback);
    
    ros::Publisher pub = nh.advertise<std_msgs::String>("join_states", 10);
    for (dxl_id = 1; dxl_id < (num_servos + 1); dxl_id++){
        result = dxl_wb.itemRead(dxl_id, "Present_Position", &get_data, &log);
        if (result == false)
        {
            printf("%s\n", log);
            printf("Failed to get present position\n");
        }
        else
        {
            positions[dxl_id] = get_data;
        }
    }
    msg.data = positions.str();
    pub.publish(msg)   
    ROS_INFO("%s", msg.data.c_str());
    ros::spin();
    return 0;    
}
