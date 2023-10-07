
#include "vex.h"



#ifndef MTO_H_
#define MTO_H_



namespace  mto {


//define funtion: 
    class Motor{

    public:

    motor *left_front;
    motor *right_front;
    motor *left_back;
    motor *right_back;

    float motor2center;
    float wheeldiameter;
    float max_width;
    float ratio;
    
    void leftgroup(double LVel);
    void rightgroup(double RVel);
    void brake();
};
class Rotation{
    public:
    rotation *front;
    rotation *back;
    rotation *right;
    rotation *left;
    float wheeldiameter;//inch
    inertial *inertial_sensor;
    float front_length;
    float back_length;
    float right_length;
    float left_length;
    
    float hor();
    float ver();
    float Heading();
    Rotation(rotation* front, rotation* back, rotation* right, rotation* left, float wheelDiameter, inertial* inertialSensor, float frontLength, float backLength, float rightLength, float leftLength)
    : front(front), back(back), right(right), left(left), wheeldiameter(wheelDiameter), inertial_sensor(inertialSensor), front_length(frontLength), back_length(backLength), right_length(rightLength), left_length(leftLength){}

};

class Encoder{
    public:
    encoder *front;
    encoder *back;
    encoder *right;
    encoder *left;
    float wheeldiameter;//inch
    inertial *inertial_sensor;
    float front_length;
    float back_length;
    float right_length;
    float left_length;
    
    float hor ();
    float ver (); 
    float Heading();
};

class location{
    public:
    float x;
    float y;
    float theta;
    
};
class Obstacle{
    public:
    float error;
    float error_angle;
    float Angle;
    float length;
    
    Obstacle(float error,float error_angle,float Angle,float length) : error(error),error_angle(error_angle),Angle(Angle),length(length){}
};



//function:
void setPID(float latKP=-1,float ngKP=-1,float OlatKP=-1,float OngKP=-1,float latKD=-1,float ngKD=-1,float latKI=-1,float ngKI=-1); 

location get_current(float x=-1,float y=-1,float theta = -1);

Obstacle square(float start_x,float start_y,float end_x,float end_y,float width,float height);

Obstacle circle (float center_x,float center_y,float diameter=15.0);

Obstacle obstacle ();

Obstacle sum(const Obstacle& accum,const Obstacle& s);

//main function:

void init();//背景odometry

void Move_abs(float target_x,float target_y,float final_theta,float speed,bool reverse=false ,float latKP=-1,float ngKP=-1,float OlatKP=-1,float OngKP=-1,float latKD=-1,float ngKD=-1,float latKI=-1,float ngKI=-1);

void Move(float target_x,float target_y,float final_theta,float speed,bool reverse=false ,float latKP=-1,float ngKP=-1,float OlatKP=-1,float OngKP=-1,float latKD=-1,float ngKD=-1,float latKI=-1,float ngKI=-1);

void Turn(float speed,float final_theta,bool reverse=false,float ngKP=-1,float ngKD=-1,float ngKI=-1);


//========================================================================
//
//                       initial settup
//
//========================================================================


static Motor Chassis{&leftfront,&rightfront,&leftback,&rightback,12.7,3.25,20.7};
static Rotation Tracking{&front_rot,nullptr,&right_rot,nullptr,2.75,&imu,16,0,6,0};
static location Current{0,0,0};


};



#endif