
#include "mto.h"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <numeric>
#include <stdexcept>
#include <string>
#include <vector>

using namespace mto;

//===================================================
//
//          obstacle setting
//
//===================================================
Obstacle mto::obstacle (){

    std::vector<Obstacle>obstacle_i={

        square(-7.5, 60, 7.5, 75, 15, 15)

    };

    return std::accumulate(obstacle_i.begin(),obstacle_i.end(),Obstacle(0.0,0.0,0.0,0.0),sum);

}
//====================================================
//
//                  main code
//
//====================================================

//----------------------------------------------------
//
//                  sensors output
//
//----------------------------------------------------
void mto::Motor::leftgroup(double LVel){

        this->left_front->setVelocity(LVel,percent);
        

        this->left_back->setVelocity(LVel,percent);

    };

void mto::Motor::rightgroup(double RVel){

        this->right_front->setVelocity(RVel,percent);

        this->right_back->setVelocity(RVel,percent);

}

void mto::Motor::brake(){

        this->left_front->stop(hold);

        this->left_back->stop(hold);

        this->right_front->stop(hold);

        this->right_back->stop(hold);

    };

float mto::Rotation::hor(){

    float value=0;

    int count=0;

    if(this->front!=nullptr){

        value+=this->front->position(degrees);

        count+=1;

    }

    if(this->back!=nullptr){

        value+=this->back->position(degrees);

        count+=1;

    }



    return value/count*this->wheeldiameter*M_PI/36000.0*2.54;

}

float mto::Rotation::ver(){

    float value=0;

    int count=0;

    if(this->right!=nullptr){

        value+=this->right->position(degrees);

        count+=1;

    }

    if(this->left!=nullptr){

        value+=this->left->position(degrees);

        count+=1;

    }

    return value/count*this->wheeldiameter*M_PI/36000.0*2.54;



}

float mto::Rotation::Heading(){

    return this->inertial_sensor->heading();

}

float mto::Encoder::hor() {

    float value=0;

    int count=0;

    if(this->front!=nullptr){

        value+=this->front->value();

        count+=1;

    }

    if(back!=nullptr){

        value+=this->back->value();

        count+=1;

    }

    return value/count*this->wheeldiameter*M_PI/360.0*2.54;

    }

float mto::Encoder::ver(){

    float value=0;

    int count=0;

    if(right!=nullptr){

        value+=this->right->value();

        count+=1;

    }

    if(left!=nullptr){

        value+=this->left->value();

        count+=1;

    }

    return value/count*this->wheeldiameter*M_PI/360.0*2.54;

}

float mto::Encoder::Heading(){

    return this->inertial_sensor->heading();

}

float heading2(bool reverse=false){//轉成範圍180~-180

    return (Tracking.Heading()+(reverse*180)-(360*(Tracking.Heading()+(reverse*180)>180)));//*(1-(reverse*2));

}

//----------------------------------------------------
//
//                  座標更新odometry
//
//----------------------------------------------------

float prehorizontal=0;

float prevertical=0;

float preheading=0;

int  Position(){
  while(1){
    float raw_hor=Tracking.hor();
    
    float raw_ver=Tracking.ver();

    float raw_imu=heading2();

    float delta_hor=raw_hor-prehorizontal;

    float delta_ver=raw_ver-prevertical;

    float delta_heading=raw_imu-preheading;

    delta_heading=delta_heading-((fabs(delta_heading)>180)*(delta_heading/fabs(delta_heading+(delta_heading==0))*360));//算出的值大於180度或小於-180度進行修正

    float delta_x=delta_hor+(delta_heading*M_PI/180.0*((Tracking.front_length-Tracking.back_length)/((Tracking.front!=nullptr)+(Tracking.back!=nullptr))));

    float delta_y=delta_ver+(delta_heading*M_PI/180.0*((Tracking.right_length-Tracking.left_length)/((Tracking.right!=nullptr)+(Tracking.left!=nullptr))));

    Current={

        Current.x+(delta_y*std::sin(float(raw_imu*M_PI/180.0)))-(delta_x*std::cos(float(raw_imu*M_PI/180.0))),

        Current.y+(delta_y*std::cos(float(raw_imu*M_PI/180.0)))+(delta_x*std::sin(float(raw_imu*M_PI/180.0))),

        heading2(),

        };

        prehorizontal=raw_hor;

        prevertical=raw_ver;

        preheading=raw_imu; 

        wait(10,msec);
        
  }

        return 0;   


}



void mto::init() {

    task postion_task = task (Position);
        

    

}

location mto::get_current(float x,float y,float theta){

    if (x!=-1)Current.x=x;
    if(y!=-1)Current.y=y;
    if(theta!=-1)Current.theta=theta;
    
    return Current;
}
//----------------------------------------------------
//
//                  躲避障礙物
//
//----------------------------------------------------
float target_x_2=0,target_y_2=0;

bool reverse2=false;

Obstacle mto::square(float start_x,float start_y,float end_x,float end_y,float width,float height){

    float x=Current.x,y=Current.y;

    float theta=atan(height/width);

    std::vector<float> vector_obstacle={(end_x-start_x)/hypotf(end_x-start_x, end_y-start_y),(end_y-start_y)/hypotf(end_x-start_x, end_y-start_y)};//BD向量


    std::vector<float> A={start_x+float((vector_obstacle[0]*std::cos(M_PI/2.0-theta)-vector_obstacle[1]*std::sin(M_PI/2.0-theta))*height),start_y+float((vector_obstacle[0]*std::sin(M_PI/2.0-theta)+vector_obstacle[1]*std::cos(M_PI/2.0-theta))*height)};

    std::vector<float> B={start_x,start_y};

    std::vector<float> C={start_x+((vector_obstacle[0]*std::cos(theta)+vector_obstacle[1]*std::cos(theta))*width),start_y+((vector_obstacle[0]*-std::sin(theta)+vector_obstacle[1]*std::cos(theta))*width)};

    std::vector<float> D={end_x,end_y};

    std::vector<float> M={(start_x+end_x)/2,(start_y+end_y)/2};//障礙物中心點

    std::vector<float> p_dir={target_x_2-x/hypotf(target_x_2, target_y_2),target_y_2-y/hypotf(target_x_2, target_y_2)};//現在車子到終點向量

    std::vector<float> BA={A[0]-B[0],A[1]-B[1]},CB={B[0]-C[0],B[1]-C[1]},DC={C[0]-D[0],C[1]-D[1]},AD={D[0]-A[0],D[1]-A[1]};

    std::vector<float> BA_N={(-A[1]+B[1])/hypotf(BA[0],BA[1]),(A[0]-B[0])/hypotf(BA[0],BA[1])};//BA法向量(單位向量)

    std::vector<float> CB_N={(-B[1]+C[1])/hypotf(CB[0],CB[1]),(B[0]-C[0])/hypotf(CB[0],CB[1])};//CB法向量(單位向量)

    std::vector<float> DC_N={(-C[1]+D[1])/hypotf(DC[0],DC[1]),(C[0]-D[0])/hypotf(DC[0],DC[1])};//DC法向量(單位向量)

    std::vector<float> AD_N={(-D[1]+A[1])/hypotf(AD[0],AD[1]),(D[0]-A[0])/hypotf(AD[0],AD[1])};//AD法向量(單位向量)

    std::vector<float> V={x-start_x,y-start_y};//障礙物開始點到現在車子點的向量

    float 
    dotBA= (V[0]*BA_N[0]+V[1]*BA_N[1]),
    dotCB= (V[0]*CB_N[0]+V[1]*CB_N[1]),
    dotDC= (V[0]*DC_N[0]+V[1]*DC_N[1]),
    dotAD= (V[0]*AD_N[0]+V[1]*AD_N[1]);

    float length = std::sqrt(pow(dotBA*(dotBA>0),2)+pow(dotCB*(dotCB>0),2)+pow((dotDC-width)*((dotDC-width)>0),2)+pow((dotAD-height)*((dotAD-height)>0),2));

    float
    dirBA=p_dir[0]*BA[0]+(p_dir[1]*BA[1]),
    dirCB=p_dir[0]*CB[0]+(p_dir[1]*CB[1]),
    dirDC=p_dir[0]*DC[0]+(p_dir[1]*DC[1]),
    dirAD=p_dir[0]*AD[0]+(p_dir[1]*AD[1]);

    float angle=std::atan2(M[0]-x,M[1]-y)*180.0/M_PI-heading2(reverse2);

    int direction = -1+(2*(((dotBA>0)*dirBA)+((dotCB>0)*dirCB)+((dotDC>0)*dirDC)+((dotAD>0)*dirAD)>0));

    return Obstacle(length*(length<(Chassis.max_width+10)),direction*(90-fabs(angle))*(length<(Chassis.max_width+10))*((90-fabs(angle)>0)),angle,length);


}

Obstacle mto::circle (float center_x,float center_y,float diameter){

    float x=Current.x,y=Current.y;

    std::vector<float> p_direction={target_x_2-x/hypotf(target_x_2 ,target_y_2),target_y_2-y/hypotf(target_x_2 ,target_y_2)};

    std::vector<float> p2center={center_x-x,center_y-y};

    float length=hypotf(p2center[0],p2center[1])-(diameter/2.0);

    float direction=(-1+(2*((p_direction[0]*-p2center[1]+(p_direction[1]*p2center[0]))>0)));

    float angle=std::atan2(p2center[0],p2center[1])*180.0/M_PI-heading2(reverse2);

    return Obstacle(length*(length<(Chassis.max_width+10)),direction*(90-fabs(angle))*(length<(Chassis.max_width+10))*((90-fabs(angle)>0)),angle,length);

}

Obstacle mto::sum(const Obstacle& accum,const Obstacle& s){

    return Obstacle(accum.error+s.error,accum.error_angle+s.error_angle,accum.Angle+s.Angle,accum.length+s.length);

}

//----------------------------------------------------
//
//                  座標移動pure pursuit
//
//----------------------------------------------------

//variable PID:

float old_latKP=1.3;//1.2
float old_ngKP=0.0015;//0.003
float old_OlatKP=0.2;//5
float old_OngKP=0.003;//0.05
float old_latKD=1;//1
float old_ngKD=0.001;//0.001
float old_latKI=0.0001;//0.0001
float old_ngKI=0.000002;//0.000002

void mto::setPID(float latKP,float ngKP,float OlatKP,float OngKP,float latKD,float ngKD,float latKI,float ngKI){
    if (latKP!=-1)old_OlatKP=latKP;
    if (ngKP!=-1)old_ngKP=ngKP;
    if (OlatKP!=-1)old_OlatKP=OlatKP;
    if (OngKP!=-1)old_OngKP=OngKP;
    if (latKI!=-1)old_latKI=latKI;
    if (ngKI!=-1)old_ngKI=ngKI;
    if (latKD!=-1)old_latKD=latKD;
    if (ngKD!=-1)old_ngKD=ngKD;
     
}

float  delta_length,error_delta_length,total_delta_length;

float ACC(float max_speed,float Distance,float input_distance){

  Distance=fabs(Distance);

  input_distance=fabs(input_distance);

  //max_speed[理想最高速度]  

float m=1.5;//斜率[(最佳最高速度-30.0[初始速度])/移動距離)]

float high_speed=20.0+((Distance/2.0)*m);//high_speed[實際最高速度]

    float output_speed=0;

  delta_length=Distance-input_distance;  

if (high_speed>max_speed){

       high_speed=max_speed;

  }

  if (20.0+(input_distance*m)<high_speed){//加速度

    output_speed=20.0+(input_distance*m);

  }

  else if(input_distance>(Distance*2/3)){//減速度

    output_speed=fabs(high_speed-(high_speed*pow(0.9,((delta_length*old_latKP)+(total_delta_length*old_latKI)+((delta_length-error_delta_length)*old_latKD)))));

  }

  else{//等速

    output_speed=high_speed;

  }

  total_delta_length+=delta_length;

  error_delta_length=delta_length;

  return output_speed;

}

void turn_coordinate(float final_theta,bool reverse,float ngKP,float ngKD,float ngKI,float start_theta,float final_speed){

    int condition2=0;

    float total_delta_theta=0,error_delta_theta=0;

    old_ngKP=ngKP;

    old_ngKD=ngKD;

    old_ngKI=ngKI;

    while (!(condition2>5)) {    //當condition1 成立時跳出迴圈時 只校正車子方向

        float delta_theta=final_theta-heading2(reverse) + start_theta;

        delta_theta=delta_theta-((fabs(delta_theta)>180)*(delta_theta/fabs(delta_theta+(delta_theta==0))*360));//算出的值大於180度或小於-180度進行修正

        float turn=((delta_theta*ngKP)+(total_delta_theta*ngKI)+((delta_theta-error_delta_theta)*ngKD))*Chassis.motor2center*final_speed;

        Chassis.leftgroup(turn*3.25/Chassis.wheeldiameter*Chassis.ratio/1.7);

        Chassis.rightgroup(-turn*3.25/Chassis.wheeldiameter*Chassis.ratio/1.7);    

        condition2+=(fabs(delta_theta)<2);

        total_delta_theta+=delta_theta;

        error_delta_theta=delta_theta;

        wait(10,msec);   

    }

}

void move_coordinatev3(float target_x,float target_y,float final_theta,float speed,bool reverse ,float latKP,float ngKP,float OlatKP,float OngKP,float latKD,float ngKD,float latKI,float ngKI,bool center){

    float start_x=Current.x,start_y=Current.y,start_theta=heading2(reverse);

    int condition1 = 0 ,condition2 = 0 ;

    float error_delta_theta=0, total_delta_theta=0.0;

    total_delta_length=0;

    target_x_2=target_x,target_y_2=target_y,reverse2=reverse;//this is for obstacle function

    float final_speed=0;

    old_latKP=latKP;//renew the pid value

    old_OlatKP=OlatKP;

    old_OngKP=OngKP;

    old_latKD=latKD;

    old_latKI=latKI;

    while(!condition1){

    Obstacle delta_wall= obstacle();//計算障礙物的距離

    float delta_x = (target_x*(1 - (reverse * 2)))+(start_x*center)-(Current.x),delta_y=(target_y*(1 - (reverse * 2)))+(start_y*center)-(Current.y);    

    float target_theta = atan2(delta_x,delta_y)*180.0/M_PI-heading2(reverse);//車子目前的車向到目標點的角度

    target_theta=target_theta-((fabs(target_theta)>180)*(target_theta/fabs(target_theta+(target_theta==0))*360));//算出的值大於180度或小於-180度進行修正

    float final_delta_theta=final_theta - heading2(reverse) + (start_theta*center);//目前車子朝向與目標朝向的角度差

    final_delta_theta= final_delta_theta-((fabs(final_delta_theta)>180)*(final_delta_theta/fabs(final_delta_theta+(final_delta_theta==0))*360));//算出的值大於180度或小於-180度進行修正

    float delta_theta = ( final_delta_theta - (2 * target_theta) ) ;//車子目前的車向到結束後車向的角度

    std::vector<float>start2target={target_x-(start_x*(!center)),target_y-(start_y*(!center))};//起始點到終點的向量(絕對座標)

    std::vector<float>start2Current={(Current.x)-start_x,(Current.y)-start_y};//起始點到車子目前位置的向

    float input_distance = (start2target[0]*start2Current[0]+(start2target[1]*start2Current[1]))/hypot(start2target[0],start2target[1]);//進度條(車子走的距離)

    float ACCspeed = ACC(speed, hypot(start2target[0],start2target[1]), input_distance);

    final_speed = (ACCspeed-((delta_wall.error!=0)*ACCspeed*std::pow(0.1,fabs(delta_wall.length)*OlatKP)))*(1 - (reverse * 2));//躲避障礙物減速度修正   (listed)

    float deltaVel = final_speed * (2 * Chassis.motor2center * std::sin(target_theta * M_PI / 180.0)/hypotf(delta_x, delta_y))*1;//橫移的速度差(走圓)   (listed)

    float turn = ( delta_theta * ngKP + (total_delta_theta*ngKI) + ((delta_theta-error_delta_theta) * ngKD))* Chassis.motor2center * final_speed;//車子轉向速度差   (listed)

    float Oturn = ( delta_wall.error_angle * OngKP ) * Chassis.motor2center * final_speed;//障礙物轉向  (listed)  

    Chassis.leftgroup((final_speed + deltaVel - turn - Oturn)*3.25/Chassis.wheeldiameter*Chassis.ratio/1.7);

    Chassis.rightgroup((final_speed - deltaVel + turn + Oturn)*3.25/Chassis.wheeldiameter*Chassis.ratio/1.7);

    total_delta_theta +=delta_theta;//KI

    error_delta_theta=delta_theta;//KD

    condition1=(fabs(delta_x)<5&&fabs(delta_y)<5);    

    wait(10,msec);

    }   

    turn_coordinate(final_theta, reverse, ngKP, ngKD, ngKI,start_theta*center,30);

    Chassis.brake();

    wait(10,msec);

}

//----------------------------------------------------
//
//                  人性化
//
//----------------------------------------------------

void mto::Move(float target_x,float target_y,float final_theta,float speed,bool reverse ,float latKP,float ngKP,float OlatKP,float OngKP,float latKD,float ngKD,float latKI,float ngKI){

    if(latKP==-1)latKP=old_latKP;

    if(ngKP==-1)ngKP=old_ngKP;

    if(OlatKP==-1)OlatKP=old_OlatKP;

    if(OngKP==-1)OngKP=old_OngKP;

    if(latKD==-1)latKD=old_latKD;

    if(ngKD==-1)ngKD=old_ngKD;

    if(latKI==-1)latKI=old_latKI;

    if(ngKI==-1)ngKI=old_ngKI;

    move_coordinatev3(target_x,target_y,final_theta,speed,reverse,latKP,ngKP,OlatKP,OngKP,latKD,ngKD,latKI,ngKI,true);

}

void mto::Move_abs(float target_x,float target_y,float final_theta,float speed,bool reverse ,float latKP,float ngKP,float OlatKP,float OngKP,float latKD,float ngKD,float latKI,float ngKI){

    if(latKP==-1)latKP=old_latKP;

    if(ngKP==-1)ngKP=old_ngKP;

    if(OlatKP==-1)OlatKP=old_OlatKP;

    if(OngKP==-1)OngKP=old_OngKP;

    if(latKD==-1)latKD=old_latKD;

    if(ngKD==-1)ngKD=old_ngKD;

    if(latKI==-1)latKI=old_latKI;

    if(ngKI==-1)ngKI=old_ngKI;

    move_coordinatev3(target_x,target_y,final_theta,speed,reverse,latKP,ngKP,OlatKP,OngKP,latKD,ngKD,latKI,ngKI,false);

}

void mto::Turn(float speed,float final_theta,bool reverse,float ngKP,float ngKD,float ngKI){

    if(ngKP==-1)ngKP=old_ngKP;

    if(ngKD==-1)ngKD=old_ngKD;

    if(ngKI==-1)ngKI=old_ngKI;

    turn_coordinate(final_theta, reverse,  ngKP,  ngKD,  ngKI,0,speed);

}
