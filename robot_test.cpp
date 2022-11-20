// File:          robot_test.cpp
// Date:          
// Description:   
// Author:        
// Modifications: 

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Motor.hpp>
#include <iostream>

#include <tuple>
#include <string>
#include <webots/Camera.hpp>
#include <webots/Display.hpp>

#include <fstream>
#include <bits/stdc++.h>
#include <cassert>  

#include<opencv2/opencv.hpp>

#define TIME_STEP 32
#define MAX_SPEED 10.0
#define DASH_SPEED 7.5
#define MOSAIC_SPEED 5.0
#define wheel_radius 2.7
#define wheel_base 17.6
#define BALL_COLOUR 2  
//2 for red, 4 for blue


// All the webots classes are defined in the "webots" namespace
using namespace webots;
using namespace std;
using namespace cv;

Mat imageproccMat;

Robot *robot = new Robot();

Camera *camera;
Display *display;

const unsigned char *image;
const unsigned char *flo_image;
int imagewidth;
int imageheight;
Mat  imageMat ;
static Scalar low_margin[6] = {Scalar(10,100,20),Scalar(0, 0,125),Scalar(0,70,50),Scalar(170,70,50),Scalar(115,200,50),Scalar(20,0,0)}; //0 = yel 1 = white 2,3 = red  4 = blue 5 = yell_colorpatch
static Scalar upper_margin[6] = {Scalar(28,255,255),Scalar(180,30,255),Scalar(10,255,255),Scalar(125,255,255),Scalar(130,255,255),Scalar(35,255,255)};


Motor *motor[2];
Motor *linearMotors[2];
Motor *base_servo = robot->getMotor("base_servo");
Motor *kick_linear = robot->getMotor("kick linear");


PositionSensor *right_ps = robot->getPositionSensor("Right_ps");
PositionSensor *left_ps = robot->getPositionSensor("Left_ps");

DistanceSensor *ds[8];
DistanceSensor *sonar[3];
int sensor_readings[8] = {0, 0, 0, 0, 0, 0, 0, 0};
char motorNames[2][15] = {"Left_motor", "Right_motor"};
char linear_MotorNames[2][15] = {"left linear", "right linear"};
char distance_sensors[8][15] = {"IR_sensor_1", "IR_sensor_2", "IR_sensor_3", "IR_sensor_4", "IR_sensor_5", "IR_sensor_6", "IR_sensor_7", "IR_sensor_8"};
char sonar_sensors[3][15]={"left_distance","right_distance","front_distance"};
const double box = 2.0, ball = 2.5, cylinder = 2.0;
double ANGLE = 0 , DISTANCE = 0;

void set_velocity(float left,float right){
        motor[0]->setVelocity(left);
        motor[1]->setVelocity(right);
}

void delay(double seconds){
  double start = double(robot->getTime());
  double current = double(robot->getTime());
  while (start + seconds > current){
    current = double(robot->getTime());
    robot->step(1);
  }
  
}

void blue_get_reading(){
  for (int i=0; i<8; i++){
    if (int(ds[i]->getValue())>900){
     sensor_readings[i] = 0; //if black
    }
    else{
      sensor_readings[i] = 1; //if white
    }
  }
}

float previous_error = 0.0;
float kp = 8.5;
float kd = 0.42;
float ki = 0;
float integral_value = 0.0;
int status = 0;

void get_reading(){
  for (int i=0; i<8; i++){
    if (int(ds[i]->getValue())>512){
     sensor_readings[i] = 0; //if black
    }
    else{
      sensor_readings[i] = 1; //if white
    }
  }
  }

tuple <float,float> PID_con(float cen_x_0){
  float error = 0;
  error += (-cen_x_0 + imagewidth/2) ;
  
  float P = kp*error;
  float I = integral_value + (ki*error);
  float D =  kd*(error-previous_error);
  float correction = (P+I+D)/30;
  float left_speed = MAX_SPEED/2 - correction;
  float right_speed = MAX_SPEED/2 + correction;  
  
 
  if (left_speed>MAX_SPEED){
    left_speed = MAX_SPEED;
  }
  if (right_speed>MAX_SPEED){
    right_speed = MAX_SPEED;
  }
  
  set_velocity(left_speed,right_speed);

  return make_tuple(error,I);
}

int mostFrequent(int arr[], int n){
    sort(arr, arr + n);
    int max_count = 1, res = arr[0], curr_count = 1;
    for (int i = 1; i < n; i++) {
        if (arr[i] == arr[i - 1])
            curr_count++;
        else {
            if (curr_count > max_count) {
                max_count = curr_count;
                res = arr[i - 1];
            }
            curr_count = 1;
        }
    }
    if (curr_count > max_count)
    {
        max_count = curr_count;
        res = arr[n - 1];
    }
    return max_count;
}


tuple <float,float> PID(double speed = 10){
  float error = 0;
  int coefficient[8] = {-16,-9,-4,-1,1,4,9,16};
  
  for (int i=0; i<8; i++){
    error += coefficient[i]*sensor_readings[i];
  }
  
  float P = kp*error;
  float I = integral_value + (ki*error);
  float D =  kd*(error-previous_error);
  float correction = (P+I+D)/30;
  float left_speed = speed/2 - correction;
  float right_speed = speed/2 + correction;  
   
  if (left_speed>speed){
    left_speed = speed;
  }
 
  if (right_speed>speed){
    right_speed = speed;
  }
   
   
  motor[0]->setVelocity(left_speed);
  motor[1]->setVelocity(right_speed);
  
  string string_result = "";
  for (int i=0; i<8; i++){
    string_result += to_string(sensor_readings[i]);
  }
  
  cout << left_speed << "," << right_speed << "," << string_result << endl;
  
  return make_tuple(error,I);
}

void forward(){
  motor[0]->setVelocity(MAX_SPEED / 2);
  motor[1]->setVelocity(MAX_SPEED / 2);
}

void stop(){
  motor[0]->setVelocity(0);
  motor[1]->setVelocity(0);  
}

void go_distance(double distance , double speed = MAX_SPEED){
  double left_start , right_start , right_distance = 0, left_distance = 0, median = 0;
  double encoder_values[2] = {0 , 0};
  
  right_ps->enable(TIME_STEP);
  left_ps->enable(TIME_STEP);

  while (robot->step(TIME_STEP) != -1){
    right_start = right_ps->getValue();
    left_start = left_ps->getValue();
    break;
  }
  motor[0]->setVelocity(speed / 2);
  motor[1]->setVelocity(speed / 2);
 
  

  while ((robot->step(TIME_STEP) != -1) && (median < distance)){
    encoder_values[0] = right_ps->getValue();
    encoder_values[1] = left_ps->getValue();
    right_distance = (encoder_values[0] - right_start) * wheel_radius;
    left_distance = (encoder_values[1] - left_start) * wheel_radius;
    median = (right_distance + left_distance) / 2;
  }
  
  motor[0]->setVelocity(0.0);
  motor[1]->setVelocity(0.0);
  
  right_ps->disable();
  left_ps->disable();
  
}

void go_reverse_distance(double distance , double speed = MAX_SPEED){
  double left_start , right_start , right_distance = 0, left_distance = 0, median = 0;
  double encoder_values[2] = {0 , 0};
  
  right_ps->enable(TIME_STEP);
  left_ps->enable(TIME_STEP);

  while (robot->step(TIME_STEP) != -1){
    right_start = right_ps->getValue();
    left_start = left_ps->getValue();
    break;
  }
  motor[0]->setVelocity(-speed / 2);
  motor[1]->setVelocity(-speed / 2);
 
  

  while ((robot->step(TIME_STEP) != -1) && (median < distance)){
    encoder_values[0] = right_ps->getValue();
    encoder_values[1] = left_ps->getValue();
    right_distance = -(encoder_values[0] - right_start) * wheel_radius;
    left_distance = -(encoder_values[1] - left_start) * wheel_radius;
    median = (right_distance + left_distance) / 2;
  }
  
  motor[0]->setVelocity(0.0);
  motor[1]->setVelocity(0.0);
  
  right_ps->disable();
  left_ps->disable();
  
}

void turn_clockwise(double angle , double speed = MAX_SPEED){
  angle = angle * 3.1416 / 180;
  double left_start = 0 , right_start = 0 , right_angle = 0, left_angle = 0, median = 0;
  double encoder_values[2] = {0 , 0};
  
  right_ps->enable(TIME_STEP);
  left_ps->enable(TIME_STEP);

  while (robot->step(TIME_STEP) != -1){
    left_start = left_ps->getValue();
    right_start = right_ps->getValue();
    break;
  }
  motor[0]->setVelocity(speed / 2);
  motor[1]->setVelocity(-speed / 2);
 
  

  while ((robot->step(TIME_STEP) != -1) && (median < angle)){
    encoder_values[0] = right_ps->getValue();
    encoder_values[1] = left_ps->getValue();
    right_angle = 2 * (encoder_values[0] - right_start) * wheel_radius / wheel_base;
    left_angle = 2 * (encoder_values[1] - left_start) * wheel_radius / wheel_base;
    median = (left_angle - right_angle) / 2;
  }
  
  motor[0]->setVelocity(0.0);
  motor[1]->setVelocity(0.0);
  
  right_ps->disable();
  left_ps->disable();
  
}


void turn_anticlockwise(double angle , double speed = MAX_SPEED){
  angle = angle * 3.1416 / 180;
  double left_start = 0 , right_start = 0 , right_angle = 0, left_angle = 0, median = 0;
  double encoder_values[2] = {0 , 0};
  
  right_ps->enable(TIME_STEP);
  left_ps->enable(TIME_STEP);

  while (robot->step(TIME_STEP) != -1){
    left_start = left_ps->getValue();
    right_start = right_ps->getValue();
    break;
  }
  motor[0]->setVelocity(-speed / 2);
  motor[1]->setVelocity(speed / 2);
 
  

  while ((robot->step(TIME_STEP) != -1) && (median < angle)){
    encoder_values[0] = right_ps->getValue();
    encoder_values[1] = left_ps->getValue();
    right_angle = 2 * (encoder_values[0] - right_start) * wheel_radius / wheel_base;
    left_angle = 2 * (encoder_values[1] - left_start) * wheel_radius / wheel_base;
    median = (right_angle - left_angle) / 2;
  }
  
  motor[0]->setVelocity(0.0);
  motor[1]->setVelocity(0.0);
  
  right_ps->disable();
  left_ps->disable();
  
  
}

void line_following(){

  while (robot->step(TIME_STEP) != -1) {
      get_reading();
      int sum = 0;
      for (int i=0;i<8;i++){
        sum += sensor_readings[i];
        }
  
      if (sum == 0){
        break;
        }
      tie(previous_error,integral_value) = PID();
  }

}


void dashed_line_following(){

  while (robot->step(TIME_STEP) != -1) {
      get_reading();
      if (sensor_readings[0] == 0 && sensor_readings[1] == 0 && sensor_readings[2] == 1 && sensor_readings[3] == 1 && sensor_readings[4] == 1 && sensor_readings[5] == 1 && sensor_readings[6] == 1 && sensor_readings[7] == 1){
        break;
        }

      tie(previous_error,integral_value) = PID(DASH_SPEED);
      }
}

void wall_following(){
  double previous_error, error = 0, threshold = 945 , correction = 0 , difference = 0;
  double kp = 1.0 , kd = 0.5;
  double right , left, front;
  double right_value, left_value;
  int state = 0;
  get_reading();
  while (robot->step(TIME_STEP) != -1){
    right = sonar[1]->getValue();
    left = sonar[0]->getValue();
    front = sonar[2]->getValue();
    
    //std::cout << front << " " << left << " " << right << std::endl;
    
    if (left > 750){
      if (front < 990){
        error = (left - threshold);
        difference = error - previous_error;
        //cout << difference << endl;
        correction = kp * error + kd * difference;
        
        right_value = MAX_SPEED / 2 - correction;
        left_value = MAX_SPEED / 2 + correction;
        
        if (right_value > MAX_SPEED) right_value = MAX_SPEED;
        if (right_value <0.0) right_value = 0.0;
        if (left_value > MAX_SPEED) left_value = MAX_SPEED;
        if (left_value < 0.0) left_value = 0.0;  
        motor[0]->setVelocity(left_value);
        motor[1]->setVelocity(right_value);
        state = 0;
        previous_error = error;
      }
      else{
        if (right > 750){
          turn_clockwise(180 , MAX_SPEED);
          state = 0;
        }
        else{
          turn_clockwise(90 , MAX_SPEED);
          go_distance(30 , MAX_SPEED);
          state = 1;
        }        
      }
    }
    else{
      if (front > 990){
        turn_anticlockwise(90 , MAX_SPEED);
        go_distance(30 , MAX_SPEED);
        state = 1;
      }
      else if (front > 740){
        forward();
        state = 0;
      }
      else if (state == 1){
        go_distance(5 , MAX_SPEED);
        turn_anticlockwise(90 , MAX_SPEED);
        go_distance(30 , MAX_SPEED);
        state = 1;
      }
      else{
        go_distance(30 , MAX_SPEED);
        turn_anticlockwise(90 , MAX_SPEED);
        go_distance(30 , MAX_SPEED);
        state = 1;
      }      
    } 
    get_reading();
    int sum = 0;
    for (int i=0;i<8;i++){
      sum += sensor_readings[i];
    }
  
    if (sum == 8){
        motor[0]->setVelocity(0.0);
        motor[1]->setVelocity(0.0);
        break;
    }     
  }  
}



void move_down(){
  base_servo->setVelocity(-1.0);
  delay(3.1);
  base_servo->setVelocity(0.0);
}

void move_up(){
  base_servo->setVelocity(1.0);
  delay(3.1);
  base_servo->setVelocity(0.0);  
}

void gripper_close(double seconds, double final_position){
  linearMotors[1]->setVelocity(0.01);
  linearMotors[0]->setVelocity(-0.01);
  delay(seconds);
  linearMotors[1]->setPosition(final_position); //0.03 for ball, 0.025 for box
  linearMotors[0]->setPosition(-final_position);  //-0.03 for ball, -0.025 for box
}

void gripper_open(double seconds, double start_position){
  linearMotors[1]->setVelocity(-0.01);
  linearMotors[0]->setVelocity(0.01);
  delay(seconds);
  linearMotors[1]->setPosition(start_position);
  linearMotors[0]->setPosition(-start_position);
}

void push_state(){
  move_down();
  delay(1);
  gripper_close(box, 0.045);
  //delay(1);
  //move_up();
}

void fetch_box(){
  move_down();
  delay(1);
  gripper_close(box, 0.025);
  delay(3);
  move_up();
}

void release_box(){
  move_down();
  delay(1);
  gripper_open(box, 0.00595);
  delay(3);
  move_up();
}

void fetch_ball(){
  move_down();
  delay(1);
  gripper_close(ball, 0.03);
  delay(3);
  move_up();
}

void release_ball(){
  move_down();
  delay(1);
  gripper_open(ball, 0.00595);
  delay(3);
  move_up();
}

void fetch_cylinder(){
  move_down();
  delay(1);
  gripper_close(cylinder, 0.025);
  delay(3);
  move_up();
}

void release_cylinder(){
  move_down();
  delay(1);
  gripper_open(cylinder, 0.00594);
  delay(3);
  move_up();
}

void Kicker_Push(double seconds){
  kick_linear->setVelocity(0.01);
  delay(seconds);
  kick_linear->setVelocity(0.0);
}

void colourline_following(){
  //go_distance(10 , MAX_SPEED);
  forward();
  delay(1.6);
  stop();
  if (BALL_COLOUR == 0) turn_anticlockwise(90 , MAX_SPEED);
  while (robot->step(TIME_STEP) != -1) {
    if (BALL_COLOUR == 0) get_reading();
    else blue_get_reading();
    
    int sum = 0;
    for (int i=0;i<8;i++){
      sum += sensor_readings[i];
      }
  
    if (sum == 8){
      stop();
      cout<<"Sum "<<sum<<endl;
      break;
      }
    tie(previous_error,integral_value) = PID();
  }
}

Mat color_filter(const unsigned char *image,int color){
  //cout<<"color_fil"<<endl;
  imageMat.data = (uchar *)image;
  Mat bgr_image;
  Mat rgb_image;
  Mat hsv;
  Mat req_img;
  Mat blur_bilateral;
  Mat thresh_otsu;
    
  cvtColor(imageMat,rgb_image,COLOR_BGRA2RGB);
  cvtColor(rgb_image,bgr_image,COLOR_RGB2BGR);
  cvtColor(bgr_image,hsv,COLOR_BGR2HSV);
  if (color == 2 ){ //|| color == 4
    Mat int1,int2;
    inRange(hsv,low_margin[color],upper_margin[color],int1); //(20,100,100) (30,255,255)
    inRange(hsv,low_margin[color+1],upper_margin[color+1],int2); 
    req_img = int1 + int2;
  }else{
  inRange(hsv,low_margin[color],upper_margin[color],req_img);
  }
  bilateralFilter(req_img,blur_bilateral,9,75,75);
  threshold(blur_bilateral,thresh_otsu,0,255,THRESH_OTSU);
  //cout<<"end colo"<<endl;
  return thresh_otsu;
}

void tracker(vector<vector<Point>> contours){
       int length = contours.size();
       float area[length];
       
       vector<Moments> mu(length);
       vector<Rect> boundingRectangles(length);
       vector<vector<Point>> contour_poly(length);
       float cx[length];
       float cy[length];
      
       for (int i = 0 ; i <length; i++){
          mu[i] = moments(contours[i]);
          cx[i] = static_cast<float>(mu[i].m10 / (mu[i].m00 + 1e-5 ));
          cy[i] = static_cast<float>(mu[i].m01 / (mu[i].m00 + 1e-5 ));
          //cout <<" area "  << contourArea(contours[i],false)<<endl;
          // if  ((contourArea(contours[i],false)<20) && (contourArea(contours[i],false)>0)){
             // for (int k = 0 ; k < 2 ; k++){
                // motor[k]->setVelocity(MAX_SPEED);
            // }
             // double seconds = 2 ;
             // double start = double(robot->getTime());
              // double current = double(robot->getTime());
              // while (start + seconds > current){
                // current = double(robot->getTime());
                // robot->step(1);
              // }
          // }
          float perimeter = arcLength(contours[i],true);
          approxPolyDP(contours[i], contour_poly[i], 0.02*perimeter,true);
          boundingRectangles[i] = boundingRect(contour_poly[i]);
          //cout << "cx" << cx[i] <<"," << cy[i]<<"height"<<boundingRectangles[i].height/2 <<endl;
          if (cy[i]+boundingRectangles[i].height/2 >imageheight){
            //cout << "outof" << endl;
            for (int k = 0 ; k < 2 ; k++){
                motor[k]->setVelocity(0.0);
            }
            return;
          }
       }
       tie(previous_error,integral_value) = PID_con(cx[0]); 
}

int obj_select(const unsigned char *image){
       //cout<<"obj_s"<<endl;
       while ((robot->step(TIME_STEP) != -1)  ){
      image = camera->getImage();
      imageMat.data = (uchar *)image;
      Mat obj_image = color_filter(image,0);
       vector<vector<Point>> contours;
       vector<Vec4i> hierarchy;
              
       erode(obj_image, obj_image, getStructuringElement(MORPH_ELLIPSE, Size(3, 3)));
       dilate(obj_image, obj_image, getStructuringElement(MORPH_ELLIPSE, Size(3, 3)));

       // findContours(thresholded, contours, heirarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

       // conPoly.resize(contours.size());
       findContours(obj_image,contours,hierarchy,RETR_TREE,CHAIN_APPROX_NONE);
       //conPoly.resize(contours.size());
       
       Mat imagecopy = imageMat.clone();
       
        int length = contours.size();
        float area[length];
        int out = -1;
     
        vector<Moments> mu(length);
        vector<Rect> boundingRectangles(length);
        vector<vector<Point>> contour_poly(length);
        float cx[length];
        float cy[length];
        
        for (int i = 0 ; i <length; i++){
          int l_cont_size = contours[i].size();
          int arr[l_cont_size];
          for (int j = 0 ; j <l_cont_size; j++){
            arr[j] = contours[i][j].x ;            
          }

          area[i] = contourArea(contours[i],false);
          mu[i] = moments(contours[i]);
          cx[i] = static_cast<float>(mu[i].m10 / (mu[i].m00 + 1e-5 ));
          cy[i] = static_cast<float>(mu[i].m01 / (mu[i].m00 + 1e-5 ));
                              
          float perimeter = arcLength(contours[i],true);
          approxPolyDP(contours[i], contour_poly[i], 0.02*perimeter,true);
          boundingRectangles[i] = boundingRect(contour_poly[i]);
          if (cy[i]+boundingRectangles[i].height/2 >63.0){
            //cout << "outof" << endl;
            for (int k = 0 ; k < 2 ; k++){
                motor[k]->setVelocity(0.0);
            }
            int n = sizeof(arr) / sizeof(arr[0]);
            int max_count = mostFrequent(arr,n);              
            //cout <<" area "  << area[i]<<endl;
            //cout << "cx "<<i<<":" << cx[i] <<endl;
            //cout << "cy "<<i<<":" << cy[i] <<endl;
            //cout <<"om"<< max_count << endl;
            //cout << "size" << l_cont_size << endl;
            //cout << "peri" << perimeter << endl;
            //cout << boundingRectangles[i].width << " height " << boundingRectangles[i].height << endl;
            //double seconds = 8 ;            
            // if ((perimeter <87 && perimeter > 81)||(perimeter <77.5) ){                           
              // if (max_count >boundingRectangles[i].height*0.75){
                // cout << "cube" << endl;
              // }else{
                // cout << "cylinder" << endl;
              // }
             // }
             // else{
                // set_velocity(-MAX_SPEED/10,-MAX_SPEED);
                // seconds = 2 ;
             // }
             
             if (max_count<9){
             out = 1;
             }else {out = 0;}
             
             // float A = area[i];
             
              // if ( A < 280){
                // out = 1;  //1 for cylinder
              // }
              // else if (280<A && A<330){
                // if (max_count > boundingRectangles[i].height*0.65){
                // out = 0; 
                // }
                // else{
                // out =1;}
              // }
              // else if(410<A && A<420){
                // out = 0;
              // }
              // else if(A>350) {
                // if (max_count > boundingRectangles[i].height*0.75){
                // out = 0; 
                // }
                // else{
                // out =1;}
              // }       
              // else{
              // if (max_count > boundingRectangles[i].height*0.6){
                // out = 0;                          
              // }else{out = 1;}
              // }return out; 
                   
              //cout << "fg"<<out <<endl;             
              /*double start = double(robot->getTime());
              double current = double(robot->getTime());
              while (start + seconds > current){
                current = double(robot->getTime());
                robot->step(1);
              }*/ 
          }
        }                
        tie(previous_error,integral_value) = PID_con(cx[0]);
              drawContours(imagecopy,contours,-1,Scalar(0,255,0),0.6);
       cvtColor(imagecopy,imageproccMat,COLOR_BGR2BGRA);  
       ImageRef *iref = display->imageNew(imagewidth,imageheight,imageproccMat.data,Display::BGRA);
        display->imagePaste(iref,0,0,false);
        display->imageDelete(iref);
       return out;  
       //cout<<"ball_e"<<endl;

      //cout<<"obj_e"<<endl;
      }
}


void ball_detect(const unsigned char *image,int color){
while ((robot->step(TIME_STEP) != -1)  ){
       cout<<"ball_s"<<endl;
       Mat ball_image = color_filter(image,color);
       vector<vector<Point>> contours;
       vector<Vec4i> hierarchy;
       findContours(ball_image,contours,hierarchy,RETR_TREE,CHAIN_APPROX_NONE);
       Mat imagecopy = imageMat.clone();
       cout<<"cs"<<contours.size()<<endl;
       int length = contours.size();
       vector<Moments> mu(length);
       vector<Rect> boundingRectangles(length);
        vector<vector<Point>> contour_poly(length);
       float cy[length];
       for (int i = 0 ; i <length; i++){
          mu[i] = moments(contours[i]);
          float perimeter = arcLength(contours[i],true);
          approxPolyDP(contours[i], contour_poly[i], 0.02*perimeter,true);
          cy[i] = static_cast<float>(mu[i].m01 / (mu[i].m00 + 1e-5 ));
          boundingRectangles[i] = boundingRect(contour_poly[i]);
          if (cy[i]+boundingRectangles[i].height/2 >53.0){
          return;
          }
       }
       for (int i = 0 ; i <length; i++){
       cout<<contourArea(contours[i],false)<<endl;
           if (contourArea(contours[i],false)>1000){
           contours.erase(contours.begin()+i,contours.begin()+i+1);
           }
       }
       
       tracker(contours);
             
       drawContours(imagecopy,contours,-1,Scalar(0,255,0),0.6);
       cvtColor(imagecopy,imageproccMat,COLOR_BGR2BGRA);    
       cout<<"ball_e"<<endl;
       }
}

void find_object(int color){
  right_ps->enable(TIME_STEP);
  left_ps->enable(TIME_STEP);
  
  double left_start , right_start , left_end , right_end , right_angle , left_angle;
  
  
  while (robot->step(TIME_STEP) != -1){
    right_start = right_ps->getValue();
    left_start = left_ps->getValue();
    break;
  }
  if (color == 0){
      motor[0]->setVelocity(0.5);
      motor[1]->setVelocity(-0.5);
  }
  else{
    motor[0]->setVelocity(-0.5);
    motor[1]->setVelocity(0.5);
  }

  //cout<<"find_obj_s"<<endl;

    while (robot->step(TIME_STEP) != -1){
      image = camera->getImage();
      imageMat.data = (uchar *)image;
      Mat obj_image = color_filter(image,color);
      
      
      vector<vector<Point>> contours;
      vector<Vec4i> hierarchy;
      findContours(obj_image,contours,hierarchy,RETR_TREE,CHAIN_APPROX_NONE);
      Mat imagecopy = imageMat.clone();
      //cout<<"cs"<<contours.size()<<endl;
      int length = contours.size();
        
      if (length > 0 ){
        vector<Moments> mu(length);
        float cx[length];
        float area[length];
        for (int i = 0 ; i <length; i++){
          area[i] = contourArea(contours[i],false);
          mu[i] = moments(contours[i]);
          cx[i] = static_cast<float>(mu[i].m10 / (mu[i].m00 + 1e-5 ));
        }
        
        // mu[0] = moments(contours[0]);
        // float cx = static_cast<float>(mu.m10 / (mu.m00 + 1e-5 ));
        //cout<<"lk"<<endl;
       drawContours(imagecopy,contours,-1,Scalar(0,255,0),0.6);
       cvtColor(imagecopy,imageproccMat,COLOR_BGR2BGRA);  
       ImageRef *iref = display->imageNew(imagewidth,imageheight,imageproccMat.data,Display::BGRA);
        display->imagePaste(iref,0,0,false);
        display->imageDelete(iref);
        cout << cx[0] <<endl;
        if ((cx[0] <34) && (cx[0] > 32)){
          motor[0]->setVelocity(0);
          motor[1]->setVelocity(0);
          
          while (robot->step(TIME_STEP) != -1){
            right_end = right_ps->getValue();
            left_end = left_ps->getValue();
            break;
          }
          right_angle = 2 * (right_end - right_start) * wheel_radius / wheel_base;
          left_angle = 2 * (left_end - left_start) * wheel_radius / wheel_base;
          ANGLE = 90 * (left_angle - right_angle) / 3.1416;
          if (ANGLE < 0) ANGLE = -ANGLE;
          break;
        }
       
       }
    }
      
}


void insert_cylinder(){
  go_reverse_distance(7.5 , MOSAIC_SPEED);
  delay(0.5);
  turn_anticlockwise(90 , MOSAIC_SPEED);
  go_distance(53 , MOSAIC_SPEED);
  release_cylinder();
  go_reverse_distance(5 , MOSAIC_SPEED);
  push_state();
  delay(3.5);
  go_distance(5 ,  MOSAIC_SPEED);
  go_reverse_distance(53 ,  MOSAIC_SPEED);
  gripper_open(0, 0.00595);
  move_up();
  delay(3);
  turn_clockwise(90 ,  MOSAIC_SPEED);
  go_distance(7.5 , MOSAIC_SPEED);
}

void insert_cube(){
  go_reverse_distance(27.5 , MOSAIC_SPEED);
  delay(0.5);
  turn_anticlockwise(90 , MOSAIC_SPEED);
  go_distance(53 , MOSAIC_SPEED);
  release_box();
  go_reverse_distance(5 , MOSAIC_SPEED);
  push_state();
  delay(3.5);
  go_distance(5 ,  MOSAIC_SPEED);
  go_reverse_distance(53 ,  MOSAIC_SPEED);
  gripper_open(0, 0.00595);
  move_up();
  delay(3);
  turn_clockwise(90 ,  MOSAIC_SPEED);
  go_distance(27.5 , MOSAIC_SPEED); 
}

void mosaic_floor(int mosaic_status){
  //cout<<"mscflr"<<endl;
  image = camera->getImage();
  double left_start, right_start, right_end, left_end, right_distance, left_distance;
  if (image){        
        //cout<<"img ok"<<endl;
        if (mosaic_status == 0){
        find_object(0);
        int obj = -1;
    
  
  right_ps->enable(TIME_STEP);
  left_ps->enable(TIME_STEP);

  while (robot->step(TIME_STEP) != -1){
    right_start = right_ps->getValue();
    left_start = left_ps->getValue();
    break;
  }      
  while (obj == -1){
    obj = obj_select(image);}  //0 for cube 1 for cylinder
 
  while (robot->step(TIME_STEP) != -1){
    right_end = right_ps->getValue();
    left_end = left_ps->getValue();
    break;
  }
  right_distance = (right_end - right_start) * wheel_radius;
  left_distance = (left_end - left_start) * wheel_radius;
  DISTANCE = (right_distance + left_distance) / 2;
  right_ps->disable();
  left_ps->disable();  
  if (obj == 0){
    go_distance(23.5 , MOSAIC_SPEED);
    DISTANCE += 23.5;
    fetch_box();    
  }
  else{
    go_distance(20 , MOSAIC_SPEED);
    DISTANCE += 20;
    fetch_cylinder();  
  }
  go_reverse_distance(DISTANCE , MOSAIC_SPEED);
  turn_anticlockwise(ANGLE , MOSAIC_SPEED);
  delay(1);
  
  if (obj == 0) insert_cube();
  else insert_cylinder();
                
        }
        else if(mosaic_status == 1){
        
          
          find_object(BALL_COLOUR);
          right_ps->enable(TIME_STEP);
          left_ps->enable(TIME_STEP);
  
          while (robot->step(TIME_STEP) != -1){
            right_start = right_ps->getValue();
            left_start = left_ps->getValue();
            break;
          }   
          
          ball_detect( image,BALL_COLOUR); //2 for red 4 blue
          
          while (robot->step(TIME_STEP) != -1){
            right_end = right_ps->getValue();
            left_end = left_ps->getValue();
            break;
          }
          right_distance = (right_end - right_start) * wheel_radius;
          left_distance = (left_end - left_start) * wheel_radius;
          DISTANCE = (right_distance + left_distance) / 2;
          right_ps->disable();
          left_ps->disable();
          go_distance(38, MOSAIC_SPEED);
          DISTANCE += 38;
          fetch_ball();
          go_reverse_distance(DISTANCE , MOSAIC_SPEED);
          turn_clockwise(ANGLE , MOSAIC_SPEED);
          delay(1);
     }
        //key_hole_find(image);
        // int color = color_detector(flo_image,imagewidthf,imageheightf);
        // cout << "color" << color<<endl;
        //slot_detect();
        ImageRef *iref = display->imageNew(imagewidth,imageheight,imageproccMat.data,Display::BGRA);
        display->imagePaste(iref,0,0,false);
        display->imageDelete(iref);
          //End of the contour code
    }
}

void come_middle(void){
  go_distance(30 , MOSAIC_SPEED);
  delay(0.5);
  turn_anticlockwise(90 , MOSAIC_SPEED);
  delay(0.5);
  go_distance(75 , MOSAIC_SPEED);
  delay(0.5);
  turn_clockwise(90 , MOSAIC_SPEED);
  delay(0.5);
  go_distance(35 , MOSAIC_SPEED);
}

void maze_out(){
  int sum;
  go_distance(33 , MOSAIC_SPEED);
  turn_anticlockwise(90 , MOSAIC_SPEED);
  go_distance(20 , MOSAIC_SPEED);
  set_velocity(MOSAIC_SPEED/2 , MOSAIC_SPEED / 2);

  while (robot->step(TIME_STEP) != -1){
    get_reading();
    sum = 0;
    for (int i=0;i<8;i++){
       sum += sensor_readings[i];
    }
    cout<<sum<<endl;
    if (sum == 8) break;
  }
  go_distance(5 , MOSAIC_SPEED);
  stop();
}
// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char **argv) {
  // create the Robot instance.
  for (int i=0; i<2; i++){
    motor[i] = robot->getMotor(motorNames[i]);
    motor[i]->setPosition(INFINITY);
    motor[i]->setVelocity(0.0);
    }

  for (int i=0; i<8; i++){
    ds[i] = robot->getDistanceSensor(distance_sensors[i]);
    ds[i]->enable(TIME_STEP);
    }

  for (int i=0; i<3; i++){
    sonar[i] = robot->getDistanceSensor(sonar_sensors[i]);
    sonar[i]->enable(TIME_STEP);
    }
  for (int i=0; i<2; i++){
    linearMotors[i] = robot->getMotor(linear_MotorNames[i]);
    linearMotors[i]->setPosition(INFINITY);
    linearMotors[i]->setVelocity(0.0);
    } 
          
  base_servo->setPosition(INFINITY);
  base_servo->setVelocity(0.0);
  
  
  kick_linear->setPosition(INFINITY);
  kick_linear->setVelocity(0.0);
  
  camera = robot->getCamera("camera");
  camera->enable(TIME_STEP);
  display = robot->getDisplay("display");
  
  // floor_camera = robot->getCamera("floor_camera");
  // floor_camera->enable(TIME_STEP);

  imagewidth = camera->getWidth();
  imageheight = camera->getHeight();
  // int imagewidthf = floor_camera->getWidth();
  // int imageheightf = floor_camera->getHeight();
  
  imageMat = Mat(Size(imagewidth,imageheight),CV_8UC4);
  //line_following();
   
  while (robot->step(TIME_STEP) != -1) {
    line_following();
    wall_following();
    come_middle();
    mosaic_floor(0);
    mosaic_floor(0);
    mosaic_floor(1);
    maze_out();
    dashed_line_following();
    colourline_following();
    release_ball();
    
    // Read the sensors:
    // Enter here functions to read sensor data, like:
    //  double val = ds->getValue();
    // Process sensor data here.  
    // Enter here functions to send actuator commands, like:
    //right();
  // Enter here exit cleanup code.
      
  
  }
  delete robot;
  return 0;
}

