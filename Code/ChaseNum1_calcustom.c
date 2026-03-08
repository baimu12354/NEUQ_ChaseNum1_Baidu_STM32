#include "ChaseNum1_calcustom.h"

//**********角度转换函数**********//
float degree_to_rad(float degree){return 0.0174533*degree;}
float rad_to_degree(float rad){return 57.295779513*rad;}
//**********角度转换函数**********//

float get_relative_angle(float now,float aim){//获取两个绝对方位角之间的相对方位角
    float difference=aim-now;
    if(difference<-180)return difference+360;
    if(difference>180)return difference-360;
    return difference;
}