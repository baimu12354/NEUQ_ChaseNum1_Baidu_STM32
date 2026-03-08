#include "TopSpeed_Include.h"


void init_PID_ctrler(PID_ctrler *ctrler,
                    float Kp,
                    float Ki,
                    float Kd,

                    float i_max,
                    float out_max){
    ctrler->first_cal=1;
    ctrler->Kp=Kp;
    ctrler->Ki=Ki;
    ctrler->Kd=Kd;
    ctrler->err=0.0f;
    ctrler->err_last=0.0f;
    ctrler->err_sum=0.0f;
    ctrler->i_max=i_max;
    ctrler->out_max=out_max;
    ctrler->output=0.0f;
}
                        
void PID_ctrler_update(PID_ctrler *ctrler,float err){
    ctrler->err=err;
    ctrler->err_sum+=ctrler->err;
    ctrler->err_sum=ctrler->err_sum>(ctrler->i_max/ctrler->Ki)?(ctrler->i_max/ctrler->Ki):ctrler->err_sum;
    ctrler->err_sum=ctrler->err_sum<(-ctrler->i_max/ctrler->Ki)?(-ctrler->i_max/ctrler->Ki):ctrler->err_sum;

    float p=ctrler->err*ctrler->Kp;
    float i=ctrler->err_sum*ctrler->Ki;
    float d=(ctrler->err-ctrler->err_last)*ctrler->Kd;
    ctrler->err_last=ctrler->err;

    if(ctrler->first_cal==1){//第一次计算没有误差变化率，不计算d
        ctrler->output=p+i;
        ctrler->first_cal=0;
    }
    else{
        ctrler->output=p+i+d;
    }

    if (ctrler->output > ctrler->out_max){
        ctrler->output = ctrler->out_max;
        ctrler->err_sum-=ctrler->err;
    } 
    else if (ctrler->output < -ctrler->out_max){
        ctrler->output = -ctrler->out_max;
        ctrler->err_sum-=ctrler->err;
    }
}

void PID_ctrler_update_with_time(PID_ctrler *ctrler,float err,uint32_t us_time){
    if(us_time==0){
        return;
    }
    float time_factor=(float)us_time/5000.0f;
    ctrler->err=err;
    ctrler->err_sum+=ctrler->err*time_factor;
    ctrler->err_sum=ctrler->err_sum>(ctrler->i_max/ctrler->Ki)?(ctrler->i_max/ctrler->Ki):ctrler->err_sum;
    ctrler->err_sum=ctrler->err_sum<(-ctrler->i_max/ctrler->Ki)?(-ctrler->i_max/ctrler->Ki):ctrler->err_sum;

    float p=ctrler->err*ctrler->Kp;
    float i=ctrler->err_sum*ctrler->Ki;
    float d=(ctrler->err-ctrler->err_last)*ctrler->Kd/time_factor;
    ctrler->err_last=ctrler->err;

    if(ctrler->first_cal==1){//第一次计算没有误差变化率，不计算d
        ctrler->output=p+i;
        ctrler->first_cal=0;
    }
    else{
        ctrler->output=p+i+d;
    }

    if (ctrler->output > ctrler->out_max){
        ctrler->output = ctrler->out_max;
        ctrler->err_sum-=ctrler->err*time_factor;
    } 
    else if (ctrler->output < -ctrler->out_max){
        ctrler->output = -ctrler->out_max;
        ctrler->err_sum-=ctrler->err*time_factor;
    }
}

void PID_ctrler_reset(PID_ctrler *ctrler){
    ctrler->first_cal=1;
    ctrler->err=0.0f;
    ctrler->err_last=0.0f;
    ctrler->err_sum=0.0f;
    ctrler->output=0.0f;
}