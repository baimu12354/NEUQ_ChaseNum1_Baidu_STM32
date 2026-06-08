#include "ChaseNum1_Include.h"
#include <math.h>

Car_Movement_struct Whole_Movement = {0};                  // 当前底盘运动命令
Movement_type_ENUM Whole_Movement_Type = Movement_Breaking; // 当前底盘运动模式

PID_ctrler Car_yaw_PID_ctrler = {0};               // 车身姿态角度环
PID_ctrler Car_yaw_moving_PID_ctrler = {0};        // 平移姿态保持预留 PID

/****************************通用工具与状态入口***************************/
static float CN_Chassis_Limit_Float(float value, float max_abs);
static float CN_Chassis_Approach_Float(float now, float target, float step);
static void CN_Chassis_Reset_Wheel_Pid(void);
static float CN_Chassis_Max_Wheel_Vel(void);
static void CN_Chassis_Reset_Speed_Ref(float vx, float vy);
static void CN_Chassis_Enter_Run(Car_Movement_struct movement, uint8_t keep_yaw_aim);
static void CN_Chassis_Enter_Rotation(Car_Movement_struct movement);
static void CN_Chassis_Enter_Translation(Car_Movement_struct movement);
static void CN_Chassis_Finish_To_Run(float yaw_hold);

// 浮点限幅，max_abs 必须为正数。
static float CN_Chassis_Limit_Float(float value, float max_abs)
{
    if (value > max_abs) return max_abs;
    if (value < -max_abs) return -max_abs;
    return value;
}

// 让 now 按最大步长 step 平滑逼近 target。
static float CN_Chassis_Approach_Float(float now, float target, float step)
{
    float err = target - now;

    if (step <= 0.0f) return target;
    if (err > step) return now + step;
    if (err < -step) return now - step;
    return target;
}

// 清空四个轮子的速度环，避免模式切换后沿用上一段运动的输出残留。
static void CN_Chassis_Reset_Wheel_Pid(void)
{
    PID_ctrler_reset(&FL_vel_PID_ctrler);
    PID_ctrler_reset(&FR_vel_PID_ctrler);
    PID_ctrler_reset(&BL_vel_PID_ctrler);
    PID_ctrler_reset(&BR_vel_PID_ctrler);
}

// 获取当前四个轮子速度绝对值中的最大值。
static float CN_Chassis_Max_Wheel_Vel(void)
{
    float max_vel = fabsf(Vel_now.M_FL_Vel);
    float abs_vel = fabsf(Vel_now.M_FR_Vel);

    if (abs_vel > max_vel) max_vel = abs_vel;
    abs_vel = fabsf(Vel_now.M_BL_Vel);
    if (abs_vel > max_vel) max_vel = abs_vel;
    abs_vel = fabsf(Vel_now.M_BR_Vel);
    if (abs_vel > max_vel) max_vel = abs_vel;

    return max_vel;
}

/****************************闭环刹车参数与变量***************************/
// 置 1 启用闭环刹车：刹车瞬间先保持车身朝向并将四轮目标速度压到 0。
#define CHASSIS_BRAKE_CLOSED_LOOP_ENABLE   1U
// 认为单个轮子已经静止的编码器速度阈值，单位为每次速度环回调的编码器增量。
#define CHASSIS_BRAKE_STILL_VEL_EPS        1.0f
// 四个轮子连续低于静止阈值的确认时间，单位 ms。
#define CHASSIS_BRAKE_STILL_CONFIRM_MS     100U
// 闭环刹车最长保持时间，超时后即使编码器仍有抖动也会放开控制，单位 ms。
#define CHASSIS_BRAKE_MAX_HOLD_MS          1000U

static uint8_t g_chassis_brake_closed_loop_active = 0U; // 是否正在闭环抱死轮子
static uint32_t g_chassis_brake_elapsed_ms = 0U;        // 闭环刹车已持续时间
static uint32_t g_chassis_brake_still_ms = 0U;          // 轮速连续满足静止条件的时间

static void CN_Chassis_Brake_Start(void);
static void CN_Chassis_Brake_Cancel(void);
static void CN_Chassis_Brake_Update(uint32_t ms);

// 启动闭环刹车：先保持当前车身朝向，再用四轮零速闭环把车压停。
static void CN_Chassis_Brake_Start(void)
{
#if CHASSIS_BRAKE_CLOSED_LOOP_ENABLE
    g_chassis_brake_closed_loop_active = 1U;
    g_chassis_brake_elapsed_ms = 0U;
    g_chassis_brake_still_ms = 0U;

    YAW_aim = YAW_now;
    CN_CAR_vel_CL_set_aim(0.0f, 0.0f);
    CN_Chassis_Reset_Speed_Ref(0.0f, 0.0f);
    CN_CAR_rotation_set_aim(0.0f);
    CN_CAR_anv_CL_set_aim(0.0f);

    PID_ctrler_reset(&Car_yaw_PID_ctrler);
    CN_Chassis_Reset_Wheel_Pid();
#else
    CN_Intr_Movement();
#endif
}

// 取消闭环刹车，新的运动命令会重新接管底盘。
static void CN_Chassis_Brake_Cancel(void)
{
    g_chassis_brake_closed_loop_active = 0U;
    g_chassis_brake_elapsed_ms = 0U;
    g_chassis_brake_still_ms = 0U;
    CN_Chassis_Reset_Wheel_Pid();
}

// 更新闭环刹车保持时间，达到静止确认或超时后放开电机控制。
static void CN_Chassis_Brake_Update(uint32_t ms)
{
#if CHASSIS_BRAKE_CLOSED_LOOP_ENABLE
    if (g_chassis_brake_closed_loop_active == 0U) return;
    if (ms == 0U) ms = 1U;

    g_chassis_brake_elapsed_ms += ms;

    if (CN_Chassis_Max_Wheel_Vel() <= CHASSIS_BRAKE_STILL_VEL_EPS) {
        g_chassis_brake_still_ms += ms;
    } else {
        g_chassis_brake_still_ms = 0U;
    }

    if ((g_chassis_brake_still_ms >= CHASSIS_BRAKE_STILL_CONFIRM_MS) ||
        (g_chassis_brake_elapsed_ms >= CHASSIS_BRAKE_MAX_HOLD_MS)) {
        g_chassis_brake_closed_loop_active = 0U;
        g_chassis_brake_elapsed_ms = 0U;
        g_chassis_brake_still_ms = 0U;
        CN_Intr_Movement();
        CN_Chassis_Reset_Wheel_Pid();
    }
#else
    (void)ms;
#endif
}

/****************************任意角度旋转参数与变量***************************/
// 旋转完成角度误差阈值，单位 deg。
#define CHASSIS_ROTATION_ANGLE_TOL_DEG   1.0f
// 距离目标角度小于该值时开始线性减速，单位 deg。
#define CHASSIS_ROTATION_SLOW_ANGLE_DEG  20.0f
// 低速段最小角速度，单位 deg/s，避免目标太小导致车身贴着死区一卡一卡。
#define CHASSIS_ROTATION_MIN_ANGVEL      1.0f  
// 上位机未给旋转速度或速度太小时使用的默认角速度，单位 deg/s。
#define CHASSIS_ROTATION_DEFAULT_ANGVEL  3.0f
// 允许的最大旋转角速度，单位 deg/s，防止上位机误发过大角速度导致输出突变。
#define CHASSIS_ROTATION_MAX_ANGVEL      10.0f

float target_yaw = 0.0f;                    // 目标相对旋转角度，可大于 180 deg
float YAW_inital = 0.0f;                    // 旋转开始时的角度
float YAW_gain = 0.0f;                      // 已累计旋转角度
float YAW_pre = 0.0f;                       // 上一次角度

static float g_chassis_rotation_target_abs_yaw = 0.0f; // 旋转完成后的姿态目标
static float g_chassis_rotation_max_angvel = 0.0f;     // 旋转最大角速度

/****************************定点定向平移参数与变量***************************/
// 平移目标距离小于该值时认为到达，单位与编码器合成位移一致。
#define CHASSIS_TRANSLATION_POS_TOL          100.0f
// 平移到达后轮速低于该值才确认完成。
#define CHASSIS_TRANSLATION_REACH_VEL_EPS    1.0f
// 平移到达条件连续满足的确认时间，单位 ms。
#define CHASSIS_TRANSLATION_REACH_CONFIRM_MS 50U
// 距离目标小于该值时开始减速（单位为内部编码器合成位移：2500 约等于上位机 50cm）
#define CHASSIS_TRANSLATION_SLOW_DISTANCE    2500.0f
// 平移速度每秒最大变化量，单位与 Car_translation_vel 一致。
#define CHASSIS_TRANSLATION_ACC              150.0f
// 低速段最小运动速度，用于克服轮子死区；进入目标容差后不再使用。
#define CHASSIS_TRANSLATION_MIN_MOVE_SPEED   0.0f
// 上位机未给速度或速度太小时使用的默认平移速度。
#define CHASSIS_TRANSLATION_DEFAULT_SPEED    0.0f
// 允许的最大平移速度。
#define CHASSIS_TRANSLATION_MAX_SPEED        20.0f

float initial_yaw = 0.0f;                    // 平移开始时锁定的车身姿态
float distance_to_go = 0.0f;                 // 沿目标直线方向的剩余距离
Pos_struct target_pose = {0};                // 相对目标点
Pos_struct start_pose = {0};                 // 平移起点，固定为当前点
float cur_x = 0.0f;                          // 编码器累计合成 X 位移
float cur_y = 0.0f;                          // 编码器累计合成 Y 位移

static float g_chassis_translation_total_distance = 0.0f; // 目标总距离
static float g_chassis_translation_dir_x = 0.0f;          // 目标方向单位向量 X
static float g_chassis_translation_dir_y = 0.0f;          // 目标方向单位向量 Y
static float g_chassis_translation_max_speed = 0.0f;      // 平移最大速度
static float g_chassis_translation_speed_ref = 0.0f;      // 平滑后的当前平移速度
static uint32_t g_chassis_translation_reach_ms = 0U;      // 到达确认计时
static uint8_t g_chassis_translation_reached = 0U;        // 平移是否完成

/****************************运动功能函数***************************/
// 运动初始化
void CN_move_init(void)
{
    CN_CAR_vel_CL_init();       // 车身速度环初始化
    CN_CAR_yaw_CL_init();       // 车身角度环初始化
    CN_CAR_anv_CL_init();       //+旋转分量初始化，当前不再使用角速度闭环。
    CN_CAR_pos_CL_init();       // 定点平移初始化
    YAW_aim = YAW_now;
    Whole_Movement_Type = Movement_Breaking;
}

// 设置目标和运动模式；协议层仍然调用这个函数，不需要新增接口。
void CN_set_movement(Car_Movement_struct movement, Movement_type_ENUM movement_type)
{
    Movement_type_ENUM last_type = Whole_Movement_Type;

    Whole_Movement = movement;

    if (movement_type == Movement_Breaking) {
        Whole_Movement_Type = Movement_Breaking;
        CN_Chassis_Brake_Start();
        return;
    }

    CN_Chassis_Brake_Cancel();

    switch (movement_type) {
        case Movement_Running: {
            uint8_t keep_yaw = (last_type == Movement_Running) ? 1U : 0U;
            CN_Chassis_Enter_Run(movement, keep_yaw);
            break;
        }
        case Movement_Rotation: {
            CN_Chassis_Enter_Rotation(movement);
            break;
        }
        case Movement_Translation: {
            CN_Chassis_Enter_Translation(movement);
            break;
        }
        default: {
            break;
        }
    }
}

// 运动停止：直接清空速度目标和 PWM 输出，用于 break 完成后的放开控制。
void CN_Intr_Movement(void)
{
    CN_CAR_vel_CL_set_aim(0.0f, 0.0f);
    CN_Chassis_Reset_Speed_Ref(0.0f, 0.0f);
    CN_CAR_anv_CL_set_aim(0.0f);
    CN_CAR_rotation_set_aim(0.0f);

    Duty_now.M_FL_Duty = 0.0f;
    Duty_now.M_FR_Duty = 0.0f;
    Duty_now.M_BL_Duty = 0.0f;
    Duty_now.M_BR_Duty = 0.0f;
    CN_set_Motor_Duty();
}

//+读取当前底盘运动模式，供通信状态机判断一次性动作是否完成。
Movement_type_ENUM CN_Chassis_Get_Movement_Type(void)
{
    return Whole_Movement_Type;
}

// 运动回调函数，建议 1ms 调用一次。
void CN_Movement_ctrl_CALLBACK(uint32_t ms)
{
    static uint32_t time = 0U;

    time += ms;
    if (time > 1000U) time = 0U;

    switch (Whole_Movement_Type) {
        case Movement_Breaking: {
            if (g_chassis_brake_closed_loop_active != 0U) {
                CN_CAR_vel_CL_set_aim(0.0f, 0.0f);
                CN_Chassis_Reset_Speed_Ref(0.0f, 0.0f);
                CN_CAR_rotation_set_aim(0.0f);

                if ((time % 10U) == 0U) {
                    CN_CAR_yaw_CL_CALLBACK(10000U);
                }
                CN_CAR_vel_CL_CALLBACK(1000U);
                CN_Chassis_Brake_Update(ms);
            } else {
                CN_Intr_Movement();
                YAW_aim = YAW_now;
            }
            break;
        }

        case Movement_Running: {
            if ((time % 10U) == 0U) {
                YAW_aim = CN_yaw_add(YAW_aim, YAW_acc * 0.01f);
                CN_CAR_yaw_CL_CALLBACK(10000U);
            }
            CN_CAR_vel_CL_CALLBACK(1000U);
            break;
        }

        case Movement_Rotation: {
            float yaw_delta = YAW_now - YAW_pre;
            float remaining = 0.0f;
            float abs_remaining = 0.0f;
            float cmd_anv = 0.0f;

            if (yaw_delta > 180.0f) {
                yaw_delta -= 360.0f;
            } else if (yaw_delta < -180.0f) {
                yaw_delta += 360.0f;
            }

            YAW_gain += yaw_delta;
            YAW_pre = YAW_now;
            remaining = target_yaw - YAW_gain;
            abs_remaining = fabsf(remaining);

            if (abs_remaining <= CHASSIS_ROTATION_ANGLE_TOL_DEG) {
                CN_Chassis_Finish_To_Run(g_chassis_rotation_target_abs_yaw);
                break;
            }

            if (abs_remaining < CHASSIS_ROTATION_SLOW_ANGLE_DEG) {
                cmd_anv = g_chassis_rotation_max_angvel *
                    (abs_remaining / CHASSIS_ROTATION_SLOW_ANGLE_DEG);
                if ((cmd_anv < CHASSIS_ROTATION_MIN_ANGVEL) &&
                    (abs_remaining > CHASSIS_ROTATION_ANGLE_TOL_DEG)) {
                    cmd_anv = CHASSIS_ROTATION_MIN_ANGVEL;
                }
            } else {
                cmd_anv = g_chassis_rotation_max_angvel;
            }

            if (remaining < 0.0f) {
                cmd_anv = -cmd_anv;
            }

            YAW_aim = YAW_now;
            CN_CAR_vel_CL_set_aim(0.0f, 0.0f);
            CN_Chassis_Reset_Speed_Ref(0.0f, 0.0f);
            CN_CAR_anv_CL_set_aim(cmd_anv);

            CN_CAR_vel_CL_CALLBACK(1000U);
            break;
        }

        case Movement_Translation: {
            YAW_aim = initial_yaw;

            if ((time % 10U) == 0U) {
                CN_CAR_yaw_CL_CALLBACK(10000U);
            }
            CN_CAR_pos_CL_CALLBACK(1000U);
            if (g_chassis_translation_reached != 0U) {
                CN_Chassis_Finish_To_Run(initial_yaw);
                break;
            }

            CN_CAR_vel_CL_CALLBACK(1000U);
            break;
        }

        default: {
            break;
        }
    }
}

// 进入 run 状态；keep_yaw_aim 为 1 时沿用已有姿态目标。
static void CN_Chassis_Enter_Run(Car_Movement_struct movement, uint8_t keep_yaw_aim)
{
    Whole_Movement_Type = Movement_Running;

    if (keep_yaw_aim == 0U) {
        YAW_aim = YAW_now;
        PID_ctrler_reset(&Car_yaw_PID_ctrler);
        CN_Chassis_Reset_Speed_Ref(0.0f, 0.0f);
        CN_Chassis_Reset_Wheel_Pid();
    }

    CN_CAR_vel_CL_set_aim(movement.Car_X_vel, movement.Car_Y_vel);
    CN_CAR_rotation_set_aim(movement.Car_angvel);
}

// rotation 或 translation 结束后回到零速度 run，保持传入的姿态目标。
static void CN_Chassis_Finish_To_Run(float yaw_hold)
{
    Car_Movement_struct stop_movement = {0};

    Whole_Movement = stop_movement;
    Whole_Movement_Type = Movement_Running;
    YAW_aim = yaw_hold;

    CN_CAR_vel_CL_set_aim(0.0f, 0.0f);
    CN_Chassis_Reset_Speed_Ref(0.0f, 0.0f);
    CN_CAR_rotation_set_aim(0.0f);
    CN_CAR_anv_CL_set_aim(0.0f);
    CN_Chassis_Reset_Wheel_Pid();
}

/****************************车身角度环***************************/
#define CAR_YAW_CL_PID_KP      (0.25f)
#define CAR_YAW_CL_PID_KI      (0.0f)
#define CAR_YAW_CL_PID_KD      (0.0f)
#define CAR_YAW_CL_PID_IMAX    (0.0f)
#define CAR_YAW_CL_PID_OUTMAX  (10.0f)

float YAW_acc = 0.0f;          // run 模式下的航向目标变化率。

// 设置 run 模式航向目标变化率。
void CN_CAR_rotation_set_aim(float oumiga)
{
    YAW_acc = oumiga;
}

// 车身角度环初始化
void CN_CAR_yaw_CL_init(void)
{
    init_PID_ctrler(&Car_yaw_PID_ctrler, CAR_YAW_CL_PID_KP, CAR_YAW_CL_PID_KI,
                    CAR_YAW_CL_PID_KD, CAR_YAW_CL_PID_IMAX, CAR_YAW_CL_PID_OUTMAX);
}

// 车身角度环回调函数
void CN_CAR_yaw_CL_CALLBACK(uint32_t us_time)
{
    float yaw_err = CN_get_relative_angle(YAW_now, YAW_aim);

    if (fabsf(yaw_err) < 1.0f) {
        yaw_err = 0.0f;
    }

    PID_ctrler_update_with_time(&Car_yaw_PID_ctrler, yaw_err, us_time);
    CN_CAR_anv_CL_set_aim(Car_yaw_PID_ctrler.output);
}

// 获取两个绝对方位角之间的最短相对角度，范围约为 -180 到 180。
float CN_get_relative_angle(float now, float aim)
{
    float difference = aim - now;

    if (difference < -180.0f) return difference + 360.0f;
    if (difference > 180.0f) return difference - 360.0f;
    return difference;
}

// 在 -180 到 180 的角度范围内累加角度。
float CN_yaw_add(float now, float aim)
{
    float res = now + aim;

    if (res < -180.0f) return res + 360.0f;
    if (res > 180.0f) return res - 360.0f;
    return res;
}

// 设置相对当前角度的姿态目标。
void CN_CAR_yaw_CL_set_aim(float yaw)
{
    YAW_aim = CN_yaw_add(YAW_now, yaw);
}
/****************************车身角度环***************************/

/****************************底盘旋转分量***************************/

// 由于该车模电机响应死驱较大，并且编码器进度较低，速度环响应延时，不适合高频响应
// 而陀螺仪的角速度变化灵敏，在控制频率为200hz的角速度环

float target_anv = 0.0f;       

//+初始化底盘旋转分量；保留原函数名，方便旧代码继续调用。
void CN_CAR_anv_CL_init(void)
{
    target_anv = 0.0f;
}

// 设置底盘解算旋转分量；定角旋转和航向角度环会直接写入该值。
void CN_CAR_anv_CL_set_aim(float anv)
{
    target_anv = anv;
}
/****************************底盘旋转分量***************************/

/****************************车身速度环***************************/
// 置 1 启用速度目标斜坡，避免 run/translation 速度命令直接跳变。
#define CHASSIS_VEL_RAMP_ENABLE  1U
// 速度目标每秒最大变化量，单位与 target_VelX/Y 一致。
#define CHASSIS_VEL_RAMP_ACC     15.0f

float target_VelX = 0.0f;       // 上层给出的 X 方向速度目标
float target_VelY = 0.0f;       // 上层给出的 Y 方向速度目标

static float g_chassis_velx_ref = 0.0f;      // 实际送入轮速解算的 X 速度
static float g_chassis_vely_ref = 0.0f;      // 实际送入轮速解算的 Y 速度

// 重置速度斜坡的当前值，常用于进入 break、rotation、translation。
static void CN_Chassis_Reset_Speed_Ref(float vx, float vy)
{
    g_chassis_velx_ref = vx;
    g_chassis_vely_ref = vy;
}

// 速度环初始化
void CN_CAR_vel_CL_init(void)
{
    CN_Motor_vel_CL_init();
    CN_Chassis_Reset_Speed_Ref(0.0f, 0.0f);
}

// 设置车身速度目标
void CN_CAR_vel_CL_set_aim(float VelX, float VelY)
{
    target_VelX = VelX;
    target_VelY = VelY;
}

// 速度环控制回调
void CN_CAR_vel_CL_CALLBACK(uint32_t us_time)
{
    float dt_s = (float)us_time / 1000000.0f;
    float ramp_step = CHASSIS_VEL_RAMP_ACC * dt_s;

    if (dt_s <= 0.0f) {
        dt_s = 0.001f;
        ramp_step = CHASSIS_VEL_RAMP_ACC * dt_s;
    }

#if CHASSIS_VEL_RAMP_ENABLE
    g_chassis_velx_ref = CN_Chassis_Approach_Float(g_chassis_velx_ref, target_VelX, ramp_step);
    g_chassis_vely_ref = CN_Chassis_Approach_Float(g_chassis_vely_ref, target_VelY, ramp_step);
#else
    g_chassis_velx_ref = target_VelX;
    g_chassis_vely_ref = target_VelY;
#endif

    Vel_aim.M_FL_Vel = (g_chassis_vely_ref + g_chassis_velx_ref + target_anv) * M_TO_ENC_FACTOR;
    Vel_aim.M_FR_Vel = (g_chassis_vely_ref - g_chassis_velx_ref - target_anv) * M_TO_ENC_FACTOR;
    Vel_aim.M_BL_Vel = (g_chassis_vely_ref - g_chassis_velx_ref + target_anv) * M_TO_ENC_FACTOR;
    Vel_aim.M_BR_Vel = (g_chassis_vely_ref + g_chassis_velx_ref - target_anv) * M_TO_ENC_FACTOR;

    CN_Motor_vel_CL_CALLBACK(us_time);
}
/****************************车身速度环***************************/

/****************************任意角度旋转***************************/
// 进入任意角度旋转状态。
static void CN_Chassis_Enter_Rotation(Car_Movement_struct movement)
{
    Whole_Movement_Type = Movement_Rotation;

    target_yaw = movement.Car_angle;
    YAW_inital = YAW_now;
    YAW_gain = 0.0f;
    YAW_pre = YAW_now;
    g_chassis_rotation_target_abs_yaw = CN_yaw_add(YAW_now, target_yaw);
    g_chassis_rotation_max_angvel = fabsf(movement.Car_angvel);

    if (g_chassis_rotation_max_angvel < CHASSIS_ROTATION_MIN_ANGVEL) {
        g_chassis_rotation_max_angvel = CHASSIS_ROTATION_DEFAULT_ANGVEL;
    }
    if (g_chassis_rotation_max_angvel > CHASSIS_ROTATION_MAX_ANGVEL) {
        g_chassis_rotation_max_angvel = CHASSIS_ROTATION_MAX_ANGVEL;
    }

    CN_CAR_vel_CL_set_aim(0.0f, 0.0f);
    CN_Chassis_Reset_Speed_Ref(0.0f, 0.0f);
    CN_CAR_rotation_set_aim(0.0f);
    CN_CAR_anv_CL_set_aim(0.0f);
    CN_Chassis_Reset_Wheel_Pid();

    if (fabsf(target_yaw) <= CHASSIS_ROTATION_ANGLE_TOL_DEG) {
        CN_Chassis_Finish_To_Run(YAW_now);
    }
}
/****************************任意角度旋转***************************/

/****************************定点定向平移***************************/
// 定点平移初始化
void CN_CAR_pos_CL_init(void)
{
    init_PID_ctrler(&Car_yaw_moving_PID_ctrler, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
}

// 进入定点平移状态。
static void CN_Chassis_Enter_Translation(Car_Movement_struct movement)
{
    Whole_Movement_Type = Movement_Translation;
    CN_CAR_pos_CL_set_aim(movement);
}

// 设置平移目标。这里不做复杂纠偏，只记录目标方向并清零相对里程。
void CN_CAR_pos_CL_set_aim(Car_Movement_struct C_pos_aim)
{
    float target_distance = 0.0f;

    initial_yaw = YAW_now;
    YAW_aim = initial_yaw;

    start_pose.x = 0.0f;
    start_pose.y = 0.0f;
    cur_x = 0.0f;
    cur_y = 0.0f;

    target_pose.x = C_pos_aim.Car_X_movement;
    target_pose.y = C_pos_aim.Car_Y_movement;
    target_distance = sqrtf(target_pose.x * target_pose.x + target_pose.y * target_pose.y);

    g_chassis_translation_total_distance = target_distance;
    distance_to_go = target_distance;
    g_chassis_translation_speed_ref = 0.0f;
    g_chassis_translation_reach_ms = 0U;
    g_chassis_translation_reached = 0U;

    g_chassis_translation_max_speed = fabsf(C_pos_aim.Car_translation_vel);
    if (g_chassis_translation_max_speed < CHASSIS_TRANSLATION_MIN_MOVE_SPEED) {
        g_chassis_translation_max_speed = CHASSIS_TRANSLATION_DEFAULT_SPEED;
    }
    g_chassis_translation_max_speed =
        CN_Chassis_Limit_Float(g_chassis_translation_max_speed, CHASSIS_TRANSLATION_MAX_SPEED);
    if (g_chassis_translation_max_speed < 0.0f) {
        g_chassis_translation_max_speed = -g_chassis_translation_max_speed;
    }

    if (target_distance <= CHASSIS_TRANSLATION_POS_TOL) {
        g_chassis_translation_dir_x = 0.0f;
        g_chassis_translation_dir_y = 0.0f;
        g_chassis_translation_reached = 1U;
    } else {
        g_chassis_translation_dir_x = target_pose.x / target_distance;
        g_chassis_translation_dir_y = target_pose.y / target_distance;
    }

    CN_CAR_vel_CL_set_aim(0.0f, 0.0f);
    CN_Chassis_Reset_Speed_Ref(0.0f, 0.0f);
    CN_CAR_rotation_set_aim(0.0f);
    CN_CAR_anv_CL_set_aim(0.0f);
    PID_ctrler_reset(&Car_yaw_PID_ctrler);
    CN_Chassis_Reset_Wheel_Pid();
}

// 位置环核心回调：相信四轮编码器合成位移，不额外做横向纠偏。
void CN_CAR_pos_CL_CALLBACK(uint32_t us_time)
{
    float fl_delta = (float)ENC_data[3];
    float fr_delta = (float)ENC_data[1];
    float bl_delta = (float)ENC_data[2];
    float br_delta = (float)ENC_data[0];
    float delta_x = 0.0f;
    float delta_y = 0.0f;
    float progress = 0.0f;
    float speed_target = 0.0f;
    float dt_s = (float)us_time / 1000000.0f;
    float speed_step = CHASSIS_TRANSLATION_ACC * dt_s;

    if (g_chassis_translation_reached != 0U) {
        CN_CAR_vel_CL_set_aim(0.0f, 0.0f);
        return;
    }

    if (dt_s <= 0.0f) {
        dt_s = 0.001f;
        speed_step = CHASSIS_TRANSLATION_ACC * dt_s;
    }

    delta_x = ((fl_delta - fr_delta - bl_delta + br_delta) / 4.0f) / M_TO_ENC_FACTOR;
    delta_y = ((fl_delta + fr_delta + bl_delta + br_delta) / 4.0f) / M_TO_ENC_FACTOR;
    cur_x += delta_x;
    cur_y += delta_y;

    progress = cur_x * g_chassis_translation_dir_x + cur_y * g_chassis_translation_dir_y;
    distance_to_go = g_chassis_translation_total_distance - progress;
    if (distance_to_go < 0.0f) {
        distance_to_go = 0.0f;
    }

    if (distance_to_go <= CHASSIS_TRANSLATION_POS_TOL) {
        CN_CAR_vel_CL_set_aim(0.0f, 0.0f);
        g_chassis_translation_speed_ref =
            CN_Chassis_Approach_Float(g_chassis_translation_speed_ref, 0.0f, speed_step);

        if (CN_Chassis_Max_Wheel_Vel() <= CHASSIS_TRANSLATION_REACH_VEL_EPS) {
            g_chassis_translation_reach_ms += (uint32_t)(us_time / 1000U);
        } else {
            g_chassis_translation_reach_ms = 0U;
        }

        if (g_chassis_translation_reach_ms >= CHASSIS_TRANSLATION_REACH_CONFIRM_MS) {
            g_chassis_translation_reached = 1U;
        }
        return;
    }

    if (distance_to_go < CHASSIS_TRANSLATION_SLOW_DISTANCE) {
        speed_target = g_chassis_translation_max_speed *
            (distance_to_go / CHASSIS_TRANSLATION_SLOW_DISTANCE);
    } else {
        speed_target = g_chassis_translation_max_speed;
    }

    if ((speed_target < CHASSIS_TRANSLATION_MIN_MOVE_SPEED) &&
        (distance_to_go > CHASSIS_TRANSLATION_POS_TOL)) {
        speed_target = CHASSIS_TRANSLATION_MIN_MOVE_SPEED;
    }

    g_chassis_translation_speed_ref =
        CN_Chassis_Approach_Float(g_chassis_translation_speed_ref, speed_target, speed_step);

    CN_CAR_vel_CL_set_aim(g_chassis_translation_speed_ref * g_chassis_translation_dir_x,
                          g_chassis_translation_speed_ref * g_chassis_translation_dir_y);
}
/****************************定点定向平移***************************/
