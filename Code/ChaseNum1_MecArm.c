#include "ChaseNum1_Include.h"
#include <string.h>
#include <stdio.h>

HostCmd_t g_HostCmd;

#ifndef M_PI
// 机械臂逆解使用的圆周率常量。
#define M_PI 3.14159265358979323846f
#endif

// 云台中心距地面高度，单位 cm。
#define ARM_H0      11.8f
// 云台中心到大齿轮关节的高度差，单位 cm。
#define ARM_H1      6.1f
// 云台中心到大齿轮关节的水平投影偏置，单位 cm。
#define ARM_R       3.1f
// 第一段机械臂名义长度，单位 cm。
#define ARM_L1      17.5f
// 第二关节相对第一段机械臂的垂直机械偏置，单位 cm。
#define ARM_C       2.0f
// 第二段机械臂长度，单位 cm。
#define ARM_L2      21.0f
// 第四舵机轴心到吸盘末端的距离，单位 cm。
#define ARM_L3      9.0f

// 0 号舵机总线 ID。
#define S0_ID       0
// 0 号舵机虚拟角到物理角的比例。
#define S0_GEAR     1.0f
// 0 号舵机虚拟角到物理角的方向。
#define S0_DIR      1.0f
// 0 号舵机物理零点偏置，单位 deg。
#define S0_OFFSET   2.0f
// 0 号舵机物理最小限位，单位 deg。
#define S0_MIN      -180.0f
// 0 号舵机物理最大限位，单位 deg。
#define S0_MAX      180.0f

// 1 号舵机总线 ID。
#define S1_ID       1
// 1 号舵机虚拟角到物理角的比例。
#define S1_GEAR     3.0f
// 1 号舵机虚拟角到物理角的方向。
#define S1_DIR      1.0f
// 1 号舵机物理零点偏置，单位 deg。
#define S1_OFFSET   -520.0f                     // -534
// 1 号舵机物理最小限位，单位 deg。
#define S1_MIN      -520.0f
// 1 号舵机物理最大限位，单位 deg。
#define S1_MAX      20.0f

// 2 号舵机总线 ID。
#define S2_ID       2
// 2 号舵机虚拟角到物理角的比例。
#define S2_GEAR     1.0f
// 2 号舵机虚拟角到物理角的方向。
#define S2_DIR      -1.0f
// 2 号舵机物理零点偏置，单位 deg。
#define S2_OFFSET   -13.0f                       // -13.0f
// 2 号舵机物理最小限位，单位 deg。
#define S2_MIN      -90.0f
// 2 号舵机物理最大限位，单位 deg。
#define S2_MAX      150.0f

// 3 号舵机总线 ID。
#define S3_ID       3
// 3 号舵机虚拟角到物理角的比例。
#define S3_GEAR     1.0f
// 3 号舵机虚拟角到物理角的方向。
#define S3_DIR      -1.0f
// 3 号舵机物理零点偏置，单位 deg。
#define S3_OFFSET   -95.0f       // -95
// 3 号舵机物理最小限位，单位 deg。
#define S3_MIN      -230.0f
// 3 号舵机物理最大限位，放开到正向以支持吸盘竖直向下，单位 deg。
#define S3_MAX      40.0f

// 置 1 使用速度型同步命令；置 0 回退为逐个舵机速度命令。
#define ARM_USE_SYNC_SERVO_SPEED_CMD   0
// 0 号云台舵机运动速度，单位 0.1 deg/s。
#define ARM_SERVO_S0_MOVE_SPEED        4000U
// 1 号多圈舵机运动速度，单位 0.1 deg/s。
#define ARM_SERVO_S1_MOVE_SPEED        5000U
// 2 号舵机运动速度，单位 0.1 deg/s。
#define ARM_SERVO_S2_MOVE_SPEED        1000U
// 3 号末端舵机运动速度，单位 0.1 deg/s。
#define ARM_SERVO_S3_MOVE_SPEED        1000U
// 速度命令的加速时间，单位 ms。
#define ARM_SERVO_ACCEL_TIME_MS        500U
// 速度命令的减速时间，单位 ms。
#define ARM_SERVO_DECEL_TIME_MS        500U
// 舵机功率限制；0 表示保持舵机固件默认值。 
#define ARM_SERVO_POWER_LIMIT          10000U

// 判断目标是否跨过 x=0 平面时使用的世界 X 死区，单位 cm。
#define ARM_X_SIDE_EPS_CM              1.0f
// 收敛姿态在当前云台竖直平面内的深度坐标，单位 cm；数值越小，折叠姿态越靠近云台。
#define ARM_FOLD_PLANE_DEPTH_CM        15.0f
// 收敛姿态的吸盘末端高度坐标，单位 cm；该高度需高于禁区上边界。
#define ARM_FOLD_PLANE_Z_CM            32.0f
// 收敛姿态的吸盘方向，单位 deg；0deg 表示吸盘末端竖直向下。
#define ARM_FOLD_SUCKER_DIR_DEG        0.0f
//+调整上面三个宏时必须保证逆解后的舵机角能通过 ARM_FOLD_SAFE_MARGIN，否则规划器会拒绝折叠动作。
// 默认休息姿态云台虚拟角，单位 deg。
#define ARM_DEFAULT_REST_BASE_DEG      90.0f
// 初始化是否走规划器回到默认休息姿态；1=先收敛再转平面再休息，0=直接下发默认休息姿态。
#define ARM_INIT_USE_PLANNER_HOME      0U
// 初始化时是否尝试读取 0 号云台舵机当前单圈角度；读取成功后按真实当前平面先收敛。
#define ARM_INIT_READ_BASE_ENABLE      1U
// 初始化读取云台舵机角度的最大等待时间，单位 ms；超时后使用下方假定角度。
#define ARM_INIT_BASE_READ_TIMEOUT_MS  50U
// 初始化读取云台角失败时假定的当前云台虚拟角，单位 deg，建议与实际上电摆放平面一致。
#define ARM_INIT_FALLBACK_BASE_DEG     ARM_DEFAULT_REST_BASE_DEG
// 默认休息姿态中 1 号舵机的虚拟角，配合 2 号舵机让第三轴连带臂向下，单位 deg。
#define ARM_REST_TH1_DEG               80.0f
// 默认休息姿态中 2 号舵机的虚拟角，比上一版略向前，单位 deg。
#define ARM_REST_TH2_DEG               -150.0f
// 默认休息姿态中 3 号舵机的虚拟角，比上一版继续向下转 30deg，单位 deg。
#define ARM_REST_TH3_DEG               0.0f
// 固定收敛姿态距离各物理限位的最小安全余量，单位 deg。
#define ARM_FOLD_SAFE_MARGIN           10.0f
// 默认休息姿态距离各物理限位的最小安全余量，单位 deg；3 号舵机向下后保留 5deg 余量。
#define ARM_REST_SAFE_MARGIN           5.0f
// 只有云台角变化小到接近数值噪声时才跳过收敛转平面阶段，单位 deg。
#define ARM_PLANE_ROTATE_SKIP_DEG      0.05f
// 每个阶段速度估算后追加的非阻塞等待余量，单位 ms。
#define ARM_PHASE_SETTLE_EXTRA_MS      500U
// 进入下一阶段前的最小非阻塞等待时间，单位 ms。
#define ARM_PHASE_MIN_INTERVAL_MS      500U
// 进入下一阶段前的最大非阻塞等待时间，单位 ms。
#define ARM_PHASE_MAX_INTERVAL_MS      5000U
// 最大阶段数：收敛、转平面、展开。
#define ARM_PLAN_PHASE_MAX             3U

// 前方竖直细杆的参考 Y 坐标，单位 cm。
#define ARM_FRONT_ROD_Y_CM             25.0f
// 前方竖直细杆的保守避让半径，最终吸盘末端不得进入该范围，单位 cm。
#define ARM_FRONT_ROD_RADIUS_CM        0.5f

//+梯形禁区近端 Y 坐标，单位 cm，使用严格不等式 3<y。
#define ARM_FORBID_Y_NEAR_CM           3.0f
//+梯形禁区远端 Y 坐标，单位 cm，使用严格不等式 y<30。
#define ARM_FORBID_Y_FAR_CM            30.0f
//+Y=3cm 时禁区 X 半宽，单位 cm，对应 -5<x<5。
#define ARM_FORBID_X_HALF_NEAR_CM      9.0f
//+Y=30cm 时禁区 X 半宽，单位 cm，对应 -50<x<50。
#define ARM_FORBID_X_HALF_FAR_CM       90.0f
//+梯形禁区 Z 上边界，单位 cm，使用严格不等式 z<20。
#define ARM_FORBID_Z_MAX_CM            20.0f
#define ARM_FORBID_ABOVE_SAFE_MARGIN_CM 2.0f
//+云台角度禁区额外余量，单位 deg；目标云台角落入禁区方向时先收敛再展开。
#define ARM_FORBID_BASE_MARGIN_DEG     0.0f

typedef struct {
    float x; 
    float y;
    float z;
    float dir;
} ArmWorldTarget_t;

typedef struct {
    float th0;
    float th1;
    float th2;
    float th3;
} ArmJointPose_t;

typedef struct {
    ArmWorldTarget_t target;
    ArmJointPose_t pose;
    uint32_t wait_ms;
    const char *name;
} ArmPlanPhase_t;

typedef struct {
    uint8_t active;
    uint8_t plan_cmd_type;
    uint8_t phase_count;
    uint8_t phase_index;
    uint32_t phase_start_tick;
    ArmPlanPhase_t phases[ARM_PLAN_PHASE_MAX];
    ArmWorldTarget_t final_target;
    ArmJointPose_t final_pose;
} ArmMotionPlan_t;

static uint8_t Arm_Target_In_Forbidden_Zone(const ArmWorldTarget_t *target);
static uint8_t Arm_Base_Angle_In_Forbidden_Direction(float base_th0);

static ArmWorldTarget_t g_arm_current_target = {0.0f, 0.0f, 0.0f, 90.0f};
static ArmJointPose_t g_arm_current_joint = {0.0f, 0.0f, 0.0f, 0.0f};
static uint8_t g_arm_current_valid = 0;
// 保留最近一次内部错误，方便仿真或在线调试器观察，暂不通过通信链路回传。
static const char * volatile g_arm_last_error = "OK";
static ArmMotionPlan_t g_arm_plan = {0};
//+机械臂内部事件由系统状态机读取后清零。
static uint8_t g_arm_status_event = ARM_STATUS_EVENT_NONE;

// 将角度归一化到 [-180, 180)。
static float Arm_Normalize_Deg(float angle_deg)
{
    while (angle_deg >= 180.0f) angle_deg -= 360.0f;
    while (angle_deg < -180.0f) angle_deg += 360.0f;
    return angle_deg;
}

// 将云台舵机物理角折回单圈范围，保证虚拟 -180~180deg 的等效方向都可下发。
static float Arm_Normalize_Base_Servo_Angle(float actual_s0)
{
    while (actual_s0 > S0_MAX) actual_s0 -= 360.0f;
    while (actual_s0 < S0_MIN) actual_s0 += 360.0f;
    return actual_s0;
}

// 将 0 号舵机读取到的物理角反算为机械臂规划使用的云台虚拟角。
static float Arm_Base_Actual_To_Virtual(float actual_s0)
{
    actual_s0 = Arm_Normalize_Base_Servo_Angle(actual_s0);
    return Arm_Normalize_Deg((actual_s0 - S0_OFFSET) / (S0_GEAR * S0_DIR));
}

// 初始化阶段读取当前云台平面；读取失败时由调用者使用保守假定角度。
static uint8_t Arm_Read_Init_Base_Angle(float *base_th0)
{
#if ARM_INIT_READ_BASE_ENABLE
    const int32_t invalid_position = 0x7FFFFFFF;
    int32_t old_position = 0;
    uint32_t start_tick = 0U;

    if (base_th0 == NULL) return 0U;

    old_position = g_ServoInfo[S0_ID].position;
    g_ServoInfo[S0_ID].position = invalid_position;
    start_tick = HAL_GetTick();
    Servo_Read_Single_Turn_Angle(S0_ID);

    while ((uint32_t)(HAL_GetTick() - start_tick) < ARM_INIT_BASE_READ_TIMEOUT_MS) {
        if (g_ServoInfo[S0_ID].position != invalid_position) {
            *base_th0 = Arm_Base_Actual_To_Virtual((float)g_ServoInfo[S0_ID].position / 10.0f);
            return 1U;
        }
    }

    g_ServoInfo[S0_ID].position = old_position;
#else
    (void)base_th0;
#endif
    return 0U;
}

// 返回浮点数绝对值。
static float Arm_Abs_Float(float value)
{
    return (value >= 0.0f) ? value : -value;
}

// 返回两个浮点数中的较大值。
static float Arm_Max_Float(float a, float b)
{
    return (a > b) ? a : b;
}

// 将 uint32 数值限制在指定上下限之间。
static uint32_t Arm_Clamp_U32(uint32_t value, uint32_t min_value, uint32_t max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

// 根据单个舵机角度变化和速度估算运动耗时，速度单位为 0.1 deg/s。
static uint32_t Arm_Estimate_Servo_Move_Ms(float delta_deg, uint16_t speed)
{
    float abs_delta = Arm_Abs_Float(delta_deg);
    uint32_t move_ms = 0U;
    uint32_t ramp_ms = (uint32_t)ARM_SERVO_ACCEL_TIME_MS + (uint32_t)ARM_SERVO_DECEL_TIME_MS;

    if (speed == 0U) {
        return ARM_PHASE_MAX_INTERVAL_MS;
    }

    move_ms = (uint32_t)((abs_delta * 10000.0f) / (float)speed);
    if (move_ms > ramp_ms) {
        move_ms += ramp_ms;
    } else {
        move_ms += ramp_ms / 2U;
    }
    return move_ms;
}

// 将目标世界 XY 坐标转换为云台虚拟角。
static float Arm_World_To_Base_Angle(float world_x, float world_y)
{
    float depth = sqrtf(world_x * world_x + world_y * world_y);
    if (depth <= 0.001f) return 0.0f;
    return Arm_Normalize_Deg(atan2f(world_x, world_y) * 180.0f / M_PI);
}

// 带死区判断世界 X 坐标位于正侧、负侧还是零点附近。
static int8_t Arm_Get_World_X_Side(float world_x)
{
    if (world_x > ARM_X_SIDE_EPS_CM) return 1;
    if (world_x < -ARM_X_SIDE_EPS_CM) return -1;
    return 0;
}

// 判断新目标是否从当前非零 X 侧跨到另一侧。
static uint8_t Arm_Target_Crosses_X_Zero(const ArmWorldTarget_t *target)
{
    int8_t current_side = 0;
    int8_t target_side = 0;

    if (target == NULL || !g_arm_current_valid) return 0;

    current_side = Arm_Get_World_X_Side(g_arm_current_target.x);
    target_side = Arm_Get_World_X_Side(target->x);
    return (current_side != 0 && target_side != 0 && current_side != target_side) ? 1U : 0U;
}

//+根据梯形禁区水平投影计算云台禁区半角，单位 deg。
static float Arm_Forbidden_Base_Half_Angle(void)
{
    float near_angle = atan2f(ARM_FORBID_X_HALF_NEAR_CM, ARM_FORBID_Y_NEAR_CM) * 180.0f / M_PI;
    float far_angle = atan2f(ARM_FORBID_X_HALF_FAR_CM, ARM_FORBID_Y_FAR_CM) * 180.0f / M_PI;

    return Arm_Max_Float(near_angle, far_angle) + ARM_FORBID_BASE_MARGIN_DEG;
}

//+判断目标云台角是否指向禁区水平投影；指向禁区时需要先收敛再转平面。
static uint8_t Arm_Base_Angle_In_Forbidden_Direction(float base_th0)
{
    float half_angle = Arm_Forbidden_Base_Half_Angle();
    float base_abs = Arm_Abs_Float(Arm_Normalize_Deg(base_th0));

    return (base_abs <= half_angle) ? 1U : 0U;
}

// 根据一组虚拟关节角正解出吸盘末端的世界坐标记录。
static ArmWorldTarget_t Arm_Forward_Target_From_Pose(const ArmJointPose_t *pose)
{
    ArmWorldTarget_t target = {0.0f, 0.0f, 0.0f, 90.0f};
    float r0 = 0.0f;
    float r1 = 0.0f;
    float r12 = 0.0f;
    float r123 = 0.0f;
    float plane_x = 0.0f;
    float plane_z = 0.0f;

    if (pose == NULL) return target;

    r0 = pose->th0 * M_PI / 180.0f;
    r1 = pose->th1 * M_PI / 180.0f;
    r12 = (pose->th1 + pose->th2) * M_PI / 180.0f;
    r123 = (pose->th1 + pose->th2 + pose->th3) * M_PI / 180.0f;

    plane_x = ARM_R
              + ARM_L1 * cosf(r1)
              - ARM_C * sinf(r1)
              + ARM_L2 * cosf(r12)
              + ARM_L3 * cosf(r123);
    plane_z = ARM_H0 + ARM_H1
              + ARM_L1 * sinf(r1)
              + ARM_C * cosf(r1)
              + ARM_L2 * sinf(r12)
              + ARM_L3 * sinf(r123);

    target.x = plane_x * sinf(r0);
    target.y = plane_x * cosf(r0);
    target.z = plane_z;
    target.dir = Arm_Normalize_Deg(pose->th1 + pose->th2 + pose->th3 + 90.0f);
    return target;
}

// 将虚拟关节角映射为标定后的真实舵机角。
static void Arm_Map_To_Servo_Angles(float v_th0, float v_th1, float v_th2, float v_th3,
                                    float *actual_s0, float *actual_s1,
                                    float *actual_s2, float *actual_s3)
{
    *actual_s0 = Arm_Normalize_Base_Servo_Angle((v_th0 * S0_GEAR * S0_DIR) + S0_OFFSET);
    *actual_s1 = (v_th1 * S1_GEAR * S1_DIR) + S1_OFFSET;
    *actual_s2 = (v_th2 * S2_GEAR * S2_DIR) + S2_OFFSET;
    *actual_s3 = (v_th3 * S3_GEAR * S3_DIR) + S3_OFFSET;
}

// 按指定安全余量检查标定后的真实舵机角是否越限。
static uint8_t Arm_Is_Servo_Angle_In_Range_With_Margin(float v_th0, float v_th1, float v_th2,
                                                       float v_th3, float margin_deg)
{
    float actual_s0 = 0.0f;
    float actual_s1 = 0.0f;
    float actual_s2 = 0.0f;
    float actual_s3 = 0.0f;

    Arm_Map_To_Servo_Angles(v_th0, v_th1, v_th2, v_th3,
                            &actual_s0, &actual_s1, &actual_s2, &actual_s3);

    if (actual_s0 < S0_MIN || actual_s0 > S0_MAX) return 0;
    if (actual_s1 < S1_MIN + margin_deg || actual_s1 > S1_MAX - margin_deg) return 0;
    if (actual_s2 < S2_MIN + margin_deg || actual_s2 > S2_MAX - margin_deg) return 0;
    if (actual_s3 < S3_MIN + margin_deg || actual_s3 > S3_MAX - margin_deg) return 0;
    return 1;
}

// 检查标定后的真实舵机角是否处于硬限位范围内。
static uint8_t Arm_Is_Servo_Angle_In_Range(float v_th0, float v_th1, float v_th2, float v_th3)
{
    return Arm_Is_Servo_Angle_In_Range_With_Margin(v_th0, v_th1, v_th2, v_th3, 0.0f);
}

// 为指定云台平面生成默认休息姿态。
static uint8_t Arm_Build_Rest_Pose(float base_th0, ArmWorldTarget_t *target, ArmJointPose_t *pose)
{
    if (target == NULL || pose == NULL) return 0;

    pose->th0 = Arm_Normalize_Deg(base_th0);
    pose->th1 = ARM_REST_TH1_DEG;
    pose->th2 = ARM_REST_TH2_DEG;
    pose->th3 = ARM_REST_TH3_DEG;

    if (!Arm_Is_Servo_Angle_In_Range_With_Margin(pose->th0, pose->th1, pose->th2,
                                                 pose->th3, ARM_REST_SAFE_MARGIN)) {
        return 0;
    }

    *target = Arm_Forward_Target_From_Pose(pose);
    if (Arm_Target_In_Forbidden_Zone(target)) return 0;
    return 1;
}

// 返回两个姿态之间最大的真实舵机角变化量。
static float Arm_Max_Servo_Delta(const ArmJointPose_t *from, const ArmJointPose_t *to)
{
    float from_s0 = 0.0f;
    float from_s1 = 0.0f;
    float from_s2 = 0.0f;
    float from_s3 = 0.0f;
    float to_s0 = 0.0f;
    float to_s1 = 0.0f;
    float to_s2 = 0.0f;
    float to_s3 = 0.0f;
    float max_delta = 0.0f;

    if (from == NULL || to == NULL) return 0.0f;

    Arm_Map_To_Servo_Angles(from->th0, from->th1, from->th2, from->th3,
                            &from_s0, &from_s1, &from_s2, &from_s3);
    Arm_Map_To_Servo_Angles(to->th0, to->th1, to->th2, to->th3,
                            &to_s0, &to_s1, &to_s2, &to_s3);

    max_delta = Arm_Max_Float(max_delta, Arm_Abs_Float(to_s0 - from_s0));
    max_delta = Arm_Max_Float(max_delta, Arm_Abs_Float(to_s1 - from_s1));
    max_delta = Arm_Max_Float(max_delta, Arm_Abs_Float(to_s2 - from_s2));
    max_delta = Arm_Max_Float(max_delta, Arm_Abs_Float(to_s3 - from_s3));
    return max_delta;
}

// 根据四个舵机各自速度估算一个阶段何时可认为完成。
static uint32_t Arm_Estimate_Phase_Wait_Ms(const ArmJointPose_t *from, const ArmJointPose_t *to)
{
    float from_s0 = 0.0f;
    float from_s1 = 0.0f;
    float from_s2 = 0.0f;
    float from_s3 = 0.0f;
    float to_s0 = 0.0f;
    float to_s1 = 0.0f;
    float to_s2 = 0.0f;
    float to_s3 = 0.0f;
    uint32_t wait_ms = 0U;

    if (from == NULL || to == NULL) {
        return ARM_PHASE_MIN_INTERVAL_MS;
    }

    Arm_Map_To_Servo_Angles(from->th0, from->th1, from->th2, from->th3,
                            &from_s0, &from_s1, &from_s2, &from_s3);
    Arm_Map_To_Servo_Angles(to->th0, to->th1, to->th2, to->th3,
                            &to_s0, &to_s1, &to_s2, &to_s3);

    wait_ms = Arm_Clamp_U32(Arm_Estimate_Servo_Move_Ms(to_s0 - from_s0, ARM_SERVO_S0_MOVE_SPEED),
                            wait_ms, ARM_PHASE_MAX_INTERVAL_MS);
    wait_ms = Arm_Clamp_U32(Arm_Estimate_Servo_Move_Ms(to_s1 - from_s1, ARM_SERVO_S1_MOVE_SPEED),
                            wait_ms, ARM_PHASE_MAX_INTERVAL_MS);
    wait_ms = Arm_Clamp_U32(Arm_Estimate_Servo_Move_Ms(to_s2 - from_s2, ARM_SERVO_S2_MOVE_SPEED),
                            wait_ms, ARM_PHASE_MAX_INTERVAL_MS);
    wait_ms = Arm_Clamp_U32(Arm_Estimate_Servo_Move_Ms(to_s3 - from_s3, ARM_SERVO_S3_MOVE_SPEED),
                            wait_ms, ARM_PHASE_MAX_INTERVAL_MS);

    wait_ms += ARM_PHASE_SETTLE_EXTRA_MS;
    return Arm_Clamp_U32(wait_ms, ARM_PHASE_MIN_INTERVAL_MS, ARM_PHASE_MAX_INTERVAL_MS);
}

//+根据当前 Y 坐标计算梯形禁区的 X 半宽，单位 cm。
static float Arm_Forbidden_X_Half_Width(float y)
{
    float ratio = (y - ARM_FORBID_Y_NEAR_CM) / (ARM_FORBID_Y_FAR_CM - ARM_FORBID_Y_NEAR_CM);
    return ARM_FORBID_X_HALF_NEAR_CM +
           ratio * (ARM_FORBID_X_HALF_FAR_CM - ARM_FORBID_X_HALF_NEAR_CM);
}

//+判断吸盘末端目标是否落入梯形禁止静止到达区域。
static uint8_t Arm_Target_In_Forbidden_Zone(const ArmWorldTarget_t *target)
{
    float x_half_width = 0.0f;

    if (target == NULL) return 1U;

    if (target->y <= ARM_FORBID_Y_NEAR_CM || target->y >= ARM_FORBID_Y_FAR_CM) {
        return 0U;
    }
    if (target->z >= ARM_FORBID_Z_MAX_CM) {
        return 0U;
    }

    x_half_width = Arm_Forbidden_X_Half_Width(target->y);
    if (target->x > -x_half_width && target->x < x_half_width) {
        return 1U;
    }
    return 0U;
}

// 判断最终目标是否过于靠近车前方竖直细杆。
static uint8_t Arm_Target_Above_Forbidden_Zone(const ArmWorldTarget_t *target)
{
    float x_half_width = 0.0f;

    if (target == NULL) return 0U;

    if (target->y <= ARM_FORBID_Y_NEAR_CM || target->y >= ARM_FORBID_Y_FAR_CM) {
        return 0U;
    }
    if (target->z < (ARM_FORBID_Z_MAX_CM + ARM_FORBID_ABOVE_SAFE_MARGIN_CM)) {
        return 0U;
    }

    x_half_width = Arm_Forbidden_X_Half_Width(target->y);
    if (target->x > -x_half_width && target->x < x_half_width) {
        return 1U;
    }
    return 0U;
}

static uint8_t Arm_Target_Too_Close_To_Front_Rod(const ArmWorldTarget_t *target)
{
    if (target == NULL) return 1;

    if (Arm_Abs_Float(target->x) <= ARM_FRONT_ROD_RADIUS_CM &&
        Arm_Abs_Float(target->y - ARM_FRONT_ROD_Y_CM) <= ARM_FRONT_ROD_RADIUS_CM) {
        return 1;
    }
    return 0;
}

// 向四个舵机发送一次同步或回退模式的速度目标命令。
static void Arm_Send_Servo_Targets(float actual_s0, float actual_s1, float actual_s2, float actual_s3)
{
    uint16_t accel_time = ARM_SERVO_ACCEL_TIME_MS;
    uint16_t decel_time = ARM_SERVO_DECEL_TIME_MS;
    uint16_t power = ARM_SERVO_POWER_LIMIT;

#if ARM_USE_SYNC_SERVO_SPEED_CMD
    Sync_Adv_Single_Speed_t single_targets[3] = {
        {S0_ID, (int16_t)(actual_s0 * 10.0f), ARM_SERVO_S0_MOVE_SPEED, accel_time, decel_time, power},
        {S2_ID, (int16_t)(actual_s2 * 10.0f), ARM_SERVO_S2_MOVE_SPEED, accel_time, decel_time, power},
        {S3_ID, (int16_t)(actual_s3 * 10.0f), ARM_SERVO_S3_MOVE_SPEED, accel_time, decel_time, power}
    };
    Sync_Adv_Multi_Speed_t multi_targets[1] = {
        {S1_ID, (int32_t)(actual_s1 * 10.0f), ARM_SERVO_S1_MOVE_SPEED, accel_time, decel_time, power}
    };

    Servo_Sync_Adv_Multi_Turn_Speed(multi_targets, 1);
    Servo_Sync_Adv_Single_Turn_Speed(single_targets, 3);
#else
    Servo_Advanced_Single_Turn_Speed(S0_ID, (int16_t)(actual_s0 * 10.0f), ARM_SERVO_S0_MOVE_SPEED, accel_time, decel_time, power);
    Servo_Advanced_Multi_Turn_Speed( S1_ID, (int32_t)(actual_s1 * 10.0f), ARM_SERVO_S1_MOVE_SPEED, accel_time, decel_time, power);
    Servo_Advanced_Single_Turn_Speed(S2_ID, (int16_t)(actual_s2 * 10.0f), ARM_SERVO_S2_MOVE_SPEED, accel_time, decel_time, power);
    Servo_Advanced_Single_Turn_Speed(S3_ID, (int16_t)(actual_s3 * 10.0f), ARM_SERVO_S3_MOVE_SPEED, accel_time, decel_time, power);
#endif
}

// 在竖直平面内求解二连杆腕点逆解。
static uint8_t Arm_Wrist_Inverse_Kinematics(float plane_x, float plane_z, float *out_th1, float *out_th2)
{
    float l_eff = sqrtf(ARM_L1 * ARM_L1 + ARM_C * ARM_C);
    float phi = atan2f(ARM_C, ARM_L1);
    float dx = plane_x - ARM_R;
    float dz = plane_z - (ARM_H0 + ARM_H1);
    float d = sqrtf(dx * dx + dz * dz);
    float cos_beta = 0.0f;
    float beta = 0.0f;
    float alpha = 0.0f;
    float cos_gamma = 0.0f;
    float gamma = 0.0f;

    if (d < 0.001f) return 0;
    if (d > (l_eff + ARM_L2) || d < fabsf(l_eff - ARM_L2)) return 0;

    cos_beta = (l_eff * l_eff + d * d - ARM_L2 * ARM_L2) / (2.0f * l_eff * d);
    beta = acosf(fmaxf(-1.0f, fminf(1.0f, cos_beta)));
    alpha = atan2f(dz, dx);

    cos_gamma = (l_eff * l_eff + ARM_L2 * ARM_L2 - d * d) / (2.0f * l_eff * ARM_L2);
    gamma = acosf(fmaxf(-1.0f, fminf(1.0f, cos_gamma)));

    *out_th1 = ((alpha + beta) - phi) * 180.0f / M_PI;
    *out_th2 = (gamma - M_PI + phi) * 180.0f / M_PI;
    return 1;
}

// 求解已转换到竖直平面内的单个目标点。
static uint8_t Arm_Plane_Inverse_Kinematics(float base_th0, float plane_x, float plane_z, float suc_dir_deg,
                                            float *out_th0, float *out_th1, float *out_th2, float *out_th3)
{
    float target_th0 = Arm_Normalize_Deg(base_th0);
    float dir_rad = suc_dir_deg * M_PI / 180.0f;
    float suc_dx = sinf(dir_rad);
    float suc_dz = -cosf(dir_rad);
    float wrist_x = plane_x - ARM_L3 * suc_dx;
    float wrist_z = plane_z - ARM_L3 * suc_dz;
    float target_th1 = 0.0f;
    float target_th2 = 0.0f;
    float suc_std_dir_deg = 0.0f;
    float target_th3 = 0.0f;

    if (!Arm_Wrist_Inverse_Kinematics(wrist_x, wrist_z, &target_th1, &target_th2)) return 0;

    suc_std_dir_deg = Arm_Normalize_Deg(suc_dir_deg - 90.0f);
    target_th3 = Arm_Normalize_Deg(suc_std_dir_deg - target_th1 - target_th2);

    if (!Arm_Is_Servo_Angle_In_Range(target_th0, target_th1, target_th2, target_th3)) return 0;

    *out_th0 = target_th0;
    *out_th1 = target_th1;
    *out_th2 = target_th2;
    *out_th3 = target_th3;
    return 1;
}

// 为指定云台平面生成收敛姿态；收敛点由竖直平面内坐标宏定义，方便现场直接调节。
static uint8_t Arm_Build_Folded_Pose(float base_th0, ArmWorldTarget_t *target, ArmJointPose_t *pose)
{
    if (target == NULL || pose == NULL) return 0;

    if (!Arm_Plane_Inverse_Kinematics(base_th0,
                                      ARM_FOLD_PLANE_DEPTH_CM,
                                      ARM_FOLD_PLANE_Z_CM,
                                      ARM_FOLD_SUCKER_DIR_DEG,
                                      &pose->th0,
                                      &pose->th1,
                                      &pose->th2,
                                      &pose->th3)) {
        return 0;
    }

    if (!Arm_Is_Servo_Angle_In_Range_With_Margin(pose->th0, pose->th1, pose->th2,
                                                 pose->th3, ARM_FOLD_SAFE_MARGIN)) {
        return 0;
    }

    *target = Arm_Forward_Target_From_Pose(pose);
    if (Arm_Target_In_Forbidden_Zone(target)) return 0;
    return 1;
}

// 将世界坐标系下的吸盘末端目标解算为虚拟关节角。
uint8_t Arm_Inverse_Kinematics(float world_x, float world_y, float world_z, float suc_dir_deg,
                               float *out_th0, float *out_th1, float *out_th2, float *out_th3)
{
    ArmWorldTarget_t target = {world_x, world_y, world_z, suc_dir_deg};
    float plane_x = sqrtf(world_x * world_x + world_y * world_y);
    float base_th0 = Arm_World_To_Base_Angle(world_x, world_y);

    if (Arm_Target_In_Forbidden_Zone(&target)) return 0;

    return Arm_Plane_Inverse_Kinematics(base_th0, plane_x, world_z, suc_dir_deg,
                                        out_th0, out_th1, out_th2, out_th3);
}

// 将世界目标结构体解算为关节姿态结构体。
static uint8_t Arm_Solve_Target(const ArmWorldTarget_t *target, ArmJointPose_t *pose)
{
    if (target == NULL || pose == NULL) return 0;

    return Arm_Inverse_Kinematics(target->x, target->y, target->z, target->dir,
                                  &pose->th0, &pose->th1, &pose->th2, &pose->th3);
}

// 使用当前速度命令配置执行一个虚拟姿态。
static uint8_t Arm_Execute_Pose(const ArmJointPose_t *pose)
{
    float actual_s0 = 0.0f;
    float actual_s1 = 0.0f;
    float actual_s2 = 0.0f;
    float actual_s3 = 0.0f;

    if (pose == NULL) return 0;
    if (!Arm_Is_Servo_Angle_In_Range(pose->th0, pose->th1, pose->th2, pose->th3)) return 0;

    Arm_Map_To_Servo_Angles(pose->th0, pose->th1, pose->th2, pose->th3,
                            &actual_s0, &actual_s1, &actual_s2, &actual_s3);
    Arm_Send_Servo_Targets(actual_s0, actual_s1, actual_s2, actual_s3);
    return 1;
}

// 保留给现有测试和外部调用的直接执行接口。
void Arm_Execute_Move(float v_th0, float v_th1, float v_th2, float v_th3)
{
    ArmJointPose_t pose = {v_th0, v_th1, v_th2, v_th3};

    if (!Arm_Execute_Pose(&pose)) {
        g_arm_last_error = "angle-limited move";
    }
}

// 记录非阻塞规划器追踪的当前目标和关节姿态。
static void Arm_Record_Current_State(const ArmWorldTarget_t *target, const ArmJointPose_t *pose)
{
    if (target != NULL) g_arm_current_target = *target;
    if (pose != NULL) g_arm_current_joint = *pose;
    g_arm_current_valid = 1;
}

// 停止并锁持四个总线舵机。
static void Arm_Stop_All_Servos(void)
{
    Servo_Stop(S0_ID, SERVO_STOP_MODE_HOLD, 0);
    Servo_Stop(S1_ID, SERVO_STOP_MODE_HOLD, 0);
    Servo_Stop(S2_ID, SERVO_STOP_MODE_HOLD, 0);
    Servo_Stop(S3_ID, SERVO_STOP_MODE_HOLD, 0);
}

// 清空当前非阻塞运动计划。
static void Arm_Clear_Plan(void)
{
    memset(&g_arm_plan, 0, sizeof(g_arm_plan));
}

// 向运动计划中追加一个阶段。
static uint8_t Arm_Add_Phase(const ArmWorldTarget_t *target, const ArmJointPose_t *pose,
                             const ArmJointPose_t *from_pose, const char *name)
{
    ArmPlanPhase_t *phase = NULL;

    if (target == NULL || pose == NULL || from_pose == NULL) return 0;
    if (g_arm_plan.phase_count >= ARM_PLAN_PHASE_MAX) {
        g_arm_last_error = "phase buffer overflow";
        return 0;
    }

    phase = &g_arm_plan.phases[g_arm_plan.phase_count++];
    phase->target = *target;
    phase->pose = *pose;
    phase->wait_ms = Arm_Estimate_Phase_Wait_Ms(from_pose, pose);
    phase->name = name;
    return 1;
}

// 构建轻量运动计划：普通目标直达；跨 x=0 或目标云台角指向禁区时，先收敛、转平面、再展开。
static uint8_t Arm_Build_Motion_Plan(const ArmWorldTarget_t *target)
{
    ArmJointPose_t final_pose = {0.0f, 0.0f, 0.0f, 0.0f};
    ArmWorldTarget_t fold_current_target;
    ArmJointPose_t fold_current_pose;
    ArmWorldTarget_t fold_target_target;
    ArmJointPose_t fold_target_pose;
    ArmJointPose_t last_pose;
    float current_base = 0.0f;
    float target_base = 0.0f;
    uint8_t cross_x_zero = 0U;
    uint8_t target_base_in_forbid = 0U;
    uint8_t need_fold_path = 0U;
    uint8_t current_above_forbid = 0U;
    uint8_t target_above_forbid = 0U;

    if (target == NULL) return 0;
    Arm_Clear_Plan();
    g_arm_plan.plan_cmd_type = ARM_HOST_CMD_MOVE;

    if (Arm_Target_In_Forbidden_Zone(target)) {
        g_arm_last_error = "target in forbidden zone";
        return 0;
    }

    if (Arm_Target_Too_Close_To_Front_Rod(target)) {
        g_arm_last_error = "target too close to front rod";
        return 0;
    }

    if (!Arm_Solve_Target(target, &final_pose)) {
        g_arm_last_error = "target unreachable or angle-limited";
        return 0;
    }

    current_base = g_arm_current_valid ? g_arm_current_joint.th0 : final_pose.th0;
    target_base = final_pose.th0;
    last_pose = g_arm_current_valid ? g_arm_current_joint : final_pose;
    cross_x_zero = Arm_Target_Crosses_X_Zero(target);
    target_base_in_forbid = Arm_Base_Angle_In_Forbidden_Direction(target_base);
    need_fold_path = (cross_x_zero || target_base_in_forbid) ? 1U : 0U;
    current_above_forbid = g_arm_current_valid ?
        Arm_Target_Above_Forbidden_Zone(&g_arm_current_target) : 0U;
    target_above_forbid = Arm_Target_Above_Forbidden_Zone(target);

    if (current_above_forbid && target_above_forbid) {
        need_fold_path = 0U;
    }

    if (need_fold_path) {
        if (!Arm_Build_Folded_Pose(current_base, &fold_current_target, &fold_current_pose)) {
            g_arm_last_error = "current-plane fold pose invalid";
            return 0;
        }

        if (!Arm_Build_Folded_Pose(target_base, &fold_target_target, &fold_target_pose)) {
            g_arm_last_error = "target-plane fold pose invalid";
            return 0;
        }

        if (Arm_Max_Servo_Delta(&last_pose, &fold_current_pose) > 0.5f) {
            if (!Arm_Add_Phase(&fold_current_target, &fold_current_pose, &last_pose, "fold")) return 0;
            last_pose = fold_current_pose;
        }

        if (Arm_Abs_Float(fold_target_pose.th0 - fold_current_pose.th0) > ARM_PLANE_ROTATE_SKIP_DEG) {
            if (!Arm_Add_Phase(&fold_target_target, &fold_target_pose, &last_pose, "rotate")) return 0;
            last_pose = fold_target_pose;
        }
    }

    if (!g_arm_current_valid || Arm_Max_Servo_Delta(&last_pose, &final_pose) > 0.5f) {
        if (!Arm_Add_Phase(target, &final_pose, &last_pose, need_fold_path ? "expand" : "direct")) return 0;
    }

    g_arm_plan.final_target = *target;
    g_arm_plan.final_pose = final_pose;
    return 1;
}

// 构建默认休息计划：必要时先在当前平面收敛，再回到默认休息姿态。
static uint8_t Arm_Build_Fold_Home_Plan(void)
{
    ArmWorldTarget_t fold_current_target;
    ArmJointPose_t fold_current_pose;
    ArmWorldTarget_t fold_rest_target;
    ArmJointPose_t fold_rest_pose;
    ArmWorldTarget_t rest_target;
    ArmJointPose_t rest_pose;
    ArmJointPose_t last_pose;
    float current_base = ARM_DEFAULT_REST_BASE_DEG;

    Arm_Clear_Plan();
    g_arm_plan.plan_cmd_type = ARM_HOST_CMD_FOLD;

    current_base = g_arm_current_valid ? g_arm_current_joint.th0 : ARM_INIT_FALLBACK_BASE_DEG;
    if (!g_arm_current_valid) {
        (void)Arm_Read_Init_Base_Angle(&current_base);
    }

    if (g_arm_current_valid) {
        last_pose = g_arm_current_joint;
    } else {
        // 上电时真实臂形未知，先假定为休息臂形，只用于估算等待时间并强制生成收敛阶段。
        last_pose.th0 = current_base;
        last_pose.th1 = ARM_REST_TH1_DEG;
        last_pose.th2 = ARM_REST_TH2_DEG;
        last_pose.th3 = ARM_REST_TH3_DEG;
    }

    if (!Arm_Build_Folded_Pose(current_base, &fold_current_target, &fold_current_pose)) {
        g_arm_last_error = "current-plane fold pose invalid";
        return 0;
    }

    if (!Arm_Build_Folded_Pose(ARM_DEFAULT_REST_BASE_DEG, &fold_rest_target, &fold_rest_pose)) {
        g_arm_last_error = "rest-plane fold pose invalid";
        return 0;
    }

    if (!Arm_Build_Rest_Pose(ARM_DEFAULT_REST_BASE_DEG, &rest_target, &rest_pose)) {
        g_arm_last_error = "default rest pose invalid";
        return 0;
    }

    if (Arm_Max_Servo_Delta(&last_pose, &fold_current_pose) > 0.5f) {
        if (!Arm_Add_Phase(&fold_current_target, &fold_current_pose, &last_pose, "fold")) return 0;
        last_pose = fold_current_pose;
    }

    if (Arm_Max_Servo_Delta(&last_pose, &fold_rest_pose) > 0.5f) {
        if (!Arm_Add_Phase(&fold_rest_target, &fold_rest_pose, &last_pose, "rotate")) return 0;
        last_pose = fold_rest_pose;
    }

    if (Arm_Max_Servo_Delta(&last_pose, &rest_pose) > 0.5f) {
        if (!Arm_Add_Phase(&rest_target, &rest_pose, &last_pose, "rest")) return 0;
        last_pose = rest_pose;
    }

    g_arm_plan.final_target = rest_target;
    g_arm_plan.final_pose = rest_pose;
    return 1;
}

// 启动已构建运动计划的第一个阶段。
static uint8_t Arm_Start_Plan(void)
{
    ArmPlanPhase_t *phase = NULL;

    if (g_arm_plan.phase_count == 0U) {
        Arm_Record_Current_State(&g_arm_plan.final_target, &g_arm_plan.final_pose);
        return 1;
    }

    g_arm_plan.active = 1;
    g_arm_plan.phase_index = 0;
    phase = &g_arm_plan.phases[0];

    if (!Arm_Execute_Pose(&phase->pose)) {
        Arm_Clear_Plan();
        g_arm_last_error = "failed to start first phase";
        return 0;
    }

    g_arm_plan.phase_start_tick = HAL_GetTick();
    return 1;
}

// 为新的目标命令构建并启动非阻塞运动计划。
static uint8_t Arm_Start_Motion_To_Target(const ArmWorldTarget_t *target)
{
    if (g_arm_plan.active) {
        g_arm_last_error = "arm planner busy";
        return 0;
    }

    if (!Arm_Build_Motion_Plan(target)) return 0;
    return Arm_Start_Plan();
}

// 为默认休息命令构建并启动非阻塞运动计划。
static uint8_t Arm_Start_Fold_Home(void)
{
    if (!Arm_Build_Fold_Home_Plan()) return 0;
    return Arm_Start_Plan();
}

// 根据系统 tick 推进非阻塞运动计划。
static void Arm_Update_Plan(void)
{
    ArmPlanPhase_t *phase = NULL;

    if (!g_arm_plan.active) return;

    phase = &g_arm_plan.phases[g_arm_plan.phase_index];
    if ((uint32_t)(HAL_GetTick() - g_arm_plan.phase_start_tick) < phase->wait_ms) return;

    Arm_Record_Current_State(&phase->target, &phase->pose);
    g_arm_plan.phase_index++;

    if (g_arm_plan.phase_index >= g_arm_plan.phase_count) {
        Arm_Record_Current_State(&g_arm_plan.final_target, &g_arm_plan.final_pose);
        g_arm_status_event = (g_arm_plan.plan_cmd_type == ARM_HOST_CMD_FOLD) ?
            ARM_STATUS_EVENT_FOLD_DONE : ARM_STATUS_EVENT_DONE;
        Arm_Clear_Plan();
        return;
    }

    phase = &g_arm_plan.phases[g_arm_plan.phase_index];
    if (!Arm_Execute_Pose(&phase->pose)) {
        Arm_Stop_All_Servos();
        g_arm_last_error = "failed to execute next phase";
        g_arm_status_event = ARM_STATUS_EVENT_REJECTED;
        Arm_Clear_Plan();
        return;
    }

    g_arm_plan.phase_start_tick = HAL_GetTick();
}

// 初始化总线舵机通信，并按“收敛、转平面、休息”的规划进入默认休息姿态。
void Arm_Init(void)
{
#if ARM_INIT_USE_PLANNER_HOME
    Servo_Init();
    Arm_Clear_Plan();
    g_arm_current_valid = 0U;
    g_arm_status_event = ARM_STATUS_EVENT_NONE;
    g_arm_last_error = "OK";

    if (!Arm_Start_Fold_Home()) {
        g_arm_status_event = ARM_STATUS_EVENT_REJECTED;
    }
#else
    ArmWorldTarget_t home_target;
    ArmJointPose_t home_pose;

    Servo_Init();
    Arm_Clear_Plan();
    g_arm_current_valid = 0U;
    g_arm_status_event = ARM_STATUS_EVENT_NONE;
    g_arm_last_error = "OK";

    if (Arm_Build_Rest_Pose(ARM_DEFAULT_REST_BASE_DEG, &home_target, &home_pose)) {
        if (Arm_Execute_Pose(&home_pose)) {
            Arm_Record_Current_State(&home_target, &home_pose);
        } else {
            g_arm_last_error = "failed to execute direct init pose";
            g_arm_status_event = ARM_STATUS_EVENT_REJECTED;
        }
    } else {
        g_arm_last_error = "default rest pose invalid";
        g_arm_status_event = ARM_STATUS_EVENT_REJECTED;
    }
#endif
}

// 打印固定测试点的逆解结果，不移动舵机也不阻塞。
void Arm_Standalone_Test(void)
{
    float v_th0 = 0.0f;
    float v_th1 = 0.0f;
    float v_th2 = 0.0f;
    float v_th3 = 0.0f;
    float test_points[4][4] = {
        {  0.0f, 23.0f, 45.0f, 115.0f},
        { 16.3f, 16.3f, 45.0f, 115.0f},
        {-15.6f, 15.6f, 46.0f, 118.0f},
        { 35.0f,  0.0f, 35.0f,  94.0f}
    };

    printf("\r\n[ARM TEST] IK-only routine...\r\n");

    for (int i = 0; i < 4; i++) {
        if (Arm_Inverse_Kinematics(test_points[i][0], test_points[i][1],
                                   test_points[i][2], test_points[i][3],
                                   &v_th0, &v_th1, &v_th2, &v_th3)) {
            printf("[ARM TEST] P%d -> th0=%.1f th1=%.1f th2=%.1f th3=%.1f\r\n",
                   i + 1, v_th0, v_th1, v_th2, v_th3);
        } else {
            printf("[ARM TEST] P%d unreachable\r\n", i + 1);
        }
    }
}

// 机械臂主任务轮询函数，需要在 MCU 主循环中反复调用。
void Arm_Task_Loop(void)
{
    if (g_HostCmd.new_cmd_flag == 1 && g_HostCmd.cmd_type == ARM_HOST_CMD_STOP) {
        g_HostCmd.new_cmd_flag = 0;
        Arm_Stop_All_Servos();
        Arm_Clear_Plan();
        g_arm_plan.plan_cmd_type = ARM_HOST_CMD_STOP;
        g_arm_last_error = "OK";
        g_arm_status_event = ARM_STATUS_EVENT_STOPPED;
        return;
    }

    if (g_HostCmd.new_cmd_flag == 1 && g_HostCmd.cmd_type == ARM_HOST_CMD_FOLD) {
        g_HostCmd.new_cmd_flag = 0;
        g_arm_last_error = "OK";
        if (Arm_Start_Fold_Home()) {
            if (!g_arm_plan.active) {
                g_arm_status_event = ARM_STATUS_EVENT_FOLD_DONE;
            }
        } else {
            g_arm_status_event = ARM_STATUS_EVENT_REJECTED;
        }
        return;
    }

    Arm_Update_Plan();

    if (g_HostCmd.new_cmd_flag != 1) return;
    g_HostCmd.new_cmd_flag = 0;

    if (g_HostCmd.cmd_type != ARM_HOST_CMD_MOVE) return;

    {
        ArmWorldTarget_t target = {
            g_HostCmd.world_x,
            g_HostCmd.world_y,
            g_HostCmd.world_z,
            g_HostCmd.suc_dir_deg
        };

        g_arm_last_error = "OK";
        if (Arm_Start_Motion_To_Target(&target)) {
            if (!g_arm_plan.active) {
                g_arm_status_event = ARM_STATUS_EVENT_DONE;
            }
        } else {
            g_arm_status_event = ARM_STATUS_EVENT_REJECTED;
        }
    }
}

//+获取机械臂规划器是否正在执行。
uint8_t Arm_Is_Planner_Active(void)
{
    return g_arm_plan.active;
}

//+读取机械臂当前逻辑末端位置和关节角。
void Arm_Get_Logical_Pose(float *x, float *y, float *z, float *dir,
                          float *th0, float *th1, float *th2, float *th3)
{
    if (x != NULL) *x = g_arm_current_target.x;
    if (y != NULL) *y = g_arm_current_target.y;
    if (z != NULL) *z = g_arm_current_target.z;
    if (dir != NULL) *dir = g_arm_current_target.dir;
    if (th0 != NULL) *th0 = g_arm_current_joint.th0;
    if (th1 != NULL) *th1 = g_arm_current_joint.th1;
    if (th2 != NULL) *th2 = g_arm_current_joint.th2;
    if (th3 != NULL) *th3 = g_arm_current_joint.th3;
}

//+读取并清除机械臂内部事件。
uint8_t Arm_Fetch_Status_Event(uint8_t *event)
{
    if (event == NULL) return 0U;
    if (g_arm_status_event == ARM_STATUS_EVENT_NONE) return 0U;

    *event = g_arm_status_event;
    g_arm_status_event = ARM_STATUS_EVENT_NONE;
    return 1U;
}
