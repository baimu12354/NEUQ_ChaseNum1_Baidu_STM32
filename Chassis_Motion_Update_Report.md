# 底盘运动功能修改报告

本次修改只完成底盘运动功能层，没有增加新的协议层命令，也没有修改上位机协议。外部仍然通过 `CN_set_movement(movement, movement_type)` 设置运动模式。

## 1. 修改文件

### `Code/ChaseNum1_Movement.c`

重写并整理底盘运动控制逻辑，主要内容包括：

- 完善 `break / run / rotation / translation` 四类运动状态。
- 增加 run 速度目标斜坡。
- 完成定点定向平移功能。
- 优化 rotation 到达目标后的自动回 run。
- 优化 break 状态下的闭环抱死和放开控制衔接。
- 所有新增注释均使用中文。

### `USER/main.c`

恢复了 1ms 底盘控制回调：

```c
void CN_PIT1_CALLBACK(void){
    CN_Movement_ctrl_CALLBACK(1);
}
```

原因：如果不调用 `CN_Movement_ctrl_CALLBACK(1)`，底盘状态机和闭环控制不会周期运行，translation / rotation / break 都无法真正执行。

## 2. 当前状态机行为

### 2.1 `Movement_Breaking`

语义：停车，并最终放开控制。

执行过程：

1. 收到 break 命令后，记录当前 `YAW_now` 到 `YAW_aim`。
2. 设置车身速度目标为 0。
3. 清空角速度环和四轮速度环 PID。
4. 进入闭环抱死阶段，四轮速度目标为 0，同时保持当前车身姿态。
5. 当四轮速度连续低于阈值，或闭环抱死超过最大时间后，清空 PWM 输出并放开控制。
6. 放开控制后持续令 `YAW_aim = YAW_now`，避免人工转动车身后再次 run 时猛地转回旧角度。

相关宏：

```c
#define CHASSIS_BRAKE_CLOSED_LOOP_ENABLE   1U
#define CHASSIS_BRAKE_STILL_VEL_EPS        1.0f
#define CHASSIS_BRAKE_STILL_CONFIRM_MS     100U
#define CHASSIS_BRAKE_MAX_HOLD_MS          1000U
```

### 2.2 `Movement_Running`

语义：正常闭环运行，可用于循迹、全向平移、连续转向。

执行过程：

1. 根据 `Car_X_vel` 和 `Car_Y_vel` 设置车身平移速度目标。
2. 根据 `Car_angvel` 逐步积分 `YAW_aim`。
3. 角度环根据 `YAW_now` 和 `YAW_aim` 输出目标角速度。
4. 角速度环根据 IMU 角速度输出旋转修正量。
5. 速度环把平移速度和旋转修正量合成为四轮速度。

run 状态不是放开控制。即使速度为 0，也会保持车身姿态。

### 2.3 `Movement_Rotation`

语义：相对当前姿态旋转指定角度，到达后自动进入 run。

输入字段：

```c
movement.Car_angle   // 相对旋转角度，单位 deg，可正可负
movement.Car_angvel  // 旋转最大角速度
```

执行过程：

1. 进入 rotation 时记录：
   - `YAW_inital = YAW_now`
   - `YAW_pre = YAW_now`
   - `YAW_gain = 0`
   - `target_yaw = movement.Car_angle`
2. 每周期用 `YAW_now - YAW_pre` 累加真实旋转角度，处理了 ±180° 跳变。
3. 根据剩余角度决定角速度：
   - 剩余角度大：按最大角速度转。
   - 接近目标：线性减速。
   - 低速段保留最小角速度，避免静摩擦导致转不动。
4. 到达角度阈值且角速度足够小时，自动进入 run 零速度姿态保持。

相关宏：

```c
#define CHASSIS_ROTATION_ANGLE_TOL_DEG   1.0f
#define CHASSIS_ROTATION_ANGVEL_TOL      0.03f
#define CHASSIS_ROTATION_SLOW_ANGLE_DEG  20.0f
#define CHASSIS_ROTATION_MIN_ANGVEL      0.12f
#define CHASSIS_ROTATION_DEFAULT_ANGVEL  0.5f
```

### 2.4 `Movement_Translation`

语义：相对当前位置定点定向定速平移，并保持进入命令时的车身姿态。

输入字段：

```c
movement.Car_X_movement     // 相对 X 位移
movement.Car_Y_movement     // 相对 Y 位移
movement.Car_translation_vel // 平移最大速度
```

注意：本次按你的要求没有做横向纠偏，完全相信四个轮子的累计合成位移。

执行过程：

1. 进入 translation 时记录当前姿态：

```c
initial_yaw = YAW_now;
YAW_aim = initial_yaw;
```

2. 清零相对位移：

```c
cur_x = 0;
cur_y = 0;
```

3. 根据目标点计算目标方向单位向量：

```c
distance = sqrtf(x*x + y*y);
dir_x = x / distance;
dir_y = y / distance;
```

4. 每 1ms 根据四轮编码器增量合成位移：

```c
delta_x = (FL - FR - BL + BR) / 4 / M_TO_ENC_FACTOR;
delta_y = (FL + FR + BL + BR) / 4 / M_TO_ENC_FACTOR;
cur_x += delta_x;
cur_y += delta_y;
```

5. 只看沿目标方向的投影进度：

```c
progress = cur_x * dir_x + cur_y * dir_y;
distance_to_go = total_distance - progress;
```

6. 速度规划：
   - 中段按 `Car_translation_vel` 走。
   - 进入 `CHASSIS_TRANSLATION_SLOW_DISTANCE` 后线性减速。
   - 速度变化受 `CHASSIS_TRANSLATION_ACC` 限制。
   - 未进入目标容差前保留 `CHASSIS_TRANSLATION_MIN_MOVE_SPEED`，用于克服轮子死区。

7. 到达目标后自动进入 run 零速度姿态保持，不进入 break。

相关宏：

```c
#define CHASSIS_TRANSLATION_POS_TOL          3.0f
#define CHASSIS_TRANSLATION_REACH_VEL_EPS    1.0f
#define CHASSIS_TRANSLATION_REACH_CONFIRM_MS 50U
#define CHASSIS_TRANSLATION_SLOW_DISTANCE    80.0f
#define CHASSIS_TRANSLATION_ACC              150.0f
#define CHASSIS_TRANSLATION_MIN_MOVE_SPEED   5.0f
#define CHASSIS_TRANSLATION_DEFAULT_SPEED    15.0f
#define CHASSIS_TRANSLATION_MAX_SPEED        50.0f
```

## 3. 速度斜坡说明

run 和 translation 都会经过车身速度斜坡：

```c
#define CHASSIS_VEL_RAMP_ENABLE  1U
#define CHASSIS_VEL_RAMP_ACC     200.0f
```

作用：

- 避免速度目标从 0 瞬间跳到大速度。
- 避免 run、translation、break 之间切换时车身突然抽动。

注意：

你提到轮子死区较大，这确实会导致很低速度的斜坡段实际不动。因此 translation 内部额外保留了：

```c
#define CHASSIS_TRANSLATION_MIN_MOVE_SPEED   5.0f
```

也就是没有进入目标容差前，目标速度不会低于这个值。这样可以避免速度规划在末段太小，轮子因为死区完全不动。

如果实车低速仍然卡住，优先调大：

```c
CHASSIS_TRANSLATION_MIN_MOVE_SPEED
```

如果起步或停车太猛，再调小：

```c
CHASSIS_TRANSLATION_ACC
CHASSIS_VEL_RAMP_ACC
```

## 4. 使用示例

### 4.1 正常 run

```c
Car_Movement_struct movement = {0};

movement.Car_X_vel = 10.0f;
movement.Car_Y_vel = 0.0f;
movement.Car_angvel = 0.0f;

CN_set_movement(movement, Movement_Running);
```

效果：车身向 X 正方向平移，并保持姿态。

### 4.2 原地旋转 90°

```c
Car_Movement_struct movement = {0};

movement.Car_angle = 90.0f;
movement.Car_angvel = 0.5f;

CN_set_movement(movement, Movement_Rotation);
```

效果：车身相对当前姿态旋转 90°，到达后自动进入 run 零速度姿态保持。

### 4.3 向前定点平移

```c
Car_Movement_struct movement = {0};

movement.Car_X_movement = 0.0f;
movement.Car_Y_movement = 200.0f;
movement.Car_translation_vel = 20.0f;

CN_set_movement(movement, Movement_Translation);
```

效果：车身保持当前姿态，沿 Y 正方向运动 200 个内部位移单位，到达后自动进入 run 零速度姿态保持。

### 4.4 斜向定点平移

```c
Car_Movement_struct movement = {0};

movement.Car_X_movement = 100.0f;
movement.Car_Y_movement = 100.0f;
movement.Car_translation_vel = 20.0f;

CN_set_movement(movement, Movement_Translation);
```

效果：车身保持当前姿态，沿目标点方向直线平移。代码不会做额外横向纠偏，只相信编码器合成位移。

### 4.5 break

```c
Car_Movement_struct movement = {0};

CN_set_movement(movement, Movement_Breaking);
```

效果：任意状态下切入 break，先闭环抱死轮胎，确认静止或超时后放开控制。

## 5. 调试建议

### 5.1 定点平移距离不准

优先检查：

- 四轮编码器方向是否与 `CN_CAR_vel_CL_CALLBACK()` 的轮速合成一致。
- `M_TO_ENC_FACTOR` 是否需要根据真实轮径、减速比、编码器线数换算。
- `Car_X_movement / Car_Y_movement` 的单位是否和当前编码器积分单位一致。

### 5.2 到目标点附近来回振荡

优先调：

```c
CHASSIS_TRANSLATION_POS_TOL
CHASSIS_TRANSLATION_SLOW_DISTANCE
CHASSIS_TRANSLATION_REACH_CONFIRM_MS
```

建议：

- 振荡明显：增大 `POS_TOL`，增大 `SLOW_DISTANCE`。
- 到达判定太慢：减小 `REACH_CONFIRM_MS`。

### 5.3 末段速度太低导致不动

优先调大：

```c
CHASSIS_TRANSLATION_MIN_MOVE_SPEED
```

如果加大后目标附近冲过头，再适当增大：

```c
CHASSIS_TRANSLATION_POS_TOL
CHASSIS_TRANSLATION_SLOW_DISTANCE
```

### 5.4 break 滑行太远

优先检查闭环控制回调是否在运行：

```c
CN_Movement_ctrl_CALLBACK(1);
```

然后调：

```c
CHASSIS_BRAKE_STILL_VEL_EPS
CHASSIS_BRAKE_STILL_CONFIRM_MS
CHASSIS_BRAKE_MAX_HOLD_MS
```

如果抱死时间太短，增大 `CHASSIS_BRAKE_MAX_HOLD_MS`。

## 6. 编译验证

已使用 Keil ARMCC 单文件编译：

```text
Code/ChaseNum1_Movement.c: 0 errors
USER/main.c: 0 errors
```

剩余 warning 来自原工程 `ChaseNum1_Typedef.h` 中的 `#pragma region / endregion`，ARMCC 不识别该 pragma，但不影响本次修改。
