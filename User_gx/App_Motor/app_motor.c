#include "app_motor.h"
#include "task.h"
#include "app_encoder.h"
#include "JY901S.h"
#include <math.h>

#include "app_speed.h"

static PID_TypeDef pid_position_mid;  // 中间位置环 PID，输出基础行驶速度
static PID_TypeDef pid_angle_yaw;     // 偏航角环 PID，输出转向修正速度

// 原地转向保留连续剩余角度，避免180°附近的反馈噪声反复改变转向。
static float turn_error;
static float turn_last_yaw;
static float turn_target;
static uint8_t turn_error_valid;

float motor_left_speed = 0.0f;        // 左电机速度目标，单位：cm/s
float motor_right_speed = 0.0f;       // 右电机速度目标，单位：cm/s

int16_t motor_encoder1 = 0;           // 左编码器累计计数
int16_t motor_encoder2 = 0;           // 右编码器累计计数

float position_l = 0.0f;              // 左轮累计位移，单位：cm
float position_r = 0.0f;              // 右轮累计位移，单位：cm
float position_mid = 0.0f;            // 左右轮平均位移，单位：cm
float position_mid_ref = 0.0f;        // 中间位置环目标位移，单位：cm

float angle_yaw = 0.0f;               // 陀螺仪实际偏航角，单位：度
float angle_yaw_ref = 0.0f;           // 角度环实际使用的目标角，单位：度
float angle_yaw_target = 0.0f;        // 调参输入的原始绝对目标角，单位：度

float motor_base_speed = 0.0f;        // 位置环输出的基础速度，单位：cm/s
float motor_turn_speed = 0.0f;        // 角度环输出的转向速度，单位：cm/s

volatile uint8_t motor_pos_done = 1;  // 位置控制完成标志，1：完成，0：未完成
volatile uint8_t motor_angle_done = 1; // 角度控制完成标志，1：完成，0：未完成

/*
 * @简介：计算浮点数的绝对值。
 * @参数：x 输入的浮点数。
 * @返回：x 的绝对值。
 */
static float App_Motor_Abs(float x)
{
    if(x < 0.0f){
        return -x;
    }

    return x;
}

/*
 * @简介：把角度限制到 [-180°, 180°) 范围。
 * @参数：angle 待处理的角度，单位：度。
 * @返回：归一化后的角度，单位：度。
 *
 * 将角度统一限制到 [-180°, 180°) 范围。
 *
 * JY901S 的角度在跨过 180° 时会从接近 +180° 变成接近 -180°。
 * 这两个数值表示的是同一个方向，不能直接把它们相减，否则会
 * 得到接近 360° 的错误角度误差。
 */
static float App_Motor_NormalizeAngle(float angle)
{
    while(angle >= 180.0f){
        angle -= 360.0f;
    }

    while(angle < -180.0f){
        angle += 360.0f;
    }

    return angle;
}

/*
 * @简介：计算两个偏航角之间的最短方向误差。
 * @参数：target 目标角度，单位：度。
 * @参数：current 当前角度，单位：度。
 * @返回：从 current 转到 target 的最短角度误差，单位：度。
 */
static float App_Motor_AngleError(float target, float current)
{
    float err = target - current;      // 未归一化的角度差

    /*
     * 角度误差也必须归一化，这样小车始终沿最短方向转动。
     * 例如目标 179°、当前 -179° 时，误差应为 -2°，而不是 358°。
     */
    return App_Motor_NormalizeAngle(err);
}

// 下发新目标时只选择一次最短方向；后续按实际航向增量扣减剩余角度。
static void App_Motor_TurnBegin(void)
{
    turn_target = angle_yaw_ref;
    turn_last_yaw = angle_yaw;
    turn_error = App_Motor_AngleError(turn_target, turn_last_yaw);
    turn_error_valid = 1;
}

static float App_Motor_TurnError(void)
{
    // 兼容调试器直接修改目标值。跨越IMU的±180°时也只累计实际小角度。
    if(!turn_error_valid || turn_target != angle_yaw_ref){
        App_Motor_TurnBegin();
    }
    turn_error -= App_Motor_AngleError(angle_yaw, turn_last_yaw);
    turn_last_yaw = angle_yaw;
    return turn_error;
}

/*
 * @简介：根据位置误差计算基础行驶速度。
 * @参数：err 位置误差，单位：cm，正负表示行驶方向。
 * @返回：基础行驶速度，单位：cm/s。
 */
static float App_Motor_PositionSpeed(float err)
{
    float speed = 0.0f;                // 计算得到的基础速度
    float err_abs = App_Motor_Abs(err);// 位置误差绝对值

    // 距离较远时匀速走，快到目标时再按剩余距离减速。
    if(err_abs > 18.0f){
        speed = 40.0f;
    }else {
        speed = 40.0f * err_abs / 18.0f;

        if(speed < 20.0f){
            speed = 20.0f;
        }
    }

    if(err < 0.0f){
        speed = -speed;
    }

    return speed;
}

/*
 * @简介：计算偏航角 PID 输出。
 * @参数：PID 角度环 PID 结构体指针。
 * @参数：err 当前角度误差，单位：度；原地转向传连续误差，位置保持传最短误差。
 * @返回：角度环输出的转向速度，单位：cm/s。
 */
static float App_Motor_PIDComputeAngle(PID_TypeDef *PID, float err)
{
    uint32_t now_ms = HAL_GetTick();               // 当前系统时间，单位：ms
    uint64_t t_k = 0;                              // 保存本次 PID 计算时间
    uint32_t delta_ms = 0;                         // 两次计算的时间间隔，单位：ms
    float deltaT = 0.0f;                           // 两次计算的时间间隔，单位：s
    float err_dev = 0.0f;                          // 误差微分项
    float err_int = 0.0f;                          // 误差积分项
    float COp = 0.0f;                              // PID 比例项输出
    float COi = 0.0f;                              // PID 积分项输出
    float COd = 0.0f;                              // PID 微分项输出
    float CO = 0.0f;                               // PID 总输出

    t_k = now_ms;
    delta_ms = now_ms - (unsigned long)PID->t_k_1;
    deltaT = delta_ms * 1.0e-3f;

    if(deltaT <= 0.0f){
        deltaT = 1.0e-3f;
    }

    if(PID->t_k_1 != 0){
        //积分分离
        /*
         * 误差本身是环形角度。若误差从 179° 变成 -179°，
         * 实际只变化了 2°，不能直接相减得到 -358°，否则
         * 微分项会瞬间产生很大的错误输出。
         */
        float err_delta = App_Motor_AngleError(err, PID->err_k_1); // 相邻两次的环形误差变化量
        err_dev = err_delta / deltaT;

        if(fabsf(err) < 15.0f){
            err_int = PID->err_int_k_1 + (err + PID->err_k_1) * deltaT * 0.5f;
        }else {
            err_int = 0.0f;
        }
    }

    COp = PID->Kp * err;
    COi = PID->Ki * err_int;
    COd = PID->Kd * err_dev;
    CO = COp + COi + COd;

    PID->t_k_1 = t_k;
    PID->err_k_1 = err;
    PID->err_int_k_1 = err_int;

    if(CO > PID->UpperLimit){
        CO = PID->UpperLimit;
    }

    if(CO < PID->LowerLimit){
        CO = PID->LowerLimit;
    }

    if(PID->err_int_k_1 > PID->UpperLimit){
        PID->err_int_k_1 = PID->UpperLimit;
    }

    if(PID->err_int_k_1 < PID->LowerLimit){
        PID->err_int_k_1 = PID->LowerLimit;
    }

    return CO;
}

void App_Update_Data(void)
{
    /*
     * @简介：更新编码器、位置和陀螺仪反馈数据。
     * @参数：无。
     * @返回：无。
     *
     * 读取 JY901S 的校准后偏航角，作为角度环反馈值。
     * IMU_GetAnglesCalibrated() 允许只读取 yaw，因此前两个参数
     * 传 NULL。再次归一化是为了保证进入 PID 的角度始终处于
     * [-180°, 180°) 范围。
     */
    IMU_GetAnglesCalibrated(NULL, NULL, &angle_yaw);
    angle_yaw = App_Motor_NormalizeAngle(angle_yaw);

    motor_encoder1 = App_Encoder_GetEncoder_L();
    motor_encoder2 = App_Encoder_GetEncoder_R();

    position_l = App_Encoder_GetPos_L() / 360.0f * PI * (WHEEL_DIAMETER / 10.0f);
    position_r = App_Encoder_GetPos_R() / 360.0f * PI * (WHEEL_DIAMETER / 10.0f);
    position_mid = (position_l + position_r) * 0.5f;
}

/*
 * @简介：初始化位置环、角度环以及相关运行状态。
 * @参数：无。
 * @返回：无。
 */
void App_Motor_Init(void)
{
    App_Update_Data();

    PID_Init(&pid_position_mid, 0.0f, 0.0f, 0.0f);
    PID_LimitConfig(&pid_position_mid, 70.0f, -70.0f);

    PID_Init(&pid_angle_yaw, 0.9f, 0.0f, 0.0f);
    /*
     * 当前角度环使用 Kp=0.9、Ki=0、Kd=0，后续可以根据小车响应继续调参。
     */
    PID_LimitConfig(&pid_angle_yaw, 70.0f, -70.0f);

    position_mid_ref = position_mid;
    angle_yaw_ref = App_Motor_NormalizeAngle(angle_yaw);
    angle_yaw_target = angle_yaw_ref;
    PID_ChangeSP(&pid_position_mid, position_mid_ref);
    PID_ChangeSP(&pid_angle_yaw, angle_yaw_ref);

    motor_left_speed = 0.0f;
    motor_right_speed = 0.0f;
    motor_base_speed = 0.0f;
    motor_turn_speed = 0.0f;

    motor_pos_done = 1;
    motor_angle_done = 1;
    turn_error_valid = 0;
}

/*
 * @简介：复位位置环、角度环和速度环的历史状态。
 * @参数：无。
 * @返回：无。
 */
void App_PID_Reset(void)
{
    PID_Reset(&pid_position_mid);
    PID_Reset(&pid_angle_yaw);
    App_Speed_Reset();
    turn_error_valid = 0;
}

/*
 * @简介：停止位置和角度控制，并清零相关输出状态。
 * @参数：无。
 * @返回：无。
 */
void App_Position_Angle_Reset(void)
{
    App_PID_Reset();

    motor_left_speed = 0.0f;
    motor_right_speed = 0.0f;
    motor_base_speed = 0.0f;
    motor_turn_speed = 0.0f;

    motor_pos_done = 1;
    motor_angle_done = 1;
}

/*
 * @简介：设置相对当前偏航角的角度目标。
 * @参数：angle_ref 相对当前角度的变化量，单位：度。
 * @返回：无。
 */
void Set_Angle_SP(float angle_ref)
{
    App_Update_Data();

    /*
     * angle_ref 表示相对当前角度的目标变化量。
     * 例如当前为 170°，传入 30°，目标应为 -160°。
     */
    angle_yaw_ref = App_Motor_NormalizeAngle(angle_yaw + angle_ref);
    angle_yaw_target = angle_yaw_ref;
    PID_ChangeSP(&pid_angle_yaw, angle_yaw_ref);
    PID_Reset(&pid_angle_yaw);

    App_Motor_TurnBegin();

    motor_angle_done = 0;
}

/*
 * @简介：设置陀螺仪坐标系下的绝对角度目标。
 * @参数：angle_target 绝对目标角度，单位：度。
 * @返回：无。
 */
void Set_Angle_Target(float angle_target)
{
    App_Update_Data();
    /*
     * 调参时直接使用上位机发送的绝对角度，不再叠加当前角度。
     * angle_yaw_target 保留发送值，用于串口输出对比；
     * angle_yaw_ref 只负责把目标限制到陀螺仪使用的角度范围。
     */
    angle_yaw_target = angle_target;
    angle_yaw_ref = App_Motor_NormalizeAngle(angle_target);

    PID_ChangeSP(&pid_angle_yaw, angle_yaw_ref);
    PID_Reset(&pid_angle_yaw);

    App_Motor_TurnBegin();

    motor_angle_done = 0;
}

/*
 * @简介：设置相对当前位置的位置目标，并保持当前偏航角。
 * @参数：position_ref 相对当前位置的位移目标，单位：cm。
 * @返回：无。
 */
void Set_Position_SP(float position_ref)
{
    App_Update_Data();
    turn_error_valid = 0;

    position_mid_ref = position_mid + position_ref;
    angle_yaw_ref = App_Motor_NormalizeAngle(angle_yaw);
    angle_yaw_target = angle_yaw_ref;
    PID_ChangeSP(&pid_position_mid, position_mid_ref);
    PID_ChangeSP(&pid_angle_yaw, angle_yaw_ref);
    PID_Reset(&pid_position_mid);
    PID_Reset(&pid_angle_yaw);

    motor_pos_done = 0;
    motor_angle_done = 0;
}

/*
 * @简介：同时设置相对当前位置和相对当前角度的目标。
 * @参数：position_ref 相对当前位置的位移目标，单位：cm。
 * @参数：angle_ref 相对当前角度的变化量，单位：度。
 * @返回：无。
 */
void Set_Position_Angle_SP(float position_ref, float angle_ref)
{
    App_Update_Data();
    turn_error_valid = 0;

    position_mid_ref = position_mid + position_ref;
    /*
     * angle_ref 与 Set_Angle_SP() 保持相同含义，表示相对当前
     * 航向角的变化量，而不是直接使用的绝对航向角。
     */
    angle_yaw_ref = App_Motor_NormalizeAngle(angle_yaw + angle_ref);
    angle_yaw_target = angle_yaw_ref;
    PID_ChangeSP(&pid_position_mid, position_mid_ref);
    PID_ChangeSP(&pid_angle_yaw, angle_yaw_ref);
    PID_Reset(&pid_position_mid);
    PID_Reset(&pid_angle_yaw);

    motor_pos_done = 0;
    motor_angle_done = 0;
}

/*
 * @简介：执行角度环控制，并驱动小车原地转向。
 * @参数：无。
 * @返回：无。
 */
void App_Angle_Pro(void)
{
    App_Angle_Pro_Limited(1.0f, 0.0f);
}

// @简介：沿用角度PID；到位范围内停车，范围外的单轮速度不低于指定下限。
void App_Angle_Pro_Limited(float tolerance_deg, float min_speed)
{
    App_Update_Data();
    PERIODIC_START(ANGLE,15)

    /*
     * angle_yaw_ref 可能由调试器直接修改，因此每次角度环运行时
     * 都同步到 PID 的设定值，确保实际计算使用最新目标角度。
     */
    PID_ChangeSP(&pid_angle_yaw, angle_yaw_ref);
    float err = App_Motor_TurnError();

    if(App_Motor_Abs(err) <= tolerance_deg){
        motor_angle_done = 1;
        motor_turn_speed = 0.0f;
        App_Speed_Set(0.0f, 0.0f);
        PID_Reset(&pid_angle_yaw);
    }else {
        motor_angle_done = 0;
        motor_turn_speed = App_Motor_PIDComputeAngle(&pid_angle_yaw, err);

        if(App_Motor_Abs(motor_turn_speed) < min_speed){
            motor_turn_speed = err > 0.0f ? min_speed : -min_speed;
        }

        App_Speed_Set(-motor_turn_speed, +motor_turn_speed);
    }
    PERIODIC_END

    App_Speed_Pro();
}

/*
 * @简介：执行位置环和角度环联合控制。
 * @参数：无。
 * @返回：无。
 */
void App_Position_Pro(void)
{
    PERIODIC_START(POSITION,30)
    App_Update_Data();

    if(App_Motor_Abs(position_mid_ref - position_mid) <= 1.0f
       && App_Motor_Abs(App_Motor_AngleError(angle_yaw_ref, angle_yaw)) <= 0.7f){
        motor_pos_done = 1;
        motor_base_speed = 0.0f;
        motor_left_speed = 0.0f;
        motor_right_speed = 0.0f;
        motor_turn_speed = 0.0f;
        App_Speed_Set(0.0f, 0.0f);
        PID_Reset(&pid_position_mid);
        PID_Reset(&pid_angle_yaw);
    }else {
        motor_pos_done = 0;
        /*
         * 位置已经到达但角度还没有到达时，停止前进，只保留
         * 原地转向，避免位置误差很小时仍以最小速度向前移动。
         */
        if(App_Motor_Abs(position_mid_ref - position_mid) <= 1.0f){
            motor_base_speed = 0.0f;
        }else {
            motor_base_speed = App_Motor_PositionSpeed(position_mid_ref - position_mid);
        }

        if(App_Motor_Abs(App_Motor_AngleError(angle_yaw_ref, angle_yaw)) <= 0.7f){
            motor_angle_done = 1;
            motor_turn_speed = 0.0f;
        }else {
            motor_angle_done = 0;
            motor_turn_speed = App_Motor_PIDComputeAngle(&pid_angle_yaw,
                                  App_Motor_AngleError(angle_yaw_ref, angle_yaw));
        }

        motor_left_speed = motor_base_speed - motor_turn_speed;
        motor_right_speed = motor_base_speed + motor_turn_speed;
        App_Speed_Set(motor_left_speed, motor_right_speed);
    }
    PERIODIC_END

    App_Speed_Pro();
}

