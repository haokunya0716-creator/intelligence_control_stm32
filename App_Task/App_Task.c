//
// Created by gaoxu on 2026/10/13.
//

#include "App_Task.h"
#include "app_motor.h"
#include "app_speed.h"
#include "app_servo.h"
#include "rescue_usart2.h"

typedef enum {
    TASK_IDLE = 0,    // 等待按键
    TASK_FORWARD,     // 启动直行
    TASK_TURN_RIGHT,
    TASK_TURN_LEFT,
    TASK_DETECT_RIGHT,
    TASK_DETECT_LEFT,
    TASK_RESELECT,    // 分离动作结束后，原地重新选目标
    TASK_ALIGN,       // 先原地对准锁定的绿色物块
    TASK_APPROACH,    // 边前进边对准
    TASK_SCATTER,     // 停在准备距离，等待分离接口
    TASK_EXECUTE      // 单独绿色已接近，后续转运留空
} App_Task_State;

static App_Task_State task_state = TASK_IDLE;
static uint32_t task_time;          // 当前步骤开始时间，ms
static uint32_t round_time;         // 本轮第一次按键时间，重试不重置
static uint8_t round_started;
static uint16_t view_id;
static uint32_t view_time;
static uint8_t view_sent;
static RescueScene task_scene;     // 原始像素坐标，禁止当作厘米
static RescueObject target;        // 上次匹配到的绿色物块，不保存列表下标
static uint16_t target_image_w;
static uint16_t target_image_h;
static uint16_t last_frame_id;
static uint32_t last_capture_ms;
static uint8_t target_mixed;        // 0：单独接近，1：只走到分离准备距离
static uint8_t correcting;          // 双门限死区的修正开关

static float App_Task_Abs(float value)
{
    return value < 0.0f ? -value : value;
}

// 换算到 640×480 做图像判断；task_scene 本身保持原始值。
static float App_Task_X(const RescueObject *object)
{
    return object->cx * 640.0f / task_scene.image_width;
}

static float App_Task_Bottom(const RescueObject *object)
{
    return (object->cy - object->height / 2U + object->height) * 480.0f
           / task_scene.image_height;
}

static void App_Task_Stop(void)
{
    App_Position_Angle_Reset();
    App_Speed_Set(0.0f, 0.0f);
    task_state = TASK_IDLE;
}

// 大角度搜索/分离动作后更换编号，旧图像不能参与新一次选目标。
static void App_Task_BeginDetect(App_Task_State state)
{
    App_Speed_Set(0.0f, 0.0f);
    task_time = HAL_GetTick();
    view_id++;
    if(view_id == 0){
        view_id = 1;
    }
    view_sent = 0;
    task_state = state;
}

// 连续对准/接近不切换 view_id，靠新帧逐步关联；禁止移动摄像头。
// 接收年龄不能反映相机推理积压，相机仍须及时采集/推理/发送。
static uint8_t App_Task_ReadScene(void)
{
    return view_sent != 0 && Rescue_Usart2_GetScene(&task_scene) != 0
           && task_scene.view_id == view_id
           && task_scene.image_width != 0 && task_scene.image_height != 0
           && (task_scene.flags & RESCUE_SCENE_IMAGE_VALID) != 0
           && (uint32_t)(HAL_GetTick() - task_scene.received_ms) < APP_TASK_TRACK_MAX_AGE_MS
           && App_Task_Abs(servo_2.Angle - SERVO2_LIFT) < 0.5f;
}

// @返回：-1 不宜接近；0 与普通/核心物资拥挤；1 图像上单独且通道未见遮挡。
// @说明：初版二维框判断不等于真实空间避障，遮挡物仍可能未被检测。
// 未知物、伤员、危险物、机器人和安全区靠近目标时排除该目标。
static int App_Task_CheckTarget(uint8_t index, uint8_t *neighbours)
{
    const RescueObject *object = &task_scene.objects[index];
    float x = App_Task_X(object);
    float bottom = App_Task_Bottom(object);
    float path_left = (x < APP_TASK_AIM_X ? x : APP_TASK_AIM_X) - APP_TASK_PATH_HALF_WIDTH;
    float path_right = (x > APP_TASK_AIM_X ? x : APP_TASK_AIM_X) + APP_TASK_PATH_HALF_WIDTH;
    uint8_t i;

    *neighbours = 0;
    for(i = 0; i < task_scene.object_count; i++){
        const RescueObject *other = &task_scene.objects[i];
        float gap_x;
        float gap_y;
        float half_width;
        float other_x;

        if(i == index || other->type == RESCUE_OBJECT_START_ZONE){
            continue;
        }
        other_x = App_Task_X(other);
        half_width = other->width * 320.0f / task_scene.image_width;
        gap_x = App_Task_Abs(x - other_x)
                - (object->width + other->width) * 320.0f / task_scene.image_width;
        gap_y = (App_Task_Abs((float)object->cy - other->cy)
                - (object->height + other->height) * 0.5f)
                * 480.0f / task_scene.image_height;
        if(gap_x < APP_TASK_ISOLATE_GAP_X && gap_y < APP_TASK_ISOLATE_GAP_Y){
            if(other->type != RESCUE_OBJECT_SUPPLY && other->type != RESCUE_OBJECT_CORE_SUPPLY){
                return -1;
            }
            (*neighbours)++;
        }
        // 目标前方的通道有其它框时不绕障，也不强行穿过物块堆。
        if(App_Task_Bottom(other) > bottom + APP_TASK_PATH_DEPTH_MARGIN
           && other_x + half_width >= path_left && other_x - half_width <= path_right){
            return -1;
        }
    }
    return *neighbours == 0 ? 1 : 0;
}

// 优先单独绿色；都拥挤时优先邻居少的外围绿色。
// 同级先看底边远近（固定俯角下的参考），再看转向幅度。
// 非绿色永不成为目标；选定后不再每帧排名，避免在多个绿色间跳动。
static uint8_t App_Task_SelectTarget(void)
{
    int best = -1;
    uint8_t best_neighbours = 255;
    uint8_t i;

    for(i = 0; i < task_scene.object_count; i++){
        const RescueObject *object = &task_scene.objects[i];
        uint8_t neighbours;
        float bottom = App_Task_Bottom(object);

        if(object->type != RESCUE_OBJECT_SUPPLY || object->confidence < APP_TASK_CONFIDENCE_MIN
           || App_Task_CheckTarget(i, &neighbours) < 0){
            continue;
        }
        if(best < 0 || neighbours < best_neighbours
           || (neighbours == best_neighbours
               && (bottom > App_Task_Bottom(&task_scene.objects[best])
                   || (bottom == App_Task_Bottom(&task_scene.objects[best])
                       && App_Task_Abs(App_Task_X(object) - APP_TASK_AIM_X)
                          < App_Task_Abs(App_Task_X(&task_scene.objects[best]) - APP_TASK_AIM_X))))){
            best = i;
            best_neighbours = neighbours;
        }
    }
    if(best < 0){
        return 0;
    }
    target = task_scene.objects[best];
    target_image_w = task_scene.image_width;
    target_image_h = task_scene.image_height;
    last_frame_id = task_scene.frame_id;
    last_capture_ms = task_scene.capture_ms;
    target_mixed = best_neighbours != 0;
    correcting = 1;
    task_time = HAL_GetTick();
    task_state = TASK_ALIGN;
    return 1;
}

// 只在新图像上匹配：同类型、中心门限、尺寸变化不超过两倍。
// 最近和次近候选太接近时拒绝，列表下标不是跨帧 ID。
static int App_Task_MatchTarget(void)
{
    float best_score = APP_TASK_TRACK_GATE * APP_TASK_TRACK_GATE;
    float second_score = 1.0e9f;
    int best = -1;
    uint8_t i;

    if(task_scene.image_width != target_image_w || task_scene.image_height != target_image_h){
        return -1;
    }
    for(i = 0; i < task_scene.object_count; i++){
        const RescueObject *object = &task_scene.objects[i];
        float dx = ((float)object->cx - target.cx) * 640.0f / task_scene.image_width;
        float dy = ((float)object->cy - target.cy) * 480.0f / task_scene.image_height;
        float score = dx * dx + dy * dy;

        if(object->type != RESCUE_OBJECT_SUPPLY || object->confidence < APP_TASK_CONFIDENCE_MIN
           || object->width * 2U < target.width || object->width > target.width * 2U
           || object->height * 2U < target.height || object->height > target.height * 2U
           || score > APP_TASK_TRACK_GATE * APP_TASK_TRACK_GATE){
            continue;
        }
        if(best < 0 || score < best_score){
            second_score = best < 0 ? second_score : best_score;
            best_score = score;
            best = i;
        }else if(score < second_score){
            second_score = score;
        }
    }
    if(second_score - best_score < APP_TASK_TRACK_AMBIGUITY * APP_TASK_TRACK_AMBIGUITY){
        return -1;
    }
    return best;
}

// 固定俯角地面近似：框底边 y = horizon + k / (前框距离 + 相机后退量)。
// 两点实测求 horizon、k；只在标定区间内估算，不用框宽猜厘米。
// 绿色框须完整且底边接地；裁切、遮挡、俯角改变均影响结果。
static uint8_t App_Task_GetDistance(float *distance)
{
    float near_y = APP_TASK_RANGE_NEAR_Y;
    float far_y = APP_TASK_RANGE_FAR_Y;
    float near_cm = APP_TASK_RANGE_NEAR_CM + APP_TASK_CAMERA_FRONT_CM;
    float far_cm = APP_TASK_RANGE_FAR_CM + APP_TASK_CAMERA_FRONT_CM;
    float bottom = App_Task_Bottom(&target);
    float k;
    float horizon;

    if(near_y <= far_y || far_y <= 0.0f || near_y >= 480.0f
       || near_cm <= 0.0f || far_cm <= near_cm || bottom < far_y || bottom > near_y
       || target.cy <= target.height / 2U || bottom >= 480.0f
       || target.cx <= target.width / 2U
       || (uint32_t)target.cx - target.width / 2U + target.width >= task_scene.image_width){
        return 0;
    }
    k = (near_y - far_y) / (1.0f / near_cm - 1.0f / far_cm);
    horizon = near_y - k / near_cm;
    *distance = k / (bottom - horizon) - APP_TASK_CAMERA_FRONT_CM;
    return *distance > 0.0f;
}

// 新帧决定速度；同一帧只运行速度闭环，不反复更新锁定/死区。
static void App_Task_Follow(void)
{
    int index;
    int condition;
    uint8_t neighbours;
    float error;
    float turn = 0.0f;
    float speed = 0.0f;
    float distance;
    float stop_cm = target_mixed ? APP_TASK_SCATTER_STOP_CM : APP_TASK_APPROACH_STOP_CM;
    uint32_t timeout = task_state == TASK_ALIGN ? APP_TASK_ALIGN_TIMEOUT_MS : APP_TASK_APPROACH_TIMEOUT_MS;

    if((uint32_t)(HAL_GetTick() - task_time) >= timeout || App_Task_ReadScene() == 0
       || (task_scene.flags & RESCUE_SCENE_TRUNCATED) != 0){
        App_Task_Stop(); // 不沿过期/不完整图像继续开车
        return;
    }
    if(task_scene.frame_id == last_frame_id && task_scene.capture_ms == last_capture_ms){
        App_Speed_Pro();
        return;
    }
    index = App_Task_MatchTarget();
    if(index < 0){
        App_Task_Stop(); // 丢失/无法区分时停车等待重新启动
        return;
    }
    condition = App_Task_CheckTarget((uint8_t)index, &neighbours);
    if(condition < 0 || (target_mixed == 0 && condition == 0)){
        App_Task_Stop(); // 原本单独的目标变拥挤，不意外带动其它物块
        return;
    }
    target = task_scene.objects[index];
    last_frame_id = task_scene.frame_id;
    last_capture_ms = task_scene.capture_ms;
    error = App_Task_X(&target) - APP_TASK_AIM_X; // 正数：目标在右

    if(task_state == TASK_APPROACH && App_Task_Abs(error) > APP_TASK_AIM_REALIGN){
        task_state = TASK_ALIGN;
        task_time = HAL_GetTick();
        correcting = 1;
    }
    if(task_state == TASK_ALIGN){
        if(App_Task_Abs(error) <= APP_TASK_AIM_DEAD_ENTER){
            task_state = TASK_APPROACH;
            task_time = HAL_GetTick();
            correcting = 0;
            App_Speed_Set(0.0f, 0.0f); // 先对准，下一幅新图再前进
            return;
        }
    }else {
        if(App_Task_GetDistance(&distance) == 0){
            App_Task_Stop(); // 未标定/超出区间，不把像素当厘米
            return;
        }
        if(distance <= stop_cm){
            App_Speed_Set(0.0f, 0.0f);
            task_state = target_mixed ? TASK_SCATTER : TASK_EXECUTE;
            task_time = HAL_GetTick();
            return;
        }
        speed = distance - stop_cm <= APP_TASK_SLOW_MARGIN_CM
                ? APP_TASK_APPROACH_SLOW : APP_TASK_APPROACH_SPEED;
        if(App_Task_Abs(error) <= APP_TASK_AIM_DEAD_ENTER){
            correcting = 0;
        }else if(App_Task_Abs(error) >= APP_TASK_AIM_DEAD_EXIT){
            correcting = 1;
        }
    }
    if(correcting != 0){
        turn = APP_TASK_STEER_KP * error;
        if(turn > APP_TASK_STEER_MAX){ turn = APP_TASK_STEER_MAX; }
        if(turn < -APP_TASK_STEER_MAX){ turn = -APP_TASK_STEER_MAX; }
        turn *= APP_TASK_STEER_SIGN;
    }
    App_Speed_Set(speed + turn, speed - turn);
    App_Speed_Pro(); // 视觉直接给轮速，不同时运行原地转向角度环
}

// 分离接口占位：当前不撞、不推、不控制前框，默认停车。
// 后续可实现低速短推 -> 后退观察；禁止抓取/抬起、推动危险物到边界/安全区。
// RUNNING 时接口自行执行闭环；DONE 后上层重新采集新视角。
App_Task_ScatterResult App_Task_Scatter_Proc(const RescueObject *object)
{
    (void)object;
    return APP_TASK_SCATTER_WAIT;
}

void App_Task_Start(void)
{
    uint32_t now_ms = HAL_GetTick();

    if(task_state != TASK_IDLE && task_state != TASK_EXECUTE){
        return;
    }
    if(round_started != 0 && (uint32_t)(now_ms - round_time) >= APP_TASK_ROUND_TIME_MS){
        return; // 本轮到时后不能通过按键重新计时，下一轮需复位
    }
    if(round_started == 0){
        round_started = 1;
        round_time = now_ms;
    }
    App_Position_Angle_Reset();
    App_Servo_SetAngle(&servo_2, SERVO2_LIFT); // 固定搜索俯角，标定须用同一角度
    App_Speed_Set(APP_TASK_FORWARD_SPEED, APP_TASK_FORWARD_SPEED);
    task_time = now_ms;
    task_state = TASK_FORWARD;
}

// 主循环先调用 Rescue_Usart2_Process()，再调用本函数；没有阻塞延时。
void App_Task_Proc(void)
{
    uint32_t now_ms = HAL_GetTick();

    if(round_started != 0 && (uint32_t)(now_ms - round_time) >= APP_TASK_ROUND_TIME_MS){
        if(task_state != TASK_IDLE){ App_Task_Stop(); }
        return;
    }
    switch(task_state){
    case TASK_FORWARD:
        if((uint32_t)(now_ms - task_time) >= APP_TASK_FORWARD_TIME_MS){
            App_Speed_Set(0.0f, 0.0f);
            Set_Angle_SP(APP_TASK_RIGHT_ANGLE);
            task_time = now_ms;
            task_state = TASK_TURN_RIGHT;
        }else {
            App_Speed_Pro();
        }
        break;

    case TASK_TURN_RIGHT:
    case TASK_TURN_LEFT:
        App_Angle_Pro();
        if(motor_angle_done == 1){
            App_Position_Angle_Reset();
            App_Task_BeginDetect(task_state == TASK_TURN_RIGHT ? TASK_DETECT_RIGHT : TASK_DETECT_LEFT);
        }else if((uint32_t)(now_ms - task_time) >= APP_TASK_TURN_TIMEOUT_MS){
            App_Task_Stop();
        }
        break;

    case TASK_DETECT_RIGHT:
    case TASK_DETECT_LEFT:
    case TASK_RESELECT:
        if((uint32_t)(now_ms - task_time) >= APP_TASK_DETECT_TIMEOUT_MS){
            App_Task_Stop();
            break;
        }
        if((uint32_t)(now_ms - task_time) < APP_TASK_SETTLE_TIME_MS){
            break;
        }
        if(view_sent == 0 || (uint32_t)(now_ms - view_time) >= APP_TASK_VIEW_RETRY_MS){
            view_sent = Rescue_Usart2_SendViewContext(view_id, 0);
            view_time = now_ms;
        }
        if(App_Task_ReadScene() == 0 || (task_scene.flags & RESCUE_SCENE_TRUNCATED) != 0){
            break; // 完整有效的新视角才选目标
        }
        if(App_Task_SelectTarget() != 0){
            break;
        }
        if(task_state == TASK_DETECT_RIGHT){
            Set_Angle_SP(APP_TASK_LEFT_ANGLE); // 右侧无合适绿色，再到左侧找
            task_time = now_ms;
            task_state = TASK_TURN_LEFT;
        }else {
            App_Task_Stop();
        }
        break;

    case TASK_ALIGN:
    case TASK_APPROACH:
        App_Task_Follow();
        break;

    case TASK_SCATTER: {
        App_Task_ScatterResult result;
        if((uint32_t)(now_ms - task_time) >= APP_TASK_SCATTER_TIMEOUT_MS){
            App_Task_Stop();
            break;
        }
        result = App_Task_Scatter_Proc(&target);
        if(result == APP_TASK_SCATTER_DONE){
            App_Position_Angle_Reset();
            App_Task_BeginDetect(TASK_RESELECT); // 动作结束不代表分离成功，必须重检
        }else if(result == APP_TASK_SCATTER_WAIT){
            App_Speed_Set(0.0f, 0.0f);
        }else if(result != APP_TASK_SCATTER_RUNNING){
            App_Task_Stop();
        }
        break;
    }

    case TASK_EXECUTE:
        // 单独绿色已接近：前框动作 -> 首次单件送本队物资区 -> 确认送达，暂留空。
        // 本版不推送/解锁其它目标，不把到达观察距离当成首运成功。
        App_Speed_Set(0.0f, 0.0f);
        break;

    case TASK_IDLE:
    default:
        break;
    }
}
