//
// Created by gaoxu on 2026/10/20.
//

#include "app_task_vision.h" // 引入视觉选择、跟随和复检接口。
#include "app_line_pid.h" // 引入视觉纠偏PID及死区接口。
#include "app_servo.h" // 引入舵机结构体及角度设置接口。
#include "app_usart.h" // 引入串口1日志接口，仅在跟随失败时打印原因。
#include "rescue_usart2.h" // 引入相机场景读取及视角发送接口。
#include "app_task_zone.h" // 用整体安全区和分界线生成本地分区框。
#include <string.h> // 只用于诊断原因去重及日志分组。

static RescueScene task_scene;       // 相机原始像素快照，不是厘米坐标
static RescueObject derived_zones[APP_TASK_DERIVED_ZONE_MAX]; // 独立保存分区，不覆盖原始危险物和障碍条目。
static uint8_t derived_zone_count;
static RescueObject target;          // 保存目标类型及检测框，不保存列表下标
static uint16_t target_image_w; // 锁定目标时的实际图像宽度，后续分辨率变化则拒绝跟随。
static uint16_t target_image_h; // 锁定目标时的实际图像高度，保持跨帧坐标比较基准一致。
static uint16_t view_id; // 当前视角编号，换阶段时递增以排除旧视角图像。
static uint16_t last_frame_id; // 最近已处理图像的帧编号。
static uint32_t last_capture_ms; // 最近已处理图像的相机采集时间，与帧号一起用于去重。
static uint32_t view_begin_time; // 当前视角开始时刻，供画面稳定等待使用。
static uint32_t view_settle_ms; // 本次新视角的画面稳定等待；复检和后退后搜索为0，其它搜索保留配置值。
static uint32_t view_time; // 上次视角上下文发送尝试时刻，限制重发频率。
static uint32_t follow_time; // 首次对准或持续接近阶段起点，用于该阶段超时保护。
static uint8_t view_sent; // 当前视角上下文是否已经成功发送。
static uint8_t frame_seen; // 本阶段是否接受过图像，静态初值为0。
static uint8_t verify_frames; // 当前复检窗口已检查的有效新图数量，空区域也占一张，重复帧不累计。
static uint8_t verify_hits; // 当前三张复检新图中，区域内恰好一个可信绿色的合格张数。
static uint8_t target_mixed; // 当前锁定是否用于撞散，锁定后不因列表变化而改变用途。
static uint8_t target_zone; // 当前锁定是否为本队物资分区。
static uint8_t target_zone_derived; // 当前分区是否由本帧整体区+分界线计算。
static uint8_t follow_forward;        // 首次对准结束后置1，接近时不再置0
static int8_t capture_y_direction; // 单独绿色纵向实际控制方向：前进1、倒车-1、停车0。
static int8_t capture_y_request; // 等待稳定确认的单独绿色纵向方向。
static uint8_t capture_y_frames; // 同一请求方向连续出现的新图计数。
static uint32_t capture_y_wait_time; // 进入停车/换向等待的起始时刻。
static uint8_t team_supply_type; // 本队物资放置分区的视觉协议类型，按队伍颜色初始化。
static float camera_angle = SERVO2_LIFT; // 本阶段期望的摄像头俯角，默认正常观察角，并非实测反馈。
static uint8_t search_empty_frames;   // 本侧连续无候选的新帧数量
static uint8_t lost_frames;           // 只统计未匹配原目标的有效新帧
static uint32_t lost_time;            // 第一次短暂丢失时间，不延长总接近超时
static uint8_t arrive_frames;         // 停车后连续到位确认，不重复计算同帧
static uint32_t arrive_time; // 当前一轮到位停车确认的计时起点。
static float arrive_x; // 当前一轮到位确认的目标中心x位置基准。
static float arrive_y; // 当前一轮到位确认的目标中心y位置基准。

// 以下字段仅保存诊断现场，不参与动作通过/失败的判断。
static const char *scene_reason = "not_checked";
static const char *match_reason = "not_checked";
static const char *cargo_reason = "not_checked";
static const char *zone_reason = "not_checked";
static const char *scatter_reason = "not_checked";
static const char *verify_wait = "new_view";
static const char *select_wait = "new_view";
static const char *verify_reported;
static const char *select_reported;
static const char *zone_reported;
static uint8_t scene_snapshot; // 1表示本次读图取得了快照；0表示task_scene仅是上次保存值。
static uint8_t view_send_failed;
static uint32_t checked_age; // 保存时效检查实际使用的合计年龄。
static uint8_t match_reject[6]; // 依次为类别、置信度、ID、宽、高、位移拒绝数量。
static float match_best_score, match_second_score;
static RescueObject diagnostic_target;
static RescueObject diagnostic_zone_object;
static uint8_t diagnostic_zone_valid;
static int cargo_bad_index = -1;
static int zone_bad_index = -1;
static int zone_path_index = -1;
static int scatter_bad_index = -1;
static uint8_t cargo_count;
static uint16_t verify_new_total, verify_duplicates, verify_invalid;

// 同一种持续等待只打印首次或原因变化；正常重复帧不周期打印。
static void App_Task_LogWait(const char *stage, const char *reason, const char **reported)
{
    if(APP_TASK_DEBUG_ENABLE && (*reported == 0 || strcmp(*reported, reason) != 0)){
        App_USART1_Printf("[%s] wait=%s\r\n", stage, reason);
    }
    *reported = reason;
}

// 原始条目下标不变；本地分区接在原始条目后，仅供分区选择、跟随和通道检查。
static const RescueObject *App_Task_ObjectAt(uint8_t index)
{
    return index < task_scene.object_count ? &task_scene.objects[index]
                                         : &derived_zones[index - task_scene.object_count];
}

// @简介：返回浮点绝对值，用于像素门限和舵机角度比较。
static float App_Task_Abs(float value) // 函数入口：返回浮点绝对值，用于像素门限和舵机角度比较。
{ // 开始App_Task_Abs函数体。
    return value < 0.0f ? -value : value; // 负值取反、非负值保持，返回浮点绝对值。
} // 结束App_Task_Abs函数体。

// @简介：把目标中心 x 换算到 640 宽度；原始检测坐标不变。
static float App_Task_X(const RescueObject *object) // 函数入口：把目标中心 x 换算到 640 宽度；原始检测坐标不变。
{ // 开始App_Task_X函数体。
    return object->cx * APP_TASK_IMAGE_W / task_scene.image_width; // 将原始中心x按实际图像宽度缩放到参考图像坐标。
} // 结束App_Task_X函数体。

// @简介：把目标中心 y 换算到 480 高度；用于停车，不使用检测框底边。
static float App_Task_Y(const RescueObject *object) // 函数入口：把目标中心 y 换算到 480 高度；用于停车，不使用检测框底边。
{ // 开始App_Task_Y函数体。
    return object->cy * APP_TASK_IMAGE_H / task_scene.image_height; // 将原始中心y按实际图像高度缩放到参考图像坐标。
} // 结束App_Task_Y函数体。

// @简介：把框底边换算到 480 高度，只用于候选排序和通道阻挡判断。
static float App_Task_Bottom(const RescueObject *object) // 函数入口：把框底边换算到 480 高度，只用于选择、通道检查和边缘裁切判断。
{ // 开始App_Task_Bottom函数体。
    return (object->cy - object->height / 2U + object->height) * APP_TASK_IMAGE_H // 以中心减半高再加整高计算框底边，并乘参考图像高度。
           / task_scene.image_height; // 除以实际图像高度，完成框底边y归一化。
} // 结束App_Task_Bottom函数体。

// @简介：画面稳定后发送/重发当前编号；相机必须在采集图像时锁存编号。
static void App_Task_UpdateView(void) // 函数入口：画面稳定后发送/重发当前编号；相机必须在采集图像时锁存编号。
{ // 开始App_Task_UpdateView函数体。
    uint32_t now_ms = HAL_GetTick(); // 读取当前毫秒计时，作为本次处理的时间基准。
    if((uint32_t)(now_ms - view_begin_time) < view_settle_ms){ return; } // 按本次视角的等待时间判断；传0的阶段立即发送上下文。
    if(view_sent == 0 || (uint32_t)(now_ms - view_time) >= APP_TASK_VIEW_RETRY_MS){ // 首次未发送成功或已经到达重发间隔时，再尝试发送。
        view_sent = Rescue_Usart2_SendViewContext(view_id, 0);
        view_send_failed = view_sent == 0; // 保存发送失败，交给调用方在停车后或等待事件中打印。 // 发送当前视角上下文，并记录串口发送接口是否接受成功。
        view_time = now_ms; // 记录本次发送尝试的时刻，供下一次重发间隔判断。
    } // 结束条件分支。
} // 结束App_Task_UpdateView函数体。

// @简介：当前图像是否为本阶段未处理的新帧；同一帧不能重复累计复检次数。
static uint8_t App_Task_NewFrame(void) // 函数入口：当前图像是否为本阶段未处理的新帧；同一帧不能重复累计复检次数。
{ // 开始App_Task_NewFrame函数体。
    if(frame_seen != 0 && task_scene.frame_id == last_frame_id // 本阶段已经见过图像且帧编号相同时，继续比较采集时间。
       && task_scene.capture_ms == last_capture_ms){ return 0; } // 帧编号及采集时间都相同则为重复图像，返回0不重复累计。
    frame_seen = 1; // 记录本阶段已经接受过图像。
    last_frame_id = task_scene.frame_id; // 保存最近处理帧的编号，用于识别重复图像。
    last_capture_ms = task_scene.capture_ms; // 保存最近处理帧的相机采集时间，与帧号共同去重。
    return 1; // 当前检查通过，返回1表示本次条件成立。
} // 结束App_Task_NewFrame函数体。

// @简介：检查本地接收年龄；扩展帧再检查采集至发送、串口及本地等待的合计估计年龄。
// @说明：扩展帧额外检查相机耗时+估算串口耗时；旧帧无相机耗时，不能声称图像真实年龄合格。
static uint8_t App_Task_ImageFresh(void)
{
    uint32_t age = (uint32_t)(HAL_GetTick() - task_scene.received_ms);
    uint32_t wire_size = RESCUE_FRAME_OVERHEAD + RESCUE_EXT_HEADER_SIZE
                         + task_scene.object_count * RESCUE_EXT_OBJECT_SIZE;
    checked_age = age;
    if(age >= APP_TASK_TRACK_MAX_AGE_MS){ scene_reason = "receive_age"; return 0; }
    if(task_scene.has_source_age == 0){ return 1; }
    age += task_scene.source_age_ms
           + (wire_size * 10000U + APP_TASK_SERIAL_BAUD - 1U) / APP_TASK_SERIAL_BAUD;
    checked_age = age;
    if(age >= APP_TASK_IMAGE_MAX_AGE_MS){ scene_reason = "source_age"; }
    return age < APP_TASK_IMAGE_MAX_AGE_MS;
}

// @简介：读取完整、当前视角、新鲜场景；无效、过期或摄像头设定角改变返回0。
static uint8_t App_Task_ReadScene(void)
{
    App_Task_UpdateView(); // 保留原发送/重发时序和短路检查顺序。
    derived_zone_count = 0;
    scene_snapshot = 0;
    if(view_sent == 0){ scene_reason = view_send_failed ? "view_send_failed" : "view_not_sent"; return 0; }
    if(Rescue_Usart2_GetScene(&task_scene) == 0){ scene_reason = "scene_unavailable"; return 0; }
    scene_snapshot = 1;
    if(task_scene.view_id != view_id){ scene_reason = "view_mismatch"; return 0; }
    if((task_scene.flags & RESCUE_SCENE_IMAGE_VALID) == 0){ scene_reason = "image_invalid"; return 0; }
    if((task_scene.flags & RESCUE_SCENE_TRUNCATED) != 0){ scene_reason = "list_truncated"; return 0; }
    if(App_Task_ImageFresh() == 0){ return 0; }
    if(!(App_Task_Abs(servo_2.Angle - camera_angle) < APP_TASK_SERVO_TOL)){
        scene_reason = "camera_angle"; return 0; // 保留原浮点比较语义，包含非数值不通过的情况。
    }
    scene_reason = "ok";
    derived_zone_count = App_Task_Zone_Derive(&task_scene, derived_zones, APP_TASK_DERIVED_ZONE_MAX);
    return 1;
}

// @简介：撞散的可见场景检查；最多3件普通/核心物资，低置信或其它实体/区域均拒绝。
// @说明：旋转期间也检查整个可见画面；看不到的物块和场地边界仍需后续感知补充。
static uint8_t App_Task_ScatterSceneClear(void)
{
    uint8_t count = 0;
    scatter_bad_index = -1;
    scatter_reason = "ok";
    for(uint8_t i = 0; i < task_scene.object_count; ++i){
        const RescueObject *object = &task_scene.objects[i];
        if(object->type == RESCUE_OBJECT_START_ZONE){ continue; }
        if(object->type != RESCUE_OBJECT_SUPPLY && object->type != RESCUE_OBJECT_CORE_SUPPLY){
            scatter_reason = "forbidden_type"; scatter_bad_index = i; return 0;
        }
        if(object->confidence < APP_TASK_CONFIDENCE_MIN){
            scatter_reason = "low_confidence"; scatter_bad_index = i; return 0;
        }
        if(++count > APP_TASK_SCATTER_MAX_COUNT){
            scatter_reason = "too_many_supplies"; scatter_bad_index = i; return 0;
        }
    }
    return 1;
}

// @简介：锁存目标并复位视觉PID，从下一幅新图开始首次对准；zone=1表示目的分区。
static void App_Task_Lock(uint8_t index, uint8_t zone, uint8_t mixed) // 函数入口：锁存目标并复位视觉PID，从下一幅新图开始首次对准；zone=1表示目的分区。
{ // 开始App_Task_Lock函数体。
    target = *App_Task_ObjectAt(index); // 支持相机分区和本地推导分区，不保存列表下标。
    target_image_w = task_scene.image_width; // 保存锁定时图像宽度，跟随期间不接受分辨率突变。
    target_image_h = task_scene.image_height; // 保存锁定时图像高度，保证后续坐标关联基准一致。
    last_frame_id = task_scene.frame_id; // 保存最近处理帧的编号，用于识别重复图像。
    last_capture_ms = task_scene.capture_ms; // 保存最近处理帧的相机采集时间，与帧号共同去重。
    frame_seen = 1; // 记录本阶段已经接受过图像。
    target_zone = zone; // 记录本次锁定是否用于接近本队物资区。
    target_zone_derived = zone && index >= task_scene.object_count;
    target_mixed = mixed; // 记录本次锁定是单件救援还是物资堆撞散用途。
    follow_forward = 0; // 新锁定目标先进入首次对准，尚未开始持续前进。
    capture_y_direction = 0; // 新目标从停车状态启动纵向控制。
    capture_y_request = 0; // 清除上一个目标遗留的待恢复方向。
    capture_y_frames = 0; // 清空待恢复方向的连续新图计数。
    lost_frames = 0; // 清零原目标连续未匹配的新帧计数。
    arrive_frames = 0; // 清零连续到位确认帧数，禁止继承旧确认结果。
    follow_time = HAL_GetTick(); // 记录当前首次对准或持续接近阶段的毫秒起点。
    App_LinePID_Reset(); // 清空视觉PID历史误差及死区状态，避免旧输出干扰新阶段。
} // 结束App_Task_Lock函数体。

// @简介：在新图像中关联已锁定目标（普通/核心物资或本队物资区），失败/歧义返回-1。
// @说明：比较本帧同类候选与上一次成功匹配目标的(x,y)，选择平方距离最小者；
//        不按列表下标、离屏幕中心远近或单独比较x重新选目标，y方向同样参与判断。
//        默认位移最多70px、框宽高变化不超过两倍；有非零相机ID时还必须同ID。
//        最近/次近候选得分难以区分时返回失败，交给连续丢失确认，不能强行换锁。
static int App_Task_MatchTarget(void) // 函数入口：在新图像中关联已锁定目标（普通/核心物资或本队物资区），失败/歧义返回-1。
{ // 开始App_Task_MatchTarget函数体。
    memset(match_reject, 0, sizeof(match_reject)); // 每幅新图独立记录各类候选拒绝。
    match_reason = "no_candidate";
    float best_score = APP_TASK_TRACK_GATE * APP_TASK_TRACK_GATE; // 最佳关联距离初值设为允许跨帧位移的平方。
    float second_score = 1.0e9f; // 次近候选距离初值设为很大，后续用于关联歧义检查。
    int best = -1; // 最佳候选下标初始化为未找到。
    uint8_t i; // 声明遍历场景目标数组的下标。

    if(task_scene.image_width != target_image_w || task_scene.image_height != target_image_h){ // 图像分辨率与原锁定不同则无法可靠比较坐标。
        match_reason = "image_size_changed";
        return -1; // 返回当前目标不可使用或未能可靠匹配，由调用者按本步骤处理。
    } // 结束条件分支。
    for(i = 0; i < task_scene.object_count + (target_zone ? derived_zone_count : 0U); i++){
        const RescueObject *object = App_Task_ObjectAt(i); // 分区跟随同时关联本帧重新推导的框。
        float dx = ((float)object->cx - target.cx) * APP_TASK_IMAGE_W / task_scene.image_width; // 计算当前候选相对于上次成功匹配目标的归一化水平位移。
        float dy = ((float)object->cy - target.cy) * APP_TASK_IMAGE_H / task_scene.image_height; // 计算当前候选相对于上次成功匹配目标的归一化垂直位移。
        float score = dx * dx + dy * dy; // 与上次目标的二维平方距离；不必开平方即可比较远近

        if(object->type != target.type){ ++match_reject[0]; continue; }
        if(object->confidence < APP_TASK_CONFIDENCE_MIN){ ++match_reject[1]; continue; }
        if(target.track_id != 0 && object->track_id != target.track_id){ ++match_reject[2]; continue; }
        if(object->width * APP_TASK_TRACK_SIZE_RATIO < target.width
           || object->width > target.width * APP_TASK_TRACK_SIZE_RATIO){ ++match_reject[3]; continue; }
        if(object->height * APP_TASK_TRACK_SIZE_RATIO < target.height
           || object->height > target.height * APP_TASK_TRACK_SIZE_RATIO){ ++match_reject[4]; continue; }
        if(score > APP_TASK_TRACK_GATE * APP_TASK_TRACK_GATE){ ++match_reject[5]; continue; }
        if(best < 0 || score < best_score){ // 第一个合格候选，或比当前最佳更接近原位置的候选，更新最佳排名。
            second_score = best < 0 ? second_score : best_score; // 已经存在最佳候选时，将其距离保存为新的次近距离。
            best_score = score; // 保存新最佳候选的二维平方距离。
            best = i; // 保存当前最佳候选的本帧下标，不能将下标当作跨帧身份。
        }else if(score < second_score){ // 当前候选不是最近，但优于已有次近候选时更新次近距离。
            second_score = score; // 保存次近距离，用于最终判断最佳匹配是否足够明确。
        } // 结束条件分支。
    } // 结束目标遍历循环。
    match_best_score = best_score; match_second_score = second_score; // 只保存排序得分供诊断。
    if(second_score - best_score < APP_TASK_TRACK_AMBIGUITY * APP_TASK_TRACK_AMBIGUITY){ // 最近与次近候选距离差太小时放弃关联，避免强行换锁。
        match_reason = "ambiguous";
        return -1; // 返回当前目标不可使用或未能可靠匹配，由调用者按本步骤处理。
    } // 结束条件分支。
    if(best >= 0){ match_reason = "ok"; }
    else if(match_reject[1] || match_reject[2] || match_reject[3] || match_reject[4] || match_reject[5]){ match_reason = "candidates_rejected"; }
    else { match_reason = "no_same_type"; }
    return best; // 返回最近且无歧义的候选下标，未找到时保持-1。
} // 结束App_Task_MatchTarget函数体。

// @简介：视觉PID输出左右轮速；speed=0用于原地对准，正负值分别表示前进和倒车。
// 平移时按速度绝对值限幅，保证两轮保持同一行驶方向；目标偏右时左轮加速、右轮减速。
static void App_Task_VisualDrive(float x, float speed, uint8_t near) // 函数入口：视觉PID输出左右轮速；speed=0原地对准，正值前进、负值倒车。
{ // 开始App_Task_VisualDrive函数体。
    float turn = near ? App_LinePID_ComputeNear(x) : App_LinePID_Compute(x); // 按是否处于近距离阶段选择近处或常规视觉PID，得到单侧差速量。
    float limit = App_Task_Abs(speed) * LINE_PID_FORWARD_RATIO; // 按基础速度绝对值计算差速限幅，避免纠偏使任一侧反向。
    if(speed != 0.0f){ // 前进和倒车均限制差速；speed为0时保留原地转向输出。
        if(turn > limit){ turn = limit; } // 限制正差速，避免转向修正抵消基础平移方向。
        if(turn < -limit){ turn = -limit; } // 限制负差速，避免转向修正抵消基础平移方向。
    } // 结束条件分支。
    App_Task_Move_Speed(speed + turn, speed - turn); // 仅运行速度内环
} // 结束App_Task_VisualDrive函数体。

// @简介：按归一化中心坐标统计前框内实体；1=单绿色通过，0=未看到，-1=有冲突。
// @说明：与App_Cargo_Test打印的x/y相同，230≤x≤490且400≤y≤450才计数；不检查框边缘。
// 中心在区域外的实体不计数；区域内未知物、危险物、伤员等也计入，不能只数绿色。
static int App_Task_CheckCargo(void) // 位置合格后继续检查单件、绿色类别和置信度。
{ // 开始App_Task_CheckCargo函数体。
    uint8_t i; // 声明遍历场景目标数组的下标。
    cargo_bad_index = -1; cargo_count = 0; cargo_reason = "cargo_empty";
    uint8_t count = 0; // 从零开始累计本次检查涉及的实体数量。
    for(i = 0; i < task_scene.object_count; i++){ // 逐项检查相机上报场景中的全部目标。
        const RescueObject *object = &task_scene.objects[i]; // 取得当前目标的只读指针，避免复制检测信息。
        float x = App_Task_X(object); // 与测试打印一致：cx×640/image_width，物块中心横坐标。
        float y = App_Task_Y(object); // 与测试打印一致：cy×480/image_height，物块中心纵坐标。

        // 区域标签不是待运实体，避免安全区背景框被统计成物块。
        if(object->type >= RESCUE_OBJECT_RED_SAFE_ZONE && object->type <= RESCUE_OBJECT_START_ZONE){ continue; } // 整体安全区及出发区标签不作为前框内实体计数。
        if(object->type >= RESCUE_OBJECT_RED_SUPPLY_ZONE){ continue; } // 物资区、伤员区等分区标签同样不计作载物实体。
        if(x < APP_TASK_CARGO_X_MIN || x > APP_TASK_CARGO_X_MAX // 中心x在230～490之外时忽略；端点包含在内。
           || y < APP_TASK_CARGO_Y_MIN || y > APP_TASK_CARGO_Y_MAX){ continue; } // 中心y在400～450之外时同样忽略。
        count++;
        cargo_count = count; // 中心位于前框区域内，计入本帧实体数量。
        if(count > 1 || object->type != RESCUE_OBJECT_SUPPLY // 区域内多个实体或非绿色时拒绝复检。
           || object->confidence < APP_TASK_CONFIDENCE_MIN){ // 绿色置信度仍须达到原门限。
            cargo_bad_index = i;
            cargo_reason = count > 1 ? "cargo_multiple"
                           : (object->type != RESCUE_OBJECT_SUPPLY ? "cargo_wrong_type" : "cargo_low_confidence");
            return -1; // 返回当前目标不可使用或未能可靠匹配，由调用者按本步骤处理。
        } // 结束中心入区后的数量、类型和置信度检查。
    } // 结束目标遍历循环。
    if(count == 1){ cargo_reason = "ok"; }
    return count == 1 ? 1 : 0; // 恰好一个合格绿色返回1，无实体中心在区域内则返回0。
} // 结束App_Task_CheckCargo函数体。

// @简介：寻找唯一可信本队物资分区；可来自相机或同帧整体安全区+分界线推导。
// @返回：下标；-1未找到；-2多个同类分区/低置信度，不能确定目的地。
static int App_Task_FindZone(void)
{
    int index = -1;
    zone_bad_index = -1;
    for(uint8_t i = 0; i < task_scene.object_count + derived_zone_count; ++i){
        const RescueObject *object = App_Task_ObjectAt(i);
        if(object->type != team_supply_type){ continue; }
        if(index >= 0){ zone_reason = "zone_multiple"; zone_bad_index = i; return -2; }
        if(object->confidence < APP_TASK_CONFIDENCE_MIN){
            zone_reason = "zone_low_confidence"; zone_bad_index = i; return -2;
        }
        index = i;
    }
    zone_reason = index >= 0 ? "ok" : App_Task_Zone_LastReason(team_supply_type);
    return index;
}

// @简介：检查前往分区的图像通道；危险物、障碍、伤员、核心物资、未知物及对方区阻挡则停。
// 普通绿色可能就是前框内物块；不据此推断实际载物数量，也不替代全局定位/避障。
static uint8_t App_Task_ZonePathClear(void) // 函数入口：检查前往分区的图像通道；危险物、障碍、伤员、核心物资、未知物及对方区阻挡则停。
{ // 开始App_Task_ZonePathClear函数体。
    zone_path_index = -1; // 保存实际阻挡目的区通道的那一个框。
    uint8_t i; // 声明遍历场景目标数组的下标。
    uint8_t green_count = 0; // 从零累计前往物资区通道中的绿色实体数量。
    uint8_t own_whole = team_supply_type == RESCUE_OBJECT_RED_SUPPLY_ZONE // 根据已锁存的物资分区类型判断本队颜色。
                        ? RESCUE_OBJECT_RED_SAFE_ZONE : RESCUE_OBJECT_BLUE_SAFE_ZONE; // 选择本队对应的整体安全区标签，用于忽略目的地背景区域。
    float x = App_Task_X(&target); // 取得锁定物资区中心的归一化x坐标。
    float path_left = (x < APP_TASK_AIM_X ? x : APP_TASK_AIM_X) - APP_TASK_PATH_HALF_WIDTH; // 以车头和目标中较小的x向左扩展，形成接近通道左边界。
    float path_right = (x > APP_TASK_AIM_X ? x : APP_TASK_AIM_X) + APP_TASK_PATH_HALF_WIDTH; // 以车头和目标中较大的x向右扩展，形成接近通道右边界。
    for(i = 0; i < task_scene.object_count + derived_zone_count; i++){
        const RescueObject *object = App_Task_ObjectAt(i); // 本地伤员区也参与阻挡检查，分界线不整体忽略。
        float half_w = object->width * (APP_TASK_IMAGE_W * 0.5f) / task_scene.image_width; // 计算当前检测框在参考图像中的半宽。
        if(object->type == team_supply_type || object->type == own_whole // 本队物资分区及本队整体安全区属于目的地背景标签。
           || object->type == RESCUE_OBJECT_START_ZONE){ continue; } // 出发区同样不作为通道实体，跳过这些区域标签。
        // 首次对准仅原地转向：本队右侧分区和紧贴物资区右沿的已关联分界线
        // 不作为平移通道阻挡；进入前进阶段后仍检查，危险物等实体始终检查。
        if(target_zone_derived && follow_forward == 0
           && (object->type == team_supply_type + 1U
               || (object->type == RESCUE_OBJECT_SAFE_ZONE_DIVIDER
                   && object->cx - object->width / 2U == target.cx - target.width / 2U + target.width
                   && object->cy >= target.cy - target.height / 2U
                   && object->cy <= target.cy - target.height / 2U + target.height))){ continue; }
        if(App_Task_X(object) + half_w < path_left || App_Task_X(object) - half_w > path_right // 检测框完全位于水平接近通道之外时不按当前阻挡处理。
           || App_Task_Bottom(object) < App_Task_Y(&target)){ continue; } // 框底边未到目的区中心深度时也跳过。
        if(object->type == RESCUE_OBJECT_SUPPLY && ++green_count == 1){ continue; } // 允许通道内第一件绿色，可能是前框内物块；不能据此确认真实载物数。
        zone_path_index = i;
        return 0; // 当前检查不通过，返回0拒绝继续该动作。
    } // 结束目标遍历循环。
    return 1; // 当前检查通过，返回1表示本次条件成立。
} // 结束App_Task_ZonePathClear函数体。

// @简介：失败/超时时打印最后检查现场；snapshot=0表示快照是旧值，不能当作当前图。
void App_Task_Vision_LogDetails(const char *stage)
{
    if(APP_TASK_DEBUG_ENABLE == 0){ return; }
    App_USART1_Printf("[SCENE] reason=%s snapshot=%u frame=%u age=%lu\r\n", scene_reason,
                     (unsigned int)scene_snapshot, (unsigned int)task_scene.frame_id,
                     (unsigned long)(HAL_GetTick() - task_scene.received_ms));
    App_USART1_Printf("[SCENE] view=%u/%u flags=%u n=%u img=(%u,%u)\r\n",
                     (unsigned int)task_scene.view_id, (unsigned int)view_id,
                     (unsigned int)task_scene.flags, (unsigned int)task_scene.object_count,
                     (unsigned int)task_scene.image_width, (unsigned int)task_scene.image_height);
    if(strcmp(scene_reason, "receive_age") == 0 || strcmp(scene_reason, "source_age") == 0){
        App_USART1_Printf("[SCENE] checked_age=%lu source=%u local_max=%u total_max=%u\r\n",
                         (unsigned long)checked_age, (unsigned int)task_scene.source_age_ms,
                         (unsigned int)APP_TASK_TRACK_MAX_AGE_MS, (unsigned int)APP_TASK_IMAGE_MAX_AGE_MS);
    }
    if(strcmp(scene_reason, "camera_angle") == 0){
        App_USART1_Printf("[SCENE] camera=%.1f expected=%.1f tol=%.1f\r\n",
                         servo_2.Angle, camera_angle, APP_TASK_SERVO_TOL);
    }
    if(strcmp(stage, "VERIFY") == 0){
        App_USART1_Printf("[VERIFY] frames=%u/%u hits=%u/%u wait=%s\r\n",
                         (unsigned int)verify_frames, (unsigned int)APP_TASK_VERIFY_FRAMES,
                         (unsigned int)verify_hits, (unsigned int)APP_TASK_VERIFY_MIN_HITS, verify_wait);
        App_USART1_Printf("[VERIFY] new=%u duplicate=%u invalid=%u cargo=%s count=%u idx=%d\r\n",
                         (unsigned int)verify_new_total, (unsigned int)verify_duplicates,
                         (unsigned int)verify_invalid, cargo_reason, (unsigned int)cargo_count, cargo_bad_index);
        App_USART1_Printf("[VERIFY] region=(%.0f..%.0f,%.0f..%.0f) ref=(%.0f,%.0f)\r\n",
                         APP_TASK_CARGO_X_MIN, APP_TASK_CARGO_X_MAX, APP_TASK_CARGO_Y_MIN,
                         APP_TASK_CARGO_Y_MAX, APP_TASK_IMAGE_W, APP_TASK_IMAGE_H);
        if(task_scene.image_width != 0 && task_scene.image_height != 0){
            for(uint8_t i = 0; i < task_scene.object_count && i < RESCUE_MAX_OBJECTS; ++i){
                const RescueObject *o = &task_scene.objects[i];
                float x = App_Task_X(o), y = App_Task_Y(o);
                unsigned int inside = x >= APP_TASK_CARGO_X_MIN && x <= APP_TASK_CARGO_X_MAX
                                      && y >= APP_TASK_CARGO_Y_MIN && y <= APP_TASK_CARGO_Y_MAX;
                unsigned int entity = !((o->type >= RESCUE_OBJECT_RED_SAFE_ZONE && o->type <= RESCUE_OBJECT_START_ZONE)
                                        || o->type >= RESCUE_OBJECT_RED_SUPPLY_ZONE);
                App_USART1_Printf("[VERIFY] obj=%u type=%u conf=%u xy=(%.1f,%.1f) in=%u entity=%u\r\n",
                                 (unsigned int)i, (unsigned int)o->type, (unsigned int)o->confidence,
                                 x, y, inside, entity);
            }
        }
    }else if(strcmp(stage, "SELECT") == 0){
        App_USART1_Printf("[SELECT] detail=%s empty=%u/%u\r\n", select_wait,
                         (unsigned int)search_empty_frames, (unsigned int)APP_TASK_SEARCH_EMPTY_FRAMES);
    }else if(strcmp(stage, "SCATTER") == 0){
        App_USART1_Printf("[SCATTER] detail=%s idx=%d camera=%.1f frame=%.1f\r\n",
                         scatter_reason, scatter_bad_index, camera_angle, servo_1.Angle);
        if(scatter_bad_index >= 0 && scatter_bad_index < task_scene.object_count){
            const RescueObject *o = &task_scene.objects[scatter_bad_index];
            App_USART1_Printf("[SCATTER] type=%u conf=%u xy=(%u,%u)\r\n", (unsigned int)o->type,
                             (unsigned int)o->confidence, (unsigned int)o->cx, (unsigned int)o->cy);
        }
    }else {
        App_USART1_Printf("[MATCH] reason=%s reject(type,conf,id,w,h,dist)=(%u,%u,%u,%u,%u,%u)\r\n",
                         match_reason, (unsigned int)match_reject[0], (unsigned int)match_reject[1],
                         (unsigned int)match_reject[2], (unsigned int)match_reject[3],
                         (unsigned int)match_reject[4], (unsigned int)match_reject[5]);
        App_USART1_Printf("[MATCH] best2=%.1f second2=%.1f gate=%.1f ambiguity2=%.1f\r\n",
                         match_best_score, match_second_score, APP_TASK_TRACK_GATE,
                         APP_TASK_TRACK_AMBIGUITY * APP_TASK_TRACK_AMBIGUITY);
        App_USART1_Printf("[TRACK] type=%u conf=%u id=%u xy=(%u,%u) wh=(%u,%u)\r\n",
                         (unsigned int)diagnostic_target.type, (unsigned int)diagnostic_target.confidence,
                         (unsigned int)diagnostic_target.track_id, (unsigned int)diagnostic_target.cx,
                         (unsigned int)diagnostic_target.cy, (unsigned int)diagnostic_target.width,
                         (unsigned int)diagnostic_target.height);
        if(strcmp(stage, "ZONE") == 0){
            int index = zone_path_index >= 0 ? zone_path_index : zone_bad_index;
            App_USART1_Printf("[ZONE] detail=%s path_idx=%d candidate_idx=%d\r\n",
                             zone_reason, zone_path_index, zone_bad_index);
            if(diagnostic_zone_valid || (index >= 0 && index < task_scene.object_count + derived_zone_count)){
                const RescueObject *o = diagnostic_zone_valid ? &diagnostic_zone_object : App_Task_ObjectAt((uint8_t)index);
                App_USART1_Printf("[ZONE] other type=%u conf=%u xy=(%u,%u) wh=(%u,%u)\r\n",
                                 (unsigned int)o->type, (unsigned int)o->confidence, (unsigned int)o->cx,
                                 (unsigned int)o->cy, (unsigned int)o->width, (unsigned int)o->height);
            }
        }
    }
}

// @简介：视觉步骤统一失败出口，立即撤销运动，不直接改变上层任务状态。
static App_Task_Result App_Task_Vision_Failed(void) // 函数入口：视觉步骤统一失败出口，立即撤销运动，不直接改变上层任务状态。
{ // 开始App_Task_Vision_Failed函数体。
    App_Task_Move_Stop(); // 撤销位置/角度控制并将左右轮目标速度置零。
    return APP_STEP_FAILED; // 报告本步骤失败，不再批准后续动作。
} // 结束App_Task_Vision_Failed函数体。

// 安全区失配立即撤销前进并旋转重搜；相机俯角不变，保留当前view_id和已处理帧记录。
static App_Task_Result App_Task_ZoneResearch(const char *reason)
{
    App_Task_Move_Stop();
    diagnostic_target = target; // 清理控制目标前保留失败现场。
    int index = zone_path_index >= 0 ? zone_path_index : zone_bad_index;
    diagnostic_zone_valid = index >= 0 && index < task_scene.object_count + derived_zone_count;
    if(diagnostic_zone_valid){ diagnostic_zone_object = *App_Task_ObjectAt((uint8_t)index); }
    target = (RescueObject){0};
    target_zone = 0;
    target_zone_derived = 0;
    follow_forward = 0;
    lost_frames = 0;
    arrive_frames = 0;
    derived_zone_count = 0;
    App_LinePID_Reset();
    App_Task_Move_Speed(APP_TASK_ZONE_SPIN_SPEED, -APP_TASK_ZONE_SPIN_SPEED);
    if(APP_TASK_DEBUG_ENABLE){ App_USART1_Printf("[ZONE] research=%s\r\n", reason); }
    App_Task_Vision_LogDetails("ZONE"); // 保持原先立即旋转的顺序，再打印已冻结的失败现场。
    return APP_STEP_RESEARCH;
}

// @简介：跟随失败时先停车，再打印一次原因和最后保存的图像/目标信息。
// @说明：phase为0表示首次对准、1表示前进；读取场景失败时，字段仍是最后保存值。
static App_Task_Result App_Task_FollowFailed(const char *reason) // reason为本次退出原因，不周期打印。
{ // 开始失败诊断函数。
    if(target_zone){ return App_Task_ZoneResearch(reason); } // 安全区重新搜索，不结束整段救援任务。
    App_Task_Result result = App_Task_Vision_Failed(); // 先撤销运动，避免串口发送延后停车。
    if(APP_TASK_DEBUG_ENABLE){ // 与任务、转向日志使用同一个调试开关。
        App_USART1_Printf("[FOLLOW] stop=%s phase=%u dt=%lu\r\n", reason, (unsigned int)follow_forward, (unsigned long)(HAL_GetTick() - follow_time)); // 输出退出原因、阶段与阶段已运行毫秒数。
        App_USART1_Printf("[VISION] frame=%u age=%lu view=%u/%u flags=%u n=%u\r\n", (unsigned int)task_scene.frame_id, (unsigned long)(HAL_GetTick() - task_scene.received_ms), (unsigned int)task_scene.view_id, (unsigned int)view_id, (unsigned int)task_scene.flags, (unsigned int)task_scene.object_count); // 输出最后保存图像的本地年龄、收到/期望视角和标志。
        App_USART1_Printf("[TARGET] xy=(%u,%u) wh=(%u,%u) img=(%u,%u) lost=%u mixed=%u\r\n", (unsigned int)target.cx, (unsigned int)target.cy, (unsigned int)target.width, (unsigned int)target.height, (unsigned int)target_image_w, (unsigned int)target_image_h, (unsigned int)lost_frames, (unsigned int)target_mixed); // 输出锁定目标原始像素框、锁定时分辨率、丢失帧数及撞散用途。
    } // 结束调试开关判断。
    App_Task_Vision_LogDetails(strcmp(reason, "scatter_unsafe") == 0 ? "SCATTER" : "FOLLOW"); // 打印实际失败子检查。
    return result; // 保持原来的失败返回值，上层随后回到停车状态。
} // 结束失败诊断函数。

// @简介：未匹配原目标时先停车；连续有效新帧均找不到才结束，旧锁定框保持不变。
// @说明：同帧/坏图不在此累计；恢复匹配仍需原ID（如有）、位置/尺寸门限，不能改选其它绿色。
static App_Task_Result App_Task_TargetMissing(void) // 函数入口：未匹配原目标时先停车；连续有效新帧均找不到才结束，旧锁定框保持不变。
{ // 开始App_Task_TargetMissing函数体。
    if(target_zone){ return App_Task_ZoneResearch("target_missing"); } // 安全区不等待3帧或600ms。
    App_Task_Move_Stop(); // 撤销位置/角度控制并将左右轮目标速度置零。
    capture_y_direction = 0; // 丢失时清除纵向实际运动方向，恢复后必须重新稳定确认。
    capture_y_request = 0; // 清除丢失前等待恢复的方向请求。
    capture_y_frames = 0; // 清除丢失期间未处理新图对应的连续帧确认。
    arrive_frames = 0; // 清零连续到位确认帧数，禁止继承旧确认结果。
    if(lost_frames == 0){ // 第一次未匹配原目标时，记录本次短暂丢失的起点。
        lost_time = HAL_GetTick(); // 先记录原计时起点，再打印；日志不延后丢失超时起点。
        if(APP_TASK_DEBUG_ENABLE){ App_USART1_Printf("[MATCH] wait=%s lost=1/%u\r\n", match_reason, (unsigned int)APP_TASK_LOST_FRAMES); }
        App_Task_Vision_LogDetails(target_zone ? "ZONE" : "FOLLOW"); // 已停车，只在首次丢失时打印关联失败现场。
        App_LinePID_Reset(); // 清空视觉PID历史误差及死区状态，避免旧输出干扰新阶段。
    } // 结束条件分支。
    if(++lost_frames >= APP_TASK_LOST_FRAMES){ return App_Task_FollowFailed("target_lost"); } // 连续有效新帧均找不到原目标达到门限后确认丢失并失败。
    return APP_STEP_WAIT; // 保持当前安全状态，等待下一张新图或机械稳定。
} // 结束App_Task_TargetMissing函数体。

// @简介：到位先停车，连续新图位置稳定且经过制动等待才批准下一步机械动作。
// @说明：中心x必须始终满足前框容差；y仅允许少量回落。位置漂移则重新计数/等待。
static App_Task_Result App_Task_Arrive(float x, float y) // 函数入口：到位先停车，连续新图位置稳定且经过制动等待才批准下一步机械动作。
{ // 开始App_Task_Arrive函数体。
    App_Task_Move_Stop(); // 撤销位置/角度控制并将左右轮目标速度置零。
    if(App_Task_Abs(x - APP_TASK_AIM_X) > APP_TASK_ARRIVE_X){ // 原横向条件保持不变，补充具体到位值。
        if(APP_TASK_DEBUG_ENABLE){ App_USART1_Printf("[ARRIVE] x=%.1f aim=%.1f tol=%.1f y=%.1f\r\n", x, APP_TASK_AIM_X, APP_TASK_ARRIVE_X, y); }
        return App_Task_FollowFailed("arrive_off_center");
    }
    if(arrive_frames == 0 || App_Task_Abs(x - arrive_x) > APP_TASK_ARRIVE_STABLE_PX // 首次复核或目标x相对本轮基准漂移过大时重新建立稳定窗口。
       || App_Task_Abs(y - arrive_y) > APP_TASK_ARRIVE_STABLE_PX){ // 目标y漂移过大同样重新计时与计帧。
        arrive_x = x; // 保存本轮到位确认的目标x位置基准。
        arrive_y = y; // 保存本轮到位确认的目标y位置基准。
        arrive_time = HAL_GetTick(); // 重新记录停车稳定等待起点。
        arrive_frames = 0; // 清零连续到位确认帧数，禁止继承旧确认结果。
    } // 结束到位确认基准及计时窗口的重建。
    if(arrive_frames < APP_TASK_ARRIVE_FRAMES){ arrive_frames++; } // 每张新图增加一次到位计数，达到门限后停止累加。
    return arrive_frames >= APP_TASK_ARRIVE_FRAMES // 到位批准必须满足连续稳定新帧数量要求。
           && (uint32_t)(HAL_GetTick() - arrive_time) >= APP_TASK_ARRIVE_WAIT_MS // 同时必须已经停车等待足够的制动及稳定时间。
           ? APP_STEP_DONE : APP_STEP_WAIT; // 帧数和时间都满足才返回完成，否则保持停车等待。
} // 结束App_Task_Arrive函数体。

// @简介：单独绿色的纵向P控制；方向切换先停车，越过外门限且稳定后再恢复。
// @说明：Y误差为目标Y减当前Y，正值前进、负值倒车；停车带内只做横向补正和到位复核。
static App_Task_Result App_Task_CaptureFollow(float x, float y) // x、y为640x480参考坐标系中的目标中心。
{ // 开始单独绿色纵向跟随函数体。
    float error = LINE_PID_STOP_Y - y; // 目标比停车位置远时为正，要求前进；过近时为负，要求倒车。
    float abs_error = App_Task_Abs(error); // 取得纵向像素误差绝对值，用于停车带和速度计算。
    int8_t request = error >= 0.0f ? 1 : -1; // 将误差方向转换为前进1或倒车-1。
    float speed; // 保存Y轴比例控制算出的轮速幅值，单位cm/s。
    uint32_t now = HAL_GetTick(); // 读取当前时间，供方向稳定等待使用。

    if(abs_error <= LINE_PID_STOP_Y_BAND){ // 目标中心进入400±10像素的真实停车带。
        capture_y_direction = 0; // 停止纵向控制，不让微小误差引起前后摆动。
        capture_y_request = 0; // 清除待恢复方向，滞回停车本身不代表到位完成。
        capture_y_frames = 0; // 清空方向确认帧数。
        App_Task_Move_Stop(); // 停车后再判断是否满足落框所需的位置条件。
        if(App_Task_Abs(x - APP_TASK_AIM_X) <= APP_TASK_ARRIVE_X){ // 纵向和横向都在允许范围才进入落框稳定复核。
            return App_Task_Arrive(x, y); // 按连续稳定新帧及最短停车时间确认到位。
        } // 结束到位位置判断。
        App_Task_VisualDrive(x, 0.0f, 0); // Y已到位但X偏差较大时，只原地用横向PID转正。
        return APP_STEP_RUNNING; // 等待下一张新图重新判断XY位置。
    } // 结束停车带处理。

    arrive_frames = 0; // 离开停车带后撤销落框到位计数。
    if(capture_y_direction != 0){ // 当前正在纵向移动时先判断是否需要改变方向。
        if(request != capture_y_direction){ // 误差方向反转时不直接反向驱动，先停车进入稳定等待。
            App_Task_Move_Stop(); // 立即停止，避免越过目标后前后猛切。
            capture_y_direction = 0; // 标记纵向控制已经停车。
            capture_y_request = request; // 保存新的反向请求，后续必须重新确认。
            capture_y_frames = 1; // 当前新图作为反向确认的第一帧。
            capture_y_wait_time = now; // 从停车时刻开始累计方向切换等待。
            return APP_STEP_WAIT; // 本帧只停车，不下发反向速度。
        } // 结束方向反转判断。
    }else { // 当前已停车，检查是否达到滞回外门限并满足稳定重启条件。
        if(abs_error < LINE_PID_STOP_Y_BAND + LINE_PID_Y_HYSTERESIS){ // 外门限默认20px；门限内保持停车且不置到位标志。
            capture_y_request = 0; // 清除尚未越过外门限的方向请求。
            capture_y_frames = 0; // 清空不满足恢复条件时的连续新帧计数。
            App_Task_Move_Stop(); // 保持停车，不把滞回区误当作到位区。
            if(App_Task_Abs(x - APP_TASK_AIM_X) > APP_TASK_ARRIVE_X){ // X仍偏离前框范围时允许原地修正朝向。
                App_Task_VisualDrive(x, 0.0f, 0); // 只调整横向，不执行纵向平移。
            } // 结束横向修正判断。
            return APP_STEP_WAIT; // 等待误差明确越过外门限，或下一帧进入真实停车带。
        } // 结束外门限判断。
        if(capture_y_request != request){ // 首次请求或方向变更时，重建方向稳定确认窗口。
            capture_y_request = request; // 保存当前请求方向。
            capture_y_frames = 1; // 当前新图计为该方向的第一帧。
            capture_y_wait_time = now; // 记录停车稳定等待起点。
        }else if(capture_y_frames < LINE_PID_Y_RESUME_FRAMES){ // 请求方向未改变时，累计同方向的新图。
            capture_y_frames++; // 增加连续同方向帧数，重复帧已在调用处过滤。
        } // 结束请求方向帧数更新。
        if(capture_y_frames < LINE_PID_Y_RESUME_FRAMES // 连续同方向新图不足时继续保持停车。
           || (uint32_t)(now - capture_y_wait_time) < LINE_PID_Y_RESUME_WAIT_MS){ // 或停车等待时间尚未达到250ms时保持停车。
            App_Task_Move_Stop(); // 在确认窗口内保持小车停车。
            return APP_STEP_WAIT; // 等待后续新帧或时间满足恢复条件。
        } // 结束运动恢复条件判断。
        capture_y_direction = request; // 同方向新帧和停车时间都满足后，允许按该方向运动。
    } // 结束停车状态下的滞回与恢复判断。

    speed = abs_error * LINE_PID_Y_KP; // 纵向P控制：离目标越远，前后调整速度越大。
    if(speed < LINE_PID_Y_MIN_SPEED){ speed = LINE_PID_Y_MIN_SPEED; } // 误差仍在运动区时施加最小轮速，克服电机静摩擦。
    if(request > 0 && speed > LINE_PID_Y_MAX_FORWARD){ speed = LINE_PID_Y_MAX_FORWARD; } // 限制向前接近的最大速度。
    if(request < 0 && speed > LINE_PID_Y_MAX_REVERSE){ speed = LINE_PID_Y_MAX_REVERSE; } // 倒车方向速度上限更低，减少过近修正冲击。
    App_Task_VisualDrive(x, request > 0 ? speed : -speed, 1); // 按Y轴P速度前进/倒车，同时用横向视觉PID差速对准。
    return APP_STEP_RUNNING; // 纵向调整仍在进行，主循环继续处理新图和速度闭环。
} // 结束单独绿色纵向跟随函数体。

// @简介：初始化队伍及跟踪用途；view_id不清零，重试不能复用之前的视角编号。
void App_Task_Vision_Init(uint8_t team) // 函数入口：初始化队伍及跟踪用途；view_id不清零，重试不能复用之前的视角编号。
{ // 开始App_Task_Vision_Init函数体。
    team_supply_type = team == APP_TASK_TEAM_RED ? RESCUE_OBJECT_RED_SUPPLY_ZONE : RESCUE_OBJECT_BLUE_SUPPLY_ZONE; // 按启动队伍锁存红方或蓝方物资放置区的协议类型。
    camera_angle = SERVO2_LIFT; // 本阶段期望俯角初始化为正常观察角。
    target_mixed = 0; // 清除撞散用途标志，后续由目标选择重新确定。
    target_zone = 0; // 清除目的分区标志，初始化为场地物块用途。
    target_zone_derived = 0;
    derived_zone_count = 0;
    App_LinePID_Reset(); // 清空视觉PID历史误差及死区状态，避免旧输出干扰新阶段。
} // 结束App_Task_Vision_Init函数体。

// @简介：开始新视角，清除上一阶段的新帧和复检计数；等待由调用方指定，0表示立即发送新编号。
void App_Task_Vision_Begin(float angle, uint32_t settle_ms) // 函数入口：开始新视角并指定稳定等待，0表示立即发送新编号。
{ // 开始App_Task_Vision_Begin函数体。
    App_Task_Move_Stop(); // 撤销位置/角度控制并将左右轮目标速度置零。
    camera_angle = angle; // 保存本阶段期望俯角，后续对照舵机结构体设定角检查。
    view_begin_time = HAL_GetTick(); // 记录新视角开始时刻，从此计算画面稳定等待。
    view_settle_ms = settle_ms; // 保存本次等待时长，不改变右转、换向或撞散重选的等待参数。
    view_id++; // 递增视角编号，使上一阶段迟到图像不再有效。
    if(view_id == 0){ view_id = 1; } // 编号回绕到0时跳过零值，继续使用非零上下文编号。
    view_sent = 0; // 标记新视角上下文尚未成功发送。
    view_send_failed = 0; scene_snapshot = 0;
    scene_reason = match_reason = cargo_reason = zone_reason = scatter_reason = "not_checked";
    verify_wait = select_wait = "new_view";
    verify_reported = select_reported = zone_reported = 0;
    verify_new_total = verify_duplicates = verify_invalid = 0;
    cargo_bad_index = zone_bad_index = zone_path_index = scatter_bad_index = -1;
    cargo_count = 0; checked_age = 0; diagnostic_zone_valid = 0;
    diagnostic_target = target;

    frame_seen = 0; // 清除已见帧标志，重新建立本阶段帧去重基准。
    verify_frames = 0; // 清零本轮复检已检查的新图数量。
    verify_hits = 0; // 清零本轮复检合格新图数量，不能继承上一阶段的命中。
    search_empty_frames = 0; // 清零连续无候选计数，重新判断本视角是否无目标。
    derived_zone_count = 0;
    lost_frames = 0; // 清零原目标连续未匹配的新帧计数。
    arrive_frames = 0; // 清零连续到位确认帧数，禁止继承旧确认结果。
    App_Task_UpdateView(); // 无稳定等待时本次调用立即发新编号；有等待的阶段仍到时后再发。
} // 结束App_Task_Vision_Begin函数体。

// @简介：初步调试只选择可信绿色，不判断其它检测框是否与其相邻。
// @说明：沿用框底边更低优先、同底边时更靠近瞄准中心x的排序；锁定后不重新排名。
//        暂不根据相邻关系选择物资堆或自动进入撞散分支。
int App_Task_Vision_Select(void) // 函数入口：从完整有效的新图中选择绿色物资，不检查邻居。
{ // 开始App_Task_Vision_Select函数体。
    int best = -1; // 最佳绿色候选下标初始化为未找到。
    uint8_t low_green = 0; // 只记录绿色自身低置信的数量。
    uint8_t i; // 声明遍历场景目标数组的下标。
    if(App_Task_ReadScene() == 0){ search_empty_frames = 0; select_wait = scene_reason; App_Task_LogWait("SELECT", select_wait, &select_reported); return -1; } // 坏图不作为空图累计，清除判空计数后继续等待。
    if(App_Task_NewFrame() == 0){ select_wait = "duplicate_frame"; return -1; } // 不刷屏，超时时仍能区分重复帧等待。
    for(i = 0; i < task_scene.object_count; i++){ // 逐项检查相机上报的目标，只按本框信息筛选绿色。
        const RescueObject *object = &task_scene.objects[i]; // 取得当前候选的只读指针。
        float error = App_Task_Abs(App_Task_X(object) - APP_TASK_AIM_X); // 计算候选偏离车头中心x的误差，用于排序。
        if(object->type != RESCUE_OBJECT_SUPPLY // 绿色仍是当前可落框目标，其它类别不作为候选。
           || object->confidence < APP_TASK_CONFIDENCE_MIN){ if(object->type == RESCUE_OBJECT_SUPPLY){ ++low_green; } continue; } // 仅记录原有筛选原因。
        if(best < 0){ best = i; continue; } // 首个合格绿色直接保存。
        if(App_Task_Bottom(object) > App_Task_Bottom(&task_scene.objects[best]) // 框底边更靠下的绿色优先。
           || (App_Task_Bottom(object) == App_Task_Bottom(&task_scene.objects[best]) // 底边相同时比较横向偏差。
               && error < App_Task_Abs(App_Task_X(&task_scene.objects[best]) - APP_TASK_AIM_X))){ best = i; } // 优先选择更靠近瞄准中心x的绿色。
    } // 结束绿色候选遍历循环。
    if(best >= 0){ // 找到可信绿色后锁定，沿用原单件对准、接近和落框流程。
        search_empty_frames = 0; // 清除连续无候选计数。
        select_wait = "ok"; select_reported = 0;
        App_Task_Lock((uint8_t)best, 0, 0); // 锁定绿色物资；不按相邻关系标记为撞散用途。
        return 1; // 返回已锁定目标，由上层停车并切换到跟随。
    } // 结束找到绿色分支。
    select_wait = low_green ? "green_low_confidence" : "no_green";
    App_Task_LogWait("SELECT", select_wait, &select_reported);
    return ++search_empty_frames >= APP_TASK_SEARCH_EMPTY_FRAMES ? 0 : -1; // 保留原无候选返回约定，启动旋转搜索收到0仍继续转。
} // 结束App_Task_Vision_Select函数体。

// @简介：报告已锁定目标用途；只有动作结束重检才会重新选择单件或撞散分支。
uint8_t App_Task_Vision_IsMixed(void) // 函数入口：报告已锁定目标用途；只有动作结束重检才会重新选择单件或撞散分支。
{ // 开始App_Task_Vision_IsMixed函数体。
    return target_mixed; // 返回锁定时确定的撞散用途标志，不随列表顺序改变。
} // 结束App_Task_Vision_IsMixed函数体。

// @简介：物块/本队物资区共用的非阻塞接近函数，封装首次对准、目标关联及视觉PID。
// @返回：DONE=到位停车复核通过；FAILED=连续丢失/超时/阻挡；其它结果继续调用。
// @说明：首次原地对准后只向前差速，不再原地纠偏；帧间只跑速度闭环。
App_Task_Result App_Task_Vision_Follow(void) // 函数入口：物块/本队物资区共用的非阻塞接近函数，封装首次对准、目标关联及视觉PID。
{ // 开始App_Task_Vision_Follow函数体。
    diagnostic_target = target; // 保存判断前原目标，供失败及安全区重搜打印。
    diagnostic_zone_valid = 0; zone_bad_index = zone_path_index = -1;
    zone_reason = scatter_reason = "not_checked"; // 本次未执行的子检查不沿用上次的ok或失败原因。
    match_reason = "not_checked"; memset(match_reject, 0, sizeof(match_reject));
    match_best_score = match_second_score = 0.0f;
    int index; // 保存本帧与原锁定目标关联成功的下标。
    uint8_t near; // 记录目标是否进入停车门限附近的减速范围。
    uint8_t capture = target_zone == 0 && target_mixed == 0; // 仅单独绿色落框使用已确认的Y目标±10，物资堆及分区保持原门限。
    float y; // 保存目标中心的归一化y坐标，向下增大。
    float speed; // 保存本次视觉驱动的基础线速度，单位cm/s。
    float stop_y = target_zone ? APP_TASK_ZONE_STOP_Y // 锁定为目的分区时使用物资区中心y停车门限。
                              : (target_mixed ? LINE_PID_SCATTER_STOP_Y : LINE_PID_STOP_Y); // 场地物块根据撞散或单绿色用途选择对应停车门限。
    if(capture){ stop_y -= LINE_PID_STOP_Y_BAND; } // 单独绿色进入390～410即开始停车复核，下界390用于停止继续前进。
    uint32_t timeout = follow_forward ? APP_TASK_APPROACH_TIMEOUT_MS : APP_TASK_ALIGN_TIMEOUT_MS; // 按当前对准或持续前进阶段，选择对应超时上限。
    if((uint32_t)(HAL_GetTick() - follow_time) >= timeout){ // 保持原阶段超时优先级，区分首次对准和前进超时。
        return App_Task_FollowFailed(follow_forward ? "approach_timeout" : "align_timeout"); // 停车并输出当前阶段超时原因。
    } // 结束阶段超时判断。
    if(App_Task_ReadScene() == 0){ return App_Task_FollowFailed("scene_invalid"); } // 图像不可用时停车，最后保存的图像字段仅供排查。
    if(lost_frames != 0 && (uint32_t)(HAL_GetTick() - lost_time) >= APP_TASK_LOST_TIMEOUT_MS){ // 保持原来的目标丢失等待上限。
        return App_Task_FollowFailed("lost_timeout"); // 输出等待原目标恢复超时。
    } // 结束丢失等待超时判断。
    if(task_scene.image_width != target_image_w || task_scene.image_height != target_image_h){ // 保持原来的图像分辨率一致性检查。
        return App_Task_FollowFailed("image_size_changed"); // 输出锁定后分辨率发生变化。
    } // 结束分辨率判断。
    if(App_Task_NewFrame() == 0){ App_Task_Move_Proc(); return APP_STEP_RUNNING; } // 没有新图时仅继续速度内环，不重复计算视觉误差或到位帧数。
    index = App_Task_MatchTarget(); // 通过类型、ID、二维位移及框尺寸判断本帧哪个候选是原目标。
    if(index < 0){ return App_Task_TargetMissing(); } // 未可靠匹配原目标时先停车，再累计连续丢失新帧。
    if(target_zone){ // 目的分区跟随还必须确认本队物资分区唯一可信。
        if(App_Task_FindZone() != index){ return App_Task_FollowFailed("zone_ambiguous"); } // 唯一可信分区与关联结果不一致时拒绝继续跟随。
    }else if(target_mixed != 0 && App_Task_ScatterSceneClear() == 0){ // 保留撞散接口原场景检查；当前绿色选择不自动触发撞散。
        return App_Task_FollowFailed("scatter_unsafe"); // 撞散用途的整幅场景检查失败时仍报告原原因。
    } // 结束用途检查，普通绿色不再检查相邻实体或拥挤情况。
    target = *App_Task_ObjectAt(index); // 每帧更新真实检测或重新推导的分区位置。
    target_zone_derived = target_zone && index >= task_scene.object_count;
    if(target_zone && App_Task_ZonePathClear() == 0){ return App_Task_FollowFailed("zone_path_blocked"); } // 目的分区通道受阻时报告并停车。
    if(lost_frames != 0){ // 当前已找回之前短暂丢失的同一目标，执行恢复前的状态清理。
        lost_frames = 0; // 清零原目标连续未匹配的新帧计数。
        App_Task_Move_Stop(); // 撤销位置/角度控制并将左右轮目标速度置零。
        App_LinePID_Reset(); // 清空视觉PID历史误差及死区状态，避免旧输出干扰新阶段。
        return APP_STEP_WAIT; // 找回原目标后保持停车一帧，下一张新图恢复原阶段，不再次原地对准
    } // 结束条件分支。
    y = App_Task_Y(&target); // 取原目标本帧中心y，作为纵向控制和停车判断依据。
    if(capture){ // 单独绿色目标的XY对准和接近沿用测试版控制行为。
        if(follow_forward == 0){ // 首次原地对准只用X误差转向，Y轴运动控制从对准完成后开始。
            if(App_LinePID_InDeadband(App_Task_X(&target))){ // 目标x进入横向死区时结束首次对准。
                App_Task_Move_Stop(); // 对准阶段结束时停车，避免把转向输出带入平移。
                App_LinePID_Reset(); // 清除横向PID历史误差及死区状态。
                follow_forward = 1; // 标记首次对准已完成，进入XY纵向跟随阶段。
                follow_time = HAL_GetTick(); // 从首次对准完成时开始计算接近阶段超时。
                capture_y_direction = 0; // 从停车状态开始判定纵向控制方向。
                capture_y_request = 0; // 清除之前可能遗留的待恢复方向。
                capture_y_frames = 0; // 重置纵向方向稳定确认帧数。
                capture_y_wait_time = HAL_GetTick(); // 记录首次平移前的停车等待起点。
                return APP_STEP_RUNNING; // 下一张新图再开始Y轴方向确认和运动。
            } // 结束首次对准完成判断。
            App_Task_VisualDrive(App_Task_X(&target), 0.0f, 0); // 目标尚未对准时仅原地执行X轴视觉PID。
            return APP_STEP_RUNNING; // 对准阶段不执行前后运动。
        } // 结束首次原地对准处理。
        return App_Task_CaptureFollow(App_Task_X(&target), y); // 对准后按测试版Y轴P、滞回和防晃动条件前后调整。
    } // 结束单独绿色跟随分支。
    if(y >= stop_y && App_Task_Abs(App_Task_X(&target) - APP_TASK_AIM_X) <= APP_TASK_ARRIVE_X){ // 到达中心Y门限且横向进入落框容差后，才开始到位停车复核。
        return App_Task_Arrive(App_Task_X(&target), y); // 停车并复核位置稳定性、连续新帧数及等待时长。
    } // 结束到位位置判断。
    if(capture == 0 && arrive_frames != 0 && y >= stop_y - APP_TASK_ARRIVE_Y_BACK){ // 物资堆/分区保留原少量回落容差；绿色必须始终处于390～410。
        return App_Task_Arrive(App_Task_X(&target), y); // 保持停车并复核目标位置。
    } // 结束复核回落容差判断。
    arrive_frames = 0; // 目标回落或尚未对准时清除旧到位计数。
    if(y >= stop_y){ // 已到前进停车门限但横向仍偏离时，禁止继续向前。
        App_Task_VisualDrive(App_Task_X(&target), 0.0f, 0); // 暂停平移，仅用视觉PID原地把目标转到前框中心。
        return APP_STEP_RUNNING; // 对准后由下一张新图进入到位停车复核。
    } // 结束停车门限处的横向对准处理。
    near = stop_y - y <= LINE_PID_SLOW_MARGIN_Y; // 目标中心y距停车门限较近时，启用减速和近处纠偏参数。
    if(follow_forward == 0){ // 首次对准尚未完成时，基础前进速度保持为零。
        if(App_LinePID_InDeadband(App_Task_X(&target))){ // 目标x进入视觉PID死区时结束首次对准。
            App_Task_Move_Stop(); // 撤销位置/角度控制并将左右轮目标速度置零。
            App_LinePID_Reset(); // 清空视觉PID历史误差及死区状态，避免旧输出干扰新阶段。
            follow_forward = 1; // 标记已进入持续前进阶段，之后不再返回首次原地对准。
            follow_time = HAL_GetTick(); // 记录当前首次对准或持续接近阶段的毫秒起点。
            return APP_STEP_RUNNING; // 下一张新图才前进
        } // 结束条件分支。
        speed = 0.0f; // 首次未对准时基础前进速度为零，仅由差速产生原地转向。
    }else { // 前述条件不满足时执行此分支。
        speed = near ? LINE_PID_SLOW_SPEED : LINE_PID_BASE_SPEED; // 持续前进时按距停车门限的远近，选择慢速或正常基础轮速。
    } // 结束其它情况分支。
    App_Task_VisualDrive(App_Task_X(&target), speed, near && follow_forward); // 根据目标x输出左右轮差速，近处前进使用近处PID且限制两轮保持向前。
    return APP_STEP_RUNNING; // 动作仍在进行，主循环需继续调用本函数。
} // 结束App_Task_Vision_Follow函数体。

// @简介：复检失败时保留窗口和物块现场，停车后打印具体原因。
static App_Task_Result App_Task_VerifyFailed(const char *reason)
{
    App_Task_Result result = App_Task_Vision_Failed();
    if(APP_TASK_DEBUG_ENABLE){ App_USART1_Printf("[VERIFY] stop=%s\r\n", reason); }
    App_Task_Vision_LogDetails("VERIFY");
    return result;
}

// @简介：收齐三张有效新图后至少一张恰好一个可信绿色才通过；空区域计入窗口，冲突立即失败。
App_Task_Result App_Task_Vision_Verify(void)
{
    int result;
    if(App_Task_ReadScene() == 0){
        verify_frames = 0; verify_hits = 0; // 原窗口清零规则保持不变。
        ++verify_invalid; verify_wait = scene_reason;
        App_Task_LogWait("VERIFY", verify_wait, &verify_reported);
        return APP_STEP_WAIT;
    }
    if(App_Task_NewFrame() == 0){ ++verify_duplicates; verify_wait = "duplicate_frame"; return APP_STEP_WAIT; }
    ++verify_new_total;
    result = App_Task_CheckCargo();
    if(result < 0){ verify_wait = cargo_reason; return App_Task_VerifyFailed(cargo_reason); }
    verify_frames++;
    if(result == 1){ verify_hits++; verify_reported = 0; } // 有效合格图后，再次出现异常时重新提示。
    verify_wait = result == 1 ? "collecting" : "cargo_empty";
    if(result == 0){ App_Task_LogWait("VERIFY", verify_wait, &verify_reported); }
    if(verify_frames < APP_TASK_VERIFY_FRAMES){ return APP_STEP_WAIT; }
    return verify_hits >= APP_TASK_VERIFY_MIN_HITS ? APP_STEP_DONE : App_Task_VerifyFailed("no_hit_in_window");
}

// @简介：持续顺时针寻找本队物资区；有效新图找到后立即锁定，不做3帧等待。
App_Task_Result App_Task_Vision_SearchZone(void)
{
    if(App_Task_ReadScene() != 0){
        if(App_Task_NewFrame()){
            int index = App_Task_FindZone();
            if(index >= 0){
                const RescueObject *object = App_Task_ObjectAt((uint8_t)index);
                App_Task_Move_Stop();
                if(APP_TASK_DEBUG_ENABLE){
                    App_USART1_Printf("[ZONE] type=%u xy=(%u,%u) wh=(%u,%u) source=%s\r\n", (unsigned int)object->type,
                                     (unsigned int)object->cx, (unsigned int)object->cy, (unsigned int)object->width,
                                     (unsigned int)object->height, index >= task_scene.object_count ? "divider" : "direct");
                }
                zone_reported = 0;
                App_Task_Lock((uint8_t)index, 1, 0);
                return APP_STEP_DONE;
            }
            App_Task_LogWait("ZONE", zone_reason, &zone_reported);
        }
    }else { App_Task_LogWait("ZONE", scene_reason, &zone_reported); }
    App_Task_Move_Speed(APP_TASK_ZONE_SPIN_SPEED, -APP_TASK_ZONE_SPIN_SPEED);
    return APP_STEP_RUNNING;
}

// @简介：接触撞散时目标会移动/遮挡，停止单目标关联，保留图像时效与整幅场景检查。
uint8_t App_Task_Vision_ScatterClear(void)
{
    if(App_Task_ReadScene() == 0){ scatter_reason = scene_reason; return 0; }
    if(camera_angle != SERVO2_LIFT){ scatter_reason = "camera_mode"; return 0; }
    if(task_scene.image_width != target_image_w || task_scene.image_height != target_image_h){
        scatter_reason = "image_size_changed"; return 0;
    }
    if(!(App_Task_Abs(servo_1.Angle - SERVO1_LIFT) < APP_TASK_SERVO_TOL)){
        scatter_reason = "frame_not_lifted"; return 0;
    }
    return App_Task_ScatterSceneClear();
}
