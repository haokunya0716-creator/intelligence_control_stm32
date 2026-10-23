//
// Created by gaoxu on 2026/10/20.
//

#ifndef INTELLIGENT_RESCUE_APP_TASK_CONFIG_H // 检查头文件保护宏，防止重复包含时反复展开声明。
#define INTELLIGENT_RESCUE_APP_TASK_CONFIG_H // 定义头文件保护宏，后续重复包含时跳过本文件。

// ==================== 启动、队伍及整轮计时 ====================
#define APP_TASK_TEAM_RED            0U        // KEY3选择红方；启动后本次任务不允许切换队伍。
#define APP_TASK_TEAM_BLUE           1U        // KEY2选择蓝方；对应蓝方物资区，不接受整体安全区代替。
#define APP_TASK_FORWARD_SPEED       20.0f     // 启动直行的左右轮目标速度，单位cm/s，使用编码器速度闭环。
#define APP_TASK_FORWARD_TIME_MS     2000U     // 启动直行持续时间，单位ms，修改后重新编译。
#define APP_TASK_RIGHT_ANGLE         (-90.0f)  // 保留的相对右转角参数，单位度；当前启动旋转搜索不使用。
#define APP_TASK_LEFT_ANGLE          180.0f    // 保留的相对换向角参数，单位度；当前启动旋转搜索不使用。
#define APP_TASK_TURN_TIMEOUT_MS     5000U     // 单次角度环转向最大时间，单位ms，超时停车。
#define APP_TASK_TURN_TOL_DEG        3.0f      // 任务转向到位角度容差±度；避免原±1度在摩擦死区附近反复微动。
#define APP_TASK_TURN_MIN_SPEED      5.0f      // 未到位时单轮最小转向速度cm/s；到位即停车，须实车调节。
#define APP_TASK_DEBUG_ENABLE        1U        // 串口1打印任务状态及转向开始/到位/超时，便于确认检测计时是否开始。
#define APP_TASK_ROUND_TIME_MS       180000U   // 整轮运行时间，单位ms，失败重试或状态切换均不重置。

// ==================== 舵机角度及机械等待 ====================
#define SERVO1_LIFT                  180.0f    // 一号前框抬起角度，单位度；搜索、撞散和卸下后保持此角度。
#define SERVO1_DROP                  75.0f     // 一号前框落下角度，单位度；单独绿色接近停车后框住物块。
#define SERVO1_SUPPORT               0.0f      // 一号前框支撑角度，单位度；预留脱困动作，当前任务不调用。
#define SERVO2_LIFT                  50.0f     // 二号摄像头正常观察角度，单位度；接近/搜索停车像素按此角度标定。
#define SERVO2_DROP                  50.0f     // 摄像头框内标定保留角度；当前与正常角相同，正式复检不切角度、不等待。
#define APP_TASK_SERVO_WAIT_MS       500U      // 前框落下/抬起后的机械等待ms；摄像头复检不动作，不使用此等待。
#define APP_TASK_SERVO_TOL           0.5f      // 舵机结构体设定角允许误差，单位度；不代表机械精度。

// ==================== 图像坐标、时效及目标关联 ====================
#define APP_TASK_IMAGE_W             640.0f    // 所有视觉门限的参考图像宽度；相机原始x按此宽度归一化。
#define APP_TASK_IMAGE_H             480.0f    // 所有视觉门限的参考图像高度；相机原始y按此高度归一化。
#define LINE_PID_AIM_X               360.0f    // 车头/前框中心在画面中的x；默认屏幕中心，须按安装偏移标定，单位像素。
#define LINE_PID_AIM_Y               240.0f    // 屏幕中心y、停车门限参考；y向下为正，单位归一化像素。
#define APP_TASK_AIM_X               LINE_PID_AIM_X // 通道/目标选择共用的横向中心，与视觉PID保持一致。
#define APP_TASK_SETTLE_TIME_MS      200U      // 撞散重选的画面稳定等待ms；启动旋转搜索、落框后复检及物资区搜索不等待。
#define APP_TASK_VIEW_RETRY_MS       100U      // 视角上下文重发间隔，单位ms；相机须在新图像采集时锁存编号。
#define APP_TASK_DETECT_TIMEOUT_MS   2000U     // 撞散重选最大等待ms，含画面稳定，超时停车；启动旋转搜索不使用。
#define APP_TASK_TRACK_MAX_AGE_MS    200U      // 图像本地接收年龄上限，单位ms，运动时达到此值立即停车。
#define APP_TASK_IMAGE_MAX_AGE_MS    300U      // 扩展帧的采集至发送耗时+串口耗时+本地接收年龄上限ms；旧帧无此信息。
#define APP_TASK_SERIAL_BAUD         115200U   // 视觉串口波特率，用于按8N1帧长估算传输耗时；须与USART2一致。
#define APP_TASK_SEARCH_EMPTY_FRAMES 3U        // 撞散重选连续多少张有效新图无目标才结束；启动旋转搜索仍继续，重复帧不累计。
#define APP_TASK_LOST_FRAMES         3U        // 连续多少张有效新图均未匹配原目标才结束；等待期间停车，不跳换目标。
#define APP_TASK_LOST_TIMEOUT_MS     600U      // 短暂丢失后的最长等待ms；不因重复帧或恢复尝试延长阶段/整轮计时。
#define APP_TASK_CONFIDENCE_MIN      70U       // 目标置信度最低百分数；危险/未知实体即使低置信也不能忽略。
#define APP_TASK_TRACK_GATE          70.0f     // 相邻帧中心移动最大距离，单位归一化像素，超过则关联失败。
#define APP_TASK_TRACK_AMBIGUITY     20.0f     // 最近/次近候选平方距离差小于此值平方时放弃，避免跨物块跳锁。
#define APP_TASK_TRACK_SIZE_RATIO    2U        // 相邻帧检测框宽高变化倍数上限，突然放大/缩小则停车。
#define APP_TASK_PATH_HALF_WIDTH     70.0f     // 车头中心至目标中心通道向两侧扩展量，单位像素，按车宽调整。
#define APP_TASK_PATH_DEPTH_MARGIN   20.0f     // 其它框底边比目标低超过此值时视为前方阻挡，单位像素。

// ==================== 视觉对准与差速前进PID ====================
#define LINE_PID_KP                  0.2f     // 横向误差比例系数，单位cm/s/px；右侧目标使左轮加速。
#define LINE_PID_KI                  0.0f      // 横向误差积分系数，初版关闭；死区内清空积分历史。
#define LINE_PID_KD                  0.0f      // 横向误差微分系数，初版关闭，避免放大图像检测噪声。
#define LINE_PID_DEAD_ENTER          10.0f     // |中心x误差|不超过此值停止纠偏，并允许首次对准结束。
#define LINE_PID_DEAD_EXIT           10.0f     // |中心x误差|达到此值恢复纠偏；两门限之间保持之前状态。
#define LINE_PID_NEAR_ENTER          10.0f      // 进入减速范围后，横向误差不超过此值停止纠偏，单位像素。
#define LINE_PID_NEAR_EXIT           10.0f      // 近处误差达到此值恢复纠偏；应不大于ARRIVE_X，避免到位条件和死区冲突。
#define LINE_PID_STEER_MAX           60.0f      // PID单侧差速输出最大绝对值，单位cm/s。
#define LINE_PID_STEER_SIGN          1.0f      // 正修正量对应右转；实车方向相反时改为-1，而非修改图像坐标。
#define LINE_PID_BASE_SPEED          25.0f      // 视觉接近基础轮速，单位cm/s；绿色、物块堆、安全区共用。
#define LINE_PID_SLOW_SPEED          15.0f      // 接近停车门限时的基础轮速，单位cm/s，须保持正值。
#define LINE_PID_SLOW_MARGIN_Y       40.0f     // 目标中心y距离停车门限不超过此像素时，切换到慢行速度。
#define LINE_PID_FORWARD_RATIO      0.8f      // 平移时差速不超过基础速度绝对值此比例，须在(0,1)，防止内侧轮反向。
#define LINE_PID_STOP_Y              422.0f    // 用户已确认的单独绿色落框目标中心Y，单位480高参考像素，摄像头保持正常观察角。
#define LINE_PID_STOP_Y_BAND         10.0f     // 单独绿色落框Y允许误差±像素，目标400时接受390～410；超出上界时倒车修正。
#define LINE_PID_Y_KP                0.12f     // 单独绿色Y误差比例系数，单位cm/s/px；误差绝对值决定纵向基础速度。
#define LINE_PID_Y_MIN_SPEED         3.0f      // Y误差离开停车带后最小调整速度绝对值，单位cm/s，避免小命令克服不了静摩擦。
#define LINE_PID_Y_MAX_FORWARD       25.0f     // Y轴P控制前进速度上限，单位cm/s。
#define LINE_PID_Y_MAX_REVERSE       3.0f      // Y轴P控制倒车速度绝对值上限，单位cm/s，降低过近修正冲击。
#define LINE_PID_Y_HYSTERESIS        10.0f     // 防晃动外门限在停车带基础上的额外像素，内10/外20 px形成滞回。
#define LINE_PID_Y_RESUME_WAIT_MS    200U      // 启动或改变前后运动方向后，至少停车等待的时间。
#define LINE_PID_Y_RESUME_FRAMES     3U        // 启动或改变方向后，连续同方向的新图数量达到此值才恢复移动。
#define LINE_PID_SCATTER_STOP_Y      LINE_PID_STOP_Y // 物块堆也先走到面前，默认与绿色相同；到此之后才额外前推。
#define APP_TASK_ZONE_STOP_Y         (LINE_PID_AIM_Y + 120.0f) // 本队物资区中心y到360停车抬框；像素不证明进入围栏。
#define APP_TASK_ALIGN_TIMEOUT_MS    50000U     // 首次原地视觉对准的最大时间，单位ms；前进后不回到原地对准。
#define APP_TASK_APPROACH_TIMEOUT_MS 15000U    // 完成首次对准后接近阶段最大时间，单位ms，状态内不重复延长。

// ==================== 到位停车复核（绿色、物资堆及物资区共用） ====================
#define APP_TASK_ARRIVE_X            10.0f     // 允许执行落框/前推/卸下的横向误差px；按前框有效宽度实测，不代表厘米。
#define APP_TASK_ARRIVE_Y_BACK       3.0f      // 停车复核时允许中心y回落到门限以下的像素，防止检测微抖造成反复起步。
#define APP_TASK_ARRIVE_STABLE_PX    6.0f      // 复核中心相对首张确认图允许的x/y波动px；超出清零重计，防止移动目标被批准。
#define APP_TASK_ARRIVE_WAIT_MS      150U      // 首次到位停车后最少等待ms；需同时满足连续新帧数量，不以时间代替图像确认。
#define APP_TASK_ARRIVE_FRAMES       3U        // 停车后连续新帧均在到位区域且位置稳定才完成；同帧不重复累计。

// ==================== 摄像头固定角度复检前框内部 ====================
#define APP_TASK_VERIFY_TIMEOUT_MS   3000U     // 前框落稳后复检最大时间ms；立即发新view_id，不额外等待画面稳定。
#define APP_TASK_VERIFY_FRAMES       3U        // 一次复检检查的有效新图数量；区域为空也计数，重复帧不累计。
#define APP_TASK_VERIFY_MIN_HITS     1U        // 收齐三张新图后至少一张区域内恰好一个可信绿色才通过；冲突仍立即停车。
#define APP_TASK_CARGO_X_MIN         230.0f    // 固定角度复检物块中心x下限，包含边界；与标定测试x相同，按640宽归一化。
#define APP_TASK_CARGO_X_MAX         490.0f    // 固定角度复检物块中心x上限，包含边界；不检查检测框左右边缘。
#define APP_TASK_CARGO_Y_MIN         400.0f    // 固定角度复检物块中心y下限，包含边界；与标定测试y相同，按480高归一化。
#define APP_TASK_CARGO_Y_MAX         450.0f    // 固定角度复检物块中心y上限，包含边界；中心在区域外的实体不计数。

// ==================== 编码器直行、后退及安全区搜索 ====================
#define APP_TASK_ZONE_INSET_X        (30.0f / 660.0f) // 整体外框左右各排除名义30mm紫色围栏，实车标定。
#define APP_TASK_ZONE_INSET_Y        (30.0f / 360.0f) // 整体外框上下各排除名义30mm紫色围栏，实车标定。
#define APP_TASK_BACK_SPEED          5.0f      // 后退轮速绝对值，单位cm/s，调用时由运动模块取负。
#define APP_TASK_RELEASE_BACK_CM     10.0f     // 抬框卸下后退离行程，单位cm，退离后前框保持抬起。
#define APP_TASK_BACK_TIMEOUT_MS     5000U     // 一次编码器直线运动超时，单位ms，也用于撞散额外前推。
#define APP_TASK_BACK_SKEW_CM        3.0f      // 两轮行程差最大值，单位cm，超过则停车，防止单轮卡死继续移动。
#define APP_TASK_REVERSE_TOL_CM      1.0f      // 编码器反馈朝指令反方向移动的容许行程，单位cm，超过则停车。
#define APP_TASK_ZONE_SPIN_SPEED     15.0f      // 启动找绿色与找本队物资区共用的单轮线速度cm/s，左正右负顺时针，非角速度。

// ==================== 撞散：接近后额外前推，再限时原地旋转 ====================
#define APP_TASK_SCATTER_PUSH_CM     5.0f      // 到达物块面前后额外前推的编码器行程cm；初始试调值，须实车调整。
#define APP_TASK_SCATTER_PUSH_SPEED  5.0f      // 额外前推基础轮速cm/s，两轮同速，使用速度闭环，初始试调值。
#define APP_TASK_SCATTER_SPIN_SPEED  5.0f      // 撞散旋转单轮线速度cm/s，左正右负顺时针；待实车确定。
#define APP_TASK_SCATTER_SPIN_MS     1000U     // 撞散原地旋转持续时间ms，时间到停车重检，避免无休止旋转。
#define APP_TASK_SCATTER_TIMEOUT_MS  8000U     // 整次额外前推和旋转最大时间ms，应大于预计前推时间加旋转时间。
#define APP_TASK_SCATTER_MAX_COUNT   3U        // 可见普通/核心物资超过3个时不撞散，保留比赛数量限制的保守检查。

#endif // 结束本头文件的重复包含保护。
