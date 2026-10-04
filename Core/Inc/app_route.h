/**
  ******************************************************************************
  * @file    app_route.h
  * @brief   ★ 路线脚本与动作原语（固定地图专用）
  *
  * 【思路】
  *   地图固定 ⇒ 不需要"看见什么再决定"，直接把整条路线写成一段顺序执行的
  *   脚本，每一步调用一个动作原语：
  *
  *       Route_LineFollow(3000);     // 循线走 3 秒
  *       Route_TurnLeft(600);        // 原地左转 0.6 秒
  *       Route_Straight(800);        // 直行 0.8 秒
  *       Route_Servo(180);           // 支撑脚放到 180°
  *       ...
  *
  *   脚本写在 app_route.c 的 APP_Route_Run() 里，改路线只改那一个函数。
  *
  * 【为什么这里的函数是"阻塞"的，却不会把系统卡死】
  *   每个原语内部都有一个小循环（等时间到），循环里会照常做
  *   10ms 控制节拍该做的事：编码器采样、按键扫描、蜂鸣器轮询、支撑脚斜坡。
  *   所以：
  *     · 脚本可以写成一行一行的顺序调用，最好读、最好改
  *     · 期间屏幕照样刷新、按键照样响应
  *     · 长按 K2 随时可以急停并中止整条路线
  *   代价是"路线运行期间主菜单不响应"—— 这是有意的：正在跑路线时，
  *   你不希望有人（或误触）从菜单里插一脚。
  *
  * 【速度用哪些参数】
  *   直行速度  = BASESPD
  *   转弯速度  = PIVOT
  *   循线速度  = BASESPD
  *   都是菜单里已经能调的参数，不再额外加旋钮。
  *
  * 【左右方向的约定】
  *   和循迹、转角模式共用同一套：STEERINV=1 时整车的左右整体对调。
  *   所以装配问题只需要改一个开关，脚本里的 Route_TurnLeft() 永远是"往左"。
  * ============================================================================
  */
#ifndef __APP_ROUTE_H
#define __APP_ROUTE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "bsp_config.h"

/* ==========================================================================
 * 入口
 * ========================================================================== */

/**
  * @brief  在屏幕上问一句"要不要跑路线"：K1 = 开始，K2 = 进菜单
  * @note   一般由 main.c 在初始化完成后调用一次。
  *         选 K2 就直接返回，后面照常进主循环（调参模式）
  */
void APP_Route_Enter(void);

/**
  * @brief  执行整条路线脚本（阻塞，直到脚本跑完或被 K2 长按中止）
  * @note   脚本内容在 app_route.c 的 APP_Route_Run() 里
  */
void APP_Route_Run(void);

/**
  * @brief  ★ 从指定的阶段开始跑路线
  * @param  step 1 = 从头跑，2..APP_Route_StepCount() = 从第几步开始
  *
  * 【为什么要有它】
  *   整条跑一次要几十秒，某一步失误（比如台阶没上去）之后，
  *   如果只能从头再来，试一次的成本非常高。
  *   指定从失误的那一步开始，就能反复练那一步。
  *
  * 【它额外做了两件事】
  *   ① 把脚本里前面那些段整段跳过（见 app_route.c 里的 if (s_from <= N)）
  *   ② ★ 把"横线计数器"垫到前面步骤跑完之后应该有的值 ——
  *      脚本里等的是【绝对第几根横线】，不预置的话它会等一个永远不来的编号
  *      （预置表在 app_route.c 顶部，改路线编号时要一起改）
  */
void APP_Route_RunFrom(uint8_t step);

/** @brief 路线一共几个阶段 */
uint8_t APP_Route_StepCount(void);

/** @brief 第 n 步的短名字（ASCII，给菜单显示用） */
const char *APP_Route_StepName(uint8_t step);

/** @brief 是否正在跑路线 */
uint8_t APP_Route_IsActive(void);

/**
  * @brief  请求中止路线（急停）
  * @note   长按 K2 会自动调用它；脚本里的每个原语都会立刻返回，
  *         所以整条脚本会在几十毫秒内"空转"跑完并停车
  */
void APP_Route_Abort(void);

/* ==========================================================================
 * 动作原语 —— 在脚本里按顺序调用
 * ========================================================================== */

/**
  * @brief  循线 ms 毫秒（用中间那几路做 PID，和菜单里的 START CAR 是同一套）
  * @note   到时间会自动停车。想一直循线就传一个大一点的数
  */
void Route_LineFollow(uint16_t ms);

/**
  * @brief  循线，直到经过 n 个"十字/停止线"（八路全黑）后停下
  * @param  n 需要经过的横线根数
  * @note   ★ 固定地图里最好用的一段式原语：
  *         路线上的横线（路口、停止线）就是天然的"路标"，
  *         用"经过几根横线"来切分路段，比用时间可靠得多
  */
void Route_LineFollowUntilCross(uint8_t n);

/* ==========================================================================
 * ★★ 循迹"结果事件"驱动路段 ★★
 * --------------------------------------------------------------------------
 *   用"某种循迹结果出现了几次"来切分路段，不用时间、也不受电池电压影响。
 *   三个可选事件（图案掩码在 bsp_config.h 的 CAR_EVT_* 里，可按实测改）：
 *
 *       LEFT3  : 最左三路全压线 —— 线跑到车头左边去了
 *       RIGHT3 : 最右三路全压线
 *       BLACK  : 全黑（横线 / 十字 / 停止线）
 *
 *   计数是【累计】的，从 APP_Route_Run() 开头的零开始，
 *   所以脚本里写的是"绝对第几次"：
 *       Route_LineFollowUntilBlack(1u);   // 循线到第 1 根横线
 *       Route_LineFollowUntilBlack(2u);   // 再循线到第 2 根横线
 * ========================================================================== */

/**
  * @brief  ★ 循线，直到某个"结果事件"累计发生到第 n 次
  * @param  evt 要等的事件：APP_CAR_EVT_LEFT3 / RIGHT3 / BLACK
  * @param  n   等到累计第几次（1 = 第一次）
  * @note   ★ 没有时间参数 —— 走到就是走到，没走到就一直在线上找。
  *         安全网是内部固定的 ROUTE_EVENT_TIMEOUT_MS（60 秒）：
  *         这么久还没等到就鸣一声长音、继续往下走，不会永远卡住。
  *         任何时候长按 K2 都能中止整条路线。
  * @note   等到之后【不停车】：下一个动作紧接着开始。
  */
void Route_LineFollowUntilEvent(uint8_t evt, uint16_t n);

/**
  * @brief  ★ 循线，直到某个事件【下一次】发生（不关心这是第几次）
  * @param  evt APP_CAR_EVT_LEFT3 / RIGHT3 / BLACK
  * @note   和 Route_LineFollowUntilEvent() 的区别：
  *           那个写死"绝对第几次"（适合横线：第 4 根、第 5 根这种已知编号）
  *           这个只等"下一次出现"（适合随机出现的图案：
  *           比如"左三压线"在一趟里可能已经出现过好几次，
  *           写死编号就得先去数清楚，很容易对不上）
  */
void Route_LineFollowUntilNextEvent(uint8_t evt);

/** @brief 循线直到"最左三路全压"【下一次】出现 */
void Route_LineFollowUntilNextLeft3(void);

/** @brief 直行 ms 毫秒（两轮同速前进，不看线） */
/** @brief 倒车 ms 毫秒（两轮同速反转，不看线） */
void Route_Reverse(uint16_t ms);

/** @brief 循线直到"全黑（横线）"【下一次】出现（不关心是第几根） */
void Route_LineFollowUntilNextBlack(void);

/** @brief 循线直到"最左三路全压"第 n 次 */
void Route_LineFollowUntilLeft3(uint16_t n);

/** @brief 循线直到"最右三路全压"第 n 次 */
void Route_LineFollowUntilRight3(uint16_t n);

/** @brief 循线直到"全黑（横线/十字）"第 n 次 */
void Route_LineFollowUntilBlack(uint16_t n);

/** @brief 两轮同速直行 ms 毫秒（速度 = BASESPD） */
void Route_Straight(uint16_t ms);

/**
  * @brief  原地左转 ms 毫秒
  * @note   履带车靠两边反向搓地：左履带倒转、右履带正转。
  *         转多少度靠时间标定（跑一次量角度，再按比例改时间）
  */
void Route_TurnLeft(uint16_t ms);

/** @brief 原地右转 ms 毫秒 */
void Route_TurnRight(uint16_t ms);

/**
  * @brief  ★ 支撑脚（舵机）缓慢转到 deg 度，并等它到位
  * @param  deg 0~180。0 = 全收起，180 = 全放下（具体含义看机械装法）
  * @note   走的是 app_leg 的角度斜坡（每 10ms 走 2°），不会猛地顶一下
  */
void Route_Servo(uint8_t deg);

/** @brief 支撑脚立刻到位（不走斜坡）。标定时用，机构脆的话慎用 */
void Route_ServoNow(uint8_t deg);

/**
  * @brief  ★ 预约一次舵机动作："after_ms 毫秒之后把支撑脚移到 deg 度"
  * @param  deg      目标角度 0~180
  * @param  after_ms 从现在起多少毫秒后执行
  *
  * 【为什么需要它 —— 让舵机和行驶"同时进行"】
  *   脚本是顺序执行的，一行不跑完不会跑下一行。可是爬台阶时车必须
  *   【一直不停顿地往前走】【同时让支撑脚按节奏收放】——
  *   这两件事在时间上是重叠的，顺序写法表达不出来。
  *
  *   于是把舵机动作"预约"出去：调用只花几微秒就返回，
  *   真正的动作交给后台节拍按时间点执行。写法：
  *
  *       Route_ServoAt(LEGDN, 1000u);   // 1 秒后放下
  *       Route_ServoAt(LEGUP, 2000u);   // 2 秒后升起
  *       Route_Straight(2000u);         // ★ 这 2 秒车一直在盲走，
  *                                      //   舵机在后台按点动作
  *
  *   注意角度是走 app_leg 的斜坡（每 10ms 走 2°，180° 约 0.9 秒走完），
  *   所以两个时间点之间留够 1 秒以上动作才走得完整；
  *   留得少就是"摆到一半又折回去"，正是快速抖动的效果。
  */
void Route_ServoAt(uint8_t deg, uint16_t after_ms);

/** @brief 停车并等待 ms 毫秒（滑行停） */
void Route_Stop(uint16_t ms);

/** @brief 刹车急停（绕组短接，最快停住） */
void Route_Brake(void);

/** @brief 鸣叫 ms 毫秒（阻塞式，用于分段提示、判断程序走到哪一步了） */
void Route_Beep(uint16_t ms);

/**
  * @brief  ★ 阶段标记：鸣两声（非阻塞）
  * @note   路线里"每过一个阶段"报一下，跑的时候凭声音就知道走到第几步了。
  *         ★ 非阻塞：请求之后立刻返回，两声由后台节拍依次响完 ——
  *         所以它**不会让车在阶段之间停顿**，下一段立刻就开始。
  */
void Route_Beep2(void);

/**
  * @brief  ★ 左右方向自检：原地左转 1 秒 → 停 → 原地右转 1 秒
  * @note   串口命令 T 调用。用来一次性确认"左"到底是哪边 ——
  *         它走的是和路线脚本完全相同的 Route_TurnLeft/Right，
  *         所以这里看到的方向，就是路线里那三步会走的方向。
  *         如果标着左转的却往右走，就是 STEERINV 反了。
  */
void APP_Route_TestTurn(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_ROUTE_H */
