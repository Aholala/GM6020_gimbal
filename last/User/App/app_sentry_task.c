/* app_sentry_task.c
 * FreeRTOS SentryTask 实现
 * 负责：等待 IMU 校准完成 → 读编码器 → 主状态机 → PID → CAN 发送 → USB 上报
 * 运行频率：100Hz（osDelay(10)）
 */

#include "app_sentry_task.h"
#include "app_sentry_globals.h"

void StartSentryTask(void *argument)
{
    const float dt = 0.01f;

    /* ---- 等待 IMU 校准完成（最长 GYRO_CALIB_MS + 裕量）---- */
    while (!imu_calib_done)
    {
        osDelay(10);
    }

    for (;;)
    {
        osDelay(10);  /* 100Hz */

        /* ---------------------------------------------------------------- */
        /* 1. 读取两个电机反馈                                               */
        /* ---------------------------------------------------------------- */
        dbg_angle2  = enc_to_angle(motor_info[MOTOR2_IDX].rotor_angle, zero_enc2);
        dbg_angle4  = enc_to_angle(motor_info[MOTOR4_IDX].rotor_angle, zero_enc4);
        dbg_yaw_enc = dbg_angle2;   /* yaw 编码器反馈，供 Ozone 观测及 USB 上报 */

        float spd2 = (float)motor_info[MOTOR2_IDX].rotor_speed;
        float spd4 = (float)motor_info[MOTOR4_IDX].rotor_speed;

        filtered_spd2 = LPF_ALPHA * spd2 + (1.0f - LPF_ALPHA) * filtered_spd2;
        filtered_spd4 = LPF_ALPHA * spd4 + (1.0f - LPF_ALPHA) * filtered_spd4;

        /* ================================================================
         * 2. 主状态机
         *
         *   STATE_HOMING    → 上电归零（homing_target 缓慢移向 0°）
         *   STATE_SCAN      → 哨兵扫描（正弦轨迹）
         *   STATE_RETURNING → 自瞄丢目标后回零，到位后切扫描
         *   STATE_TRACK     → 自瞄跟随（视觉绝对角度 + 限位保护）
         *
         *   切换规则（优先级从高到低）：
         *     1. 任意状态：detected 连续 TRACK_CONFIRM_FRAMES 帧且两轴已归位 → STATE_TRACK
         *     2. STATE_TRACK：detected=0 超过 LOST_TIMEOUT_MS → STATE_RETURNING
         *     3. STATE_RETURNING：两轴均到零点（< RETURN_THRESH）→ STATE_SCAN
         *     4. STATE_HOMING：两轴均到零点 → STATE_SCAN
         * ================================================================ */

        /* ---- 2-1. 归位阶段（STATE_HOMING）---- */
        if (state2 == STATE_HOMING)
        {
            /* ID2 yaw 轴：homing_target2 独立步进向 0° */
            float step2 = HOMING_SPEED * dt;
            float diff2 = angle_err_calc(0.0f, homing_target2);
            if      (diff2 >  step2) homing_target2 += step2;
            else if (diff2 < -step2) homing_target2 -= step2;
            else                     homing_target2  = 0.0f;

            if (fabsf(angle_err_calc(0.0f, dbg_angle2)) < HOMING_THRESH)
                state2 = STATE_SCAN;
        }

        if (state4 == STATE_HOMING)
        {
            /* ID4 pitch 轴：homing_target4 独立步进向 0°，与 ID2 完全解耦 */
            float step4 = HOMING_SPEED * dt;
            float diff4 = angle_err_calc(0.0f, homing_target4);
            if      (diff4 >  step4) homing_target4 += step4;
            else if (diff4 < -step4) homing_target4 -= step4;
            else                     homing_target4  = 0.0f;

            if (fabsf(angle_err_calc(0.0f, dbg_angle4)) < HOMING_THRESH)
                state4 = STATE_SCAN;
        }

        /* 归位未完成时跳过后续模式切换，防止提前进入自瞄 */
        int homing_done = (state2 != STATE_HOMING && state4 != STATE_HOMING);

        /* ---- 2-2. 视觉帧活性检测 -----------------------------------------
         *
         *  g_vision_rx.detected 收到帧后不会自动清零，视觉断连时仍保持 1。
         *  用 vis_rx_ok 计数器判断本周期是否有新帧到达：
         *    - 有新帧：vision_active = g_vision_rx.detected（尊重视觉判断）
         *    - 无新帧：vision_active = 0（强制认为丢失，触发超时回零）
         * ------------------------------------------------------------------ */
        static uint32_t last_vis_rx_ok = 0;
        uint8_t vision_active = 0;
        if (vis_rx_ok != last_vis_rx_ok)
        {
            /* 本周期收到了新帧，以视觉 detected 字段为准 */
            vision_active  = g_vision_rx.detected;
            last_vis_rx_ok = vis_rx_ok;
        }

        /* ---- 2-3. 收到视觉数据 → 更新目标角度并切/保持自瞄 ----
         *
         *  切入逻辑：vision_active=1 连续 TRACK_CONFIRM_FRAMES 帧后切入 STATE_TRACK，
         *            防止 detected 单帧抖动误触发。
         * --------------------------------------------------------------- */
        static int track_confirm_cnt = 0;

        if (homing_done && vision_active && !returning_lock)
        {
            track_confirm_cnt++;
            if (state2 != STATE_TRACK && track_confirm_cnt >= TRACK_CONFIRM_FRAMES)
            {
                /* 连续确认，切入自瞄
                 * 用当前编码器位置初始化低通滤波器，切入瞬间目标=当前位置，
                 * 低通以 VIS_LPF_ALPHA 速率平滑收敛到视觉角度，
                 * 避免目标跳变导致电机猛冲甩出视野
                 * 同时清前馈和 PID 积分 */
                vis_yaw_filtered   = dbg_angle2;
                vis_pitch_filtered = dbg_angle4;
                dbg_spd_ff         = 0.0f;
                angle_pid2.i_out   = 0.0f;
                angle_pid4.i_out   = 0.0f;
                speed_pid2.i_out   = 0.0f;
                speed_pid4.i_out   = 0.0f;
                state2 = STATE_TRACK;
                state4 = STATE_TRACK;
            }
        }
        else if (state2 != STATE_TRACK)
        {
            /* 非自瞄状态且无有效视觉帧：重置确认计数 */
            track_confirm_cnt = 0;
        }

        /* ---- STATE_TRACK 内部：目标更新与丢目标计时 ---- */
        if (state2 == STATE_TRACK)
        {
            if (vision_active)
            {
                /* 本周期收到新帧：先做单帧跳变检测，超过 VIS_JUMP_THRESH 认为是异常帧直接丢弃 */
                float vis_yaw_delta   = g_vision_rx.yaw_rad   - vis_yaw_filtered;
                float vis_pitch_delta = g_vision_rx.pitch_rad - vis_pitch_filtered;

                if (fabsf(vis_yaw_delta) > VIS_JUMP_THRESH || fabsf(vis_pitch_delta) > VIS_JUMP_THRESH)
                {
                    /* 跳变帧：目标保持不动，只重置丢目标计时，不更新滤波器 */
                    lost_timer_ms     = 0;
                    track_confirm_cnt = 0;
                }
                else
                {
                    /* 正常帧：经低通滤波后覆盖目标，重置丢目标计时和确认计数 */
                    lost_timer_ms     = 0;
                    track_confirm_cnt = 0;

                    vis_yaw_filtered   = VIS_LPF_ALPHA * g_vision_rx.yaw_rad
                                       + (1.0f - VIS_LPF_ALPHA) * vis_yaw_filtered;
                    vis_pitch_filtered = VIS_LPF_ALPHA * g_vision_rx.pitch_rad
                                       + (1.0f - VIS_LPF_ALPHA) * vis_pitch_filtered;

                    /* 减去视觉零点与编码器零点的偏移，对齐坐标系 */
                    track_target_yaw   = vis_yaw_filtered   - vis_yaw_offset;
                    track_target_pitch = vis_pitch_filtered - vis_pitch_offset;

                    if      (track_target_yaw   >  TRACK_LIMIT_YAW)   track_target_yaw   =  TRACK_LIMIT_YAW;
                    else if (track_target_yaw   < -TRACK_LIMIT_YAW)   track_target_yaw   = -TRACK_LIMIT_YAW;
                    if      (track_target_pitch >  TRACK_LIMIT_PITCH)  track_target_pitch =  TRACK_LIMIT_PITCH;
                    else if (track_target_pitch < -TRACK_LIMIT_PITCH)  track_target_pitch = -TRACK_LIMIT_PITCH;
                }
            }
            else
            {
                /* 本周期无新帧：目标保持不动，累加丢目标时间 */
                lost_timer_ms += 10;
                if (lost_timer_ms >= LOST_TIMEOUT_MS)
                {
                    /* 超时：从当前编码器位置出发平滑步进回零，清 PID 积分
                     * 置回零保护锁，防止回零途中视觉再次触发 STATE_TRACK */
                    returning_lock   = 1;
                    homing_target2   = dbg_angle2;
                    homing_target4   = dbg_angle4;
                    angle_pid2.i_out = 0.0f;
                    angle_pid4.i_out = 0.0f;
                    speed_pid2.i_out = 0.0f;
                    speed_pid4.i_out = 0.0f;
                    state2           = STATE_RETURNING;
                    state4           = STATE_RETURNING;
                    lost_timer_ms    = 0;
                }
            }
        }

        /* ---- 2-4. 回零阶段（STATE_RETURNING）：两轴各自步进向扫描零点 ----
         *
         *  homing_target2 / homing_target4 从自瞄末位置出发，
         *  每帧以 RETURN_SPEED（°/s）独立向 0 步进，互不干扰。
         *  到位条件：各自虚拟目标已到零 且 编码器误差 < RETURN_THRESH。
         *  两轴均满足才切扫描，保证同步完成后再开始正弦轨迹。
         * ------------------------------------------------------------------ */
        if (state2 == STATE_RETURNING)
        {
            float step2 = RETURN_SPEED * dt;
            if      (homing_target2 >  step2) homing_target2 -= step2;
            else if (homing_target2 < -step2) homing_target2 += step2;
            else                              homing_target2  = 0.0f;
        }

        if (state4 == STATE_RETURNING)
        {
            float step4 = RETURN_SPEED * dt;
            if      (homing_target4 >  step4) homing_target4 -= step4;
            else if (homing_target4 < -step4) homing_target4 += step4;
            else                              homing_target4  = 0.0f;
        }

        /* 两轴均完成回零才切扫描 */
        if (state2 == STATE_RETURNING || state4 == STATE_RETURNING)
        {
            int ret2_done = (state2 != STATE_RETURNING) ||
                            (fabsf(homing_target2) < 0.1f &&
                             fabsf(angle_err_calc(0.0f, dbg_angle2)) < RETURN_THRESH);
            int ret4_done = (state4 != STATE_RETURNING) ||
                            (fabsf(homing_target4) < 0.1f &&
                             fabsf(angle_err_calc(0.0f, dbg_angle4)) < RETURN_THRESH);

            if (ret2_done && ret4_done)
            {
                returning_lock = 0;   /* 解锁，允许视觉重新切入自瞄 */
                state2         = STATE_SCAN;
                state4         = STATE_SCAN;
                scan_phase     = 0.0f;   /* 相位归零，从 sin(0)=0 重新开始，无位置跳变 */
                prof_pos       = 0.0f;
                homing_target2 = 0.0f;
                homing_target4 = 0.0f;
            }
        }

        /* ---- 2-5. 扫描阶段：更新正弦轨迹 ---- */
        if (state2 == STATE_SCAN)
        {
            scan_phase     += dt;
            float omega     = 2.0f * (float)M_PI * SCAN_FREQ;
            prof_pos        = SCAN_AMP * sinf(omega * scan_phase);
            float ff_deg_s  = SCAN_AMP * omega * cosf(omega * scan_phase);
            dbg_spd_ff      = ff_deg_s * SPEED_FF_GAIN;
        }

        /* ---- 2-6. Ozone 调试变量（dbg_state：0=归位  1=扫描  2=回零  3=自瞄）---- */
        if      (state2 == STATE_HOMING)    dbg_state = 0;
        else if (state2 == STATE_SCAN)      dbg_state = 1;
        else if (state2 == STATE_RETURNING) dbg_state = 2;
        else                                dbg_state = 3;

        dbg_prof = prof_pos;

        /* ================================================================
         * 3. PID 计算
         *
         *   STATE_TRACK               目标 = 视觉绝对角度（已限位）
         *   STATE_HOMING/RETURNING    目标 = 各轴独立 homing_target（解耦归零）
         *   STATE_SCAN                目标 = prof_pos（正弦扫描轨迹）
         *
         *   ID2（yaw 轴）  ：target = -track_target_yaw / -homing_target2 / -prof_pos
         *   ID4（pitch 轴）：target = -track_target_pitch / -homing_target4 / -prof_pos
         *   负号含义与原始代码一致（电机安装方向取反）
         * ================================================================ */

        /* ---- ID2 PID（yaw 轴）---- */
        {
            float tgt2;
            if      (state2 == STATE_TRACK)   tgt2 = track_target_yaw;
            else if (state2 == STATE_SCAN)    tgt2 = -prof_pos;
            else /* HOMING / RETURNING */     tgt2 = -homing_target2;

            dbg_err2          = angle_err_calc(tgt2, dbg_angle2);
            angle_pid2.err[1] = angle_pid2.err[0];
            angle_pid2.err[0] = dbg_err2;
            angle_pid2.p_out  = angle_pid2.kp * angle_pid2.err[0];
            angle_pid2.i_out += angle_pid2.ki * angle_pid2.err[0];
            LIMIT_MIN_MAX(angle_pid2.i_out, -angle_pid2.i_max, angle_pid2.i_max);
            angle_pid2.d_out  = angle_pid2.kd * (angle_pid2.err[0] - angle_pid2.err[1]);
            float pid_spd2    = angle_pid2.p_out + angle_pid2.i_out + angle_pid2.d_out;

            /* 自瞄时不叠加扫描前馈；回零时单独限幅，防止全速冲向零点甩过 */
            float ff2         = (state2 == STATE_TRACK) ? 0.0f : dbg_spd_ff;
            float out_max2    = (state2 == STATE_RETURNING) ? RETURN_ANGLE_OUT_MAX : angle_pid2.out_max;
            float spd_target2 = pid_spd2 + ff2;
            LIMIT_MIN_MAX(spd_target2, -out_max2, out_max2);

            float spd_err2    = spd_target2 - filtered_spd2;
            speed_pid2.err[1] = speed_pid2.err[0];
            speed_pid2.err[0] = spd_err2;
            speed_pid2.p_out  = speed_pid2.kp * speed_pid2.err[0];
            speed_pid2.i_out += speed_pid2.ki * speed_pid2.err[0];
            LIMIT_MIN_MAX(speed_pid2.i_out, -speed_pid2.i_max, speed_pid2.i_max);
            speed_pid2.d_out  = speed_pid2.kd * (speed_pid2.err[0] - speed_pid2.err[1]);
            dbg_voltage2      = speed_pid2.p_out + speed_pid2.i_out + speed_pid2.d_out;
            LIMIT_MIN_MAX(dbg_voltage2, -speed_pid2.out_max, speed_pid2.out_max);
        }

        /* ---- ID4 PID（pitch 轴）---- */
        {
            float tgt4;
            if      (state4 == STATE_TRACK)   tgt4 = track_target_pitch;
            else if (state4 == STATE_SCAN)    tgt4 = -prof_pos;
            else /* HOMING / RETURNING */     tgt4 = -homing_target4;

            dbg_err4          = angle_err_calc(tgt4, dbg_angle4);
            angle_pid4.err[1] = angle_pid4.err[0];
            angle_pid4.err[0] = dbg_err4;
            angle_pid4.p_out  = angle_pid4.kp * angle_pid4.err[0];
            angle_pid4.i_out += angle_pid4.ki * angle_pid4.err[0];
            LIMIT_MIN_MAX(angle_pid4.i_out, -angle_pid4.i_max, angle_pid4.i_max);
            angle_pid4.d_out  = angle_pid4.kd * (angle_pid4.err[0] - angle_pid4.err[1]);
            float pid_spd4    = angle_pid4.p_out + angle_pid4.i_out + angle_pid4.d_out;

            /* 自瞄时不叠加扫描前馈；回零时单独限幅，防止全速冲向零点甩过 */
            float ff4         = (state4 == STATE_TRACK) ? 0.0f : dbg_spd_ff;
            float out_max4    = (state4 == STATE_RETURNING) ? RETURN_ANGLE_OUT_MAX : angle_pid4.out_max;
            float spd_target4 = pid_spd4 + ff4;
            LIMIT_MIN_MAX(spd_target4, -out_max4, out_max4);

            float spd_err4    = spd_target4 - filtered_spd4;
            speed_pid4.err[1] = speed_pid4.err[0];
            speed_pid4.err[0] = spd_err4;
            speed_pid4.p_out  = speed_pid4.kp * speed_pid4.err[0];
            speed_pid4.i_out += speed_pid4.ki * speed_pid4.err[0];
            LIMIT_MIN_MAX(speed_pid4.i_out, -speed_pid4.i_max, speed_pid4.i_max);
            speed_pid4.d_out  = speed_pid4.kd * (speed_pid4.err[0] - speed_pid4.err[1]);
            dbg_voltage4      = speed_pid4.p_out + speed_pid4.i_out + speed_pid4.d_out;
            LIMIT_MIN_MAX(dbg_voltage4, -speed_pid4.out_max, speed_pid4.out_max);
        }

        set_motor_voltage(0,
                          0,
                          (int16_t)dbg_voltage2,
                          0,
                          (int16_t)dbg_voltage4);

        /* ---------------------------------------------------------------- */
        /* 4. USB 上报当前状态帧（每 10ms 随控制周期发送）                   */
        /*    pitch_deg：IMU Mahony 解算 pitch（度）                         */
        /*    yaw_deg  ：编码器 yaw 角（度），即 dbg_yaw_enc                 */
        /*    mode     ：0x01=扫描/回零/归位  0x02=自瞄                     */
        /* ---------------------------------------------------------------- */
        {
            uint8_t tx_mode = (state2 == STATE_TRACK) ? 0x02 : 0x01;
            BSP_USB_Send(g_imu.pitch_m, dbg_yaw_enc, tx_mode);
        }

    } /* end for(;;) */
}
