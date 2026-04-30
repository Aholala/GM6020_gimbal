/* app_sentry_task.c
 * FreeRTOS SentryTask 实现
 * 更新：Pitch 独立扫描轨迹、速度/加速度前馈、阶跃测试模块、上电视觉暖机
 */

#include "app_sentry_task.h"
#include "app_sentry_globals.h"

void StartSentryTask(void *argument)
{
    const float dt = 0.01f;

    /* 阶跃测试边沿检测 */
    uint8_t step_trigger_last = 0;

    /* ---- 等待 IMU 校准完成 ---- */
    while (!imu_calib_done)
    {
        osDelay(10);
    }

    for (;;)
    {
        osDelay(10);  /* 100Hz */

        /* ---------------------------------------------------------------- */
        /* 1. 读取电机反馈                                                   */
        /* ---------------------------------------------------------------- */
        dbg_angle2  = enc_to_angle(motor_info[MOTOR2_IDX].rotor_angle, zero_enc2);
        dbg_angle4  = enc_to_angle(motor_info[MOTOR4_IDX].rotor_angle, zero_enc4);
        dbg_yaw_enc = dbg_angle2;

        float spd2 = (float)motor_info[MOTOR2_IDX].rotor_speed;
        float spd4 = (float)motor_info[MOTOR4_IDX].rotor_speed;
        filtered_spd2 = LPF_ALPHA * spd2 + (1.0f - LPF_ALPHA) * filtered_spd2;
        filtered_spd4 = LPF_ALPHA * spd4 + (1.0f - LPF_ALPHA) * filtered_spd4;

        /* ================================================================
         * 2. 主状态机
         * ================================================================ */

        /* ---- 2-1. 归位阶段 ---- */
        if (state2 == STATE_HOMING)
        {
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
            float step4 = HOMING_SPEED * dt;
            float diff4 = angle_err_calc(0.0f, homing_target4);
            if      (diff4 >  step4) homing_target4 += step4;
            else if (diff4 < -step4) homing_target4 -= step4;
            else                     homing_target4  = 0.0f;

            if (fabsf(angle_err_calc(0.0f, dbg_angle4)) < HOMING_THRESH)
                state4 = STATE_SCAN;
        }

        int homing_done = (state2 != STATE_HOMING && state4 != STATE_HOMING);

        /* ---- 2-2. 阶跃测试模式（step_test_en=1 时接管 SCAN 状态）---- */
        if (step_test_en && homing_done && state2 == STATE_SCAN)
        {
            /* 边沿检测：step_trigger 0→1 时触发一次阶跃 */
            if (step_trigger && !step_trigger_last)
            {
                /* 清 PID 积分，避免历史积分叠加在阶跃上 */
                angle_pid2.i_out = 0.0f;
                angle_pid4.i_out = 0.0f;
                speed_pid2.i_out = 0.0f;
                speed_pid4.i_out = 0.0f;

                /* clamp 防止超过限位 */
                float sy = step_target_yaw;
                float sp = step_target_pitch;
                if      (sy >  TRACK_LIMIT_YAW)   sy =  TRACK_LIMIT_YAW;
                else if (sy < -TRACK_LIMIT_YAW)   sy = -TRACK_LIMIT_YAW;
                if      (sp >  TRACK_LIMIT_PITCH)  sp =  TRACK_LIMIT_PITCH;
                else if (sp < -TRACK_LIMIT_PITCH)  sp = -TRACK_LIMIT_PITCH;

                dbg_step_ref2  = sy;
                dbg_step_ref4  = sp;
                dbg_step_t0_ms = HAL_GetTick();
                step_trigger   = 0;   /* 单次触发，自动清零 */
            }
            step_trigger_last = step_trigger;
            /* 阶跃模式下 prof_pos 保持不更新（扫描停止）*/
        }
        else
        {
            step_trigger_last = step_trigger;
        }

        /* ---- 2-3. 视觉帧活性检测 ---- */
        static uint32_t last_vis_rx_ok = 0;
        uint8_t vision_active = 0;
        if (vis_rx_ok != last_vis_rx_ok)
        {
            vision_active  = g_vision_rx.detected;
            last_vis_rx_ok = vis_rx_ok;
        }

        /* ---- 2-4. 上电暖机：丢弃前 VIS_WARMUP_FRAMES 帧 ---- */
        if (vision_active && dbg_vis_warmup_cnt < VIS_WARMUP_FRAMES)
        {
            dbg_vis_warmup_cnt++;
            vision_active = 0;  /* 暖机期间强制忽略视觉 */
        }

        /* ---- 2-5. 自瞄切入逻辑 ---- */
        static int track_confirm_cnt = 0;

        if (homing_done && vision_active && !returning_lock && !step_test_en)
        {
            track_confirm_cnt++;
            if (state2 != STATE_TRACK && track_confirm_cnt >= TRACK_CONFIRM_FRAMES)
            {
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
            track_confirm_cnt = 0;
        }

        /* ---- 2-6. STATE_TRACK 内部：目标更新与丢目标计时 ---- */
        if (state2 == STATE_TRACK)
        {
            if (vision_active)
            {
                float vis_yaw_delta   = g_vision_rx.yaw_rad   - vis_yaw_filtered;
                float vis_pitch_delta = g_vision_rx.pitch_rad - vis_pitch_filtered;

                if (fabsf(vis_yaw_delta) > VIS_JUMP_THRESH || fabsf(vis_pitch_delta) > VIS_JUMP_THRESH)
                {
                    /* 跳变帧丢弃 */
                    lost_timer_ms     = 0;
                    track_confirm_cnt = 0;
                }
                else
                {
                    lost_timer_ms     = 0;
                    track_confirm_cnt = 0;

                    vis_yaw_filtered   = VIS_LPF_ALPHA * g_vision_rx.yaw_rad
                                       + (1.0f - VIS_LPF_ALPHA) * vis_yaw_filtered;
                    vis_pitch_filtered = VIS_LPF_ALPHA * g_vision_rx.pitch_rad
                                       + (1.0f - VIS_LPF_ALPHA) * vis_pitch_filtered;

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
                lost_timer_ms += 10;
                if (lost_timer_ms >= LOST_TIMEOUT_MS)
                {
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

        /* ---- 2-7. 回零阶段 ---- */
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
                returning_lock  = 0;
                state2          = STATE_SCAN;
                state4          = STATE_SCAN;
                scan_phase      = 0.0f;
                scan_phase_pitch= 0.0f;
                prof_pos        = 0.0f;
                prof_pos_pitch  = 0.0f;
                homing_target2  = 0.0f;
                homing_target4  = 0.0f;
            }
        }

        /* ---- 2-8. 扫描阶段：Yaw/Pitch 独立轨迹 + 速度/加速度前馈 ---- */
        float spd_ff_yaw   = 0.0f;
        float spd_ff_pitch = 0.0f;
        float accel_ff_yaw   = 0.0f;
        float accel_ff_pitch = 0.0f;

        if (state2 == STATE_SCAN)
        {
            if (step_test_en)
            {
                /* 阶跃测试模式：扫描停止，用阶跃目标 */
                prof_pos       = 0.0f;
                prof_pos_pitch = 0.0f;
            }
            else
            {
                scan_phase       += dt;
                scan_phase_pitch += dt;

                float omega_yaw   = 2.0f * (float)M_PI * SCAN_FREQ;
                float omega_pitch = 2.0f * (float)M_PI * SCAN_FREQ_PITCH;

                prof_pos       = SCAN_AMP       * sinf(omega_yaw   * scan_phase);
                prof_pos_pitch = SCAN_AMP_PITCH * sinf(omega_pitch * scan_phase_pitch);

                /* 速度前馈：正弦位置目标的一阶导数 */
                spd_ff_yaw   = -SPEED_FF_GAIN * SCAN_AMP       * omega_yaw   * cosf(omega_yaw   * scan_phase);
                spd_ff_pitch = -SPEED_FF_GAIN * SCAN_AMP_PITCH * omega_pitch * cosf(omega_pitch * scan_phase_pitch);

                /* 加速度前馈：速度前馈的一阶导数 */
                accel_ff_yaw   = -SPEED_FF_GAIN * SCAN_AMP       * omega_yaw   * omega_yaw   * sinf(omega_yaw   * scan_phase);
                accel_ff_pitch = -SPEED_FF_GAIN * SCAN_AMP_PITCH * omega_pitch * omega_pitch * sinf(omega_pitch * scan_phase_pitch);

                dbg_spd_ff = spd_ff_yaw;
            }
        }

        /* ---- 2-9. Ozone 调试变量 ---- */
        if      (state2 == STATE_HOMING)    dbg_state = 0;
        else if (state2 == STATE_SCAN)      dbg_state = 1;
        else if (state2 == STATE_RETURNING) dbg_state = 2;
        else                                dbg_state = 3;

        dbg_prof       = -prof_pos;
        dbg_prof_pitch =  prof_pos_pitch;

        /* ================================================================
         * 3. PID 计算
         * ================================================================ */

        /* ---- ID2 PID（Yaw 轴）---- */
        {
            float tgt2;
            if (state2 == STATE_TRACK)
            {
                tgt2 = track_target_yaw;
            }
            else if (state2 == STATE_SCAN)
            {
                tgt2 = step_test_en ? dbg_step_ref2 : -prof_pos;
            }
            else /* HOMING / RETURNING */
            {
                tgt2 = -homing_target2;
            }

            dbg_err2          = angle_err_calc(tgt2, dbg_angle2);
            angle_pid2.err[1] = angle_pid2.err[0];
            angle_pid2.err[0] = dbg_err2;
            angle_pid2.p_out  = angle_pid2.kp * angle_pid2.err[0];
            angle_pid2.i_out += angle_pid2.ki * angle_pid2.err[0];
            LIMIT_MIN_MAX(angle_pid2.i_out, -angle_pid2.i_max, angle_pid2.i_max);
            angle_pid2.d_out  = angle_pid2.kd * (angle_pid2.err[0] - angle_pid2.err[1]);
            float pid_spd2    = angle_pid2.p_out + angle_pid2.i_out + angle_pid2.d_out;

            float out_max2    = (state2 == STATE_RETURNING) ? RETURN_ANGLE_OUT_MAX : angle_pid2.out_max;
            float spd_target2 = pid_spd2;
            LIMIT_MIN_MAX(spd_target2, -out_max2, out_max2);

            float spd_err2    = spd_target2 - filtered_spd2;
            speed_pid2.err[1] = speed_pid2.err[0];
            speed_pid2.err[0] = spd_err2;
            speed_pid2.p_out  = speed_pid2.kp * speed_pid2.err[0];
            speed_pid2.i_out += speed_pid2.ki * speed_pid2.err[0];
            LIMIT_MIN_MAX(speed_pid2.i_out, -speed_pid2.i_max, speed_pid2.i_max);
            speed_pid2.d_out  = speed_pid2.kd * (speed_pid2.err[0] - speed_pid2.err[1]);
            dbg_voltage2      = speed_pid2.p_out + speed_pid2.i_out + speed_pid2.d_out;

            /* 速度/加速度前馈直接叠加到电压，绕过速度环 KP 放大 */
            if (state2 == STATE_SCAN && !step_test_en)
            {
                dbg_voltage2 += spd_ff_yaw   * SPEED_FF_VOLTAGE;
                dbg_voltage2 += accel_ff_yaw * ACCEL_FF_VOLTAGE;
            }

            LIMIT_MIN_MAX(dbg_voltage2, -speed_pid2.out_max, speed_pid2.out_max);
        }

        /* ---- ID4 PID（Pitch 轴）---- */
        {
            float tgt4;
            if (state4 == STATE_TRACK)
            {
                tgt4 = track_target_pitch;
            }
            else if (state4 == STATE_SCAN)
            {
                tgt4 = step_test_en ? dbg_step_ref4 : prof_pos_pitch;
            }
            else /* HOMING / RETURNING */
            {
                tgt4 = -homing_target4;
            }

            dbg_err4          = angle_err_calc(tgt4, dbg_angle4);
            angle_pid4.err[1] = angle_pid4.err[0];
            angle_pid4.err[0] = dbg_err4;
            angle_pid4.p_out  = angle_pid4.kp * angle_pid4.err[0];
            angle_pid4.i_out += angle_pid4.ki * angle_pid4.err[0];
            LIMIT_MIN_MAX(angle_pid4.i_out, -angle_pid4.i_max, angle_pid4.i_max);
            angle_pid4.d_out  = angle_pid4.kd * (angle_pid4.err[0] - angle_pid4.err[1]);
            float pid_spd4    = angle_pid4.p_out + angle_pid4.i_out + angle_pid4.d_out;

            float out_max4    = (state4 == STATE_RETURNING) ? RETURN_ANGLE_OUT_MAX : angle_pid4.out_max;
            float spd_target4 = pid_spd4;
            LIMIT_MIN_MAX(spd_target4, -out_max4, out_max4);

            float spd_err4    = spd_target4 - filtered_spd4;
            speed_pid4.err[1] = speed_pid4.err[0];
            speed_pid4.err[0] = spd_err4;
            speed_pid4.p_out  = speed_pid4.kp * speed_pid4.err[0];
            speed_pid4.i_out += speed_pid4.ki * speed_pid4.err[0];
            LIMIT_MIN_MAX(speed_pid4.i_out, -speed_pid4.i_max, speed_pid4.i_max);
            speed_pid4.d_out  = speed_pid4.kd * (speed_pid4.err[0] - speed_pid4.err[1]);
            dbg_voltage4      = speed_pid4.p_out + speed_pid4.i_out + speed_pid4.d_out;

            /* 速度/加速度前馈直接叠加到电压 */
            if (state4 == STATE_SCAN && !step_test_en)
            {
                dbg_voltage4 += spd_ff_pitch   * SPEED_FF_VOLTAGE;
                dbg_voltage4 += accel_ff_pitch * ACCEL_FF_VOLTAGE;
            }

            LIMIT_MIN_MAX(dbg_voltage4, -speed_pid4.out_max, speed_pid4.out_max);
        }

        set_motor_voltage(0,
                          0,
                          (int16_t)dbg_voltage2,
                          0,
                          (int16_t)dbg_voltage4);

        /* ---------------------------------------------------------------- */
        /* 4. USB 上报                                                       */
        /* ---------------------------------------------------------------- */
        {
            uint8_t tx_mode = (state2 == STATE_TRACK) ? 0x02 : 0x01;
            BSP_USB_Send(g_imu.pitch_m, dbg_yaw_enc, tx_mode);
        }

    } /* end for(;;) */
}