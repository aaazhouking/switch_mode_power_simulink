#include "pid.h"

/* ================= PID 参数 ================= */
static double s_kp      = 0.10;   /* 比例增益 */
static double s_ki      = 1.00;   /* 积分增益 (1/s) */
static double s_kd      = 0.00;   /* 微分增益 (s) */
static double s_out_min = 0.00;   /* 输出下限（占空比） */
static double s_out_max = 1.00;   /* 输出上限（占空比） */
static double s_d_tau   = 0.00;   /* 微分一阶滤波时间常数 (s)，0 = 不过滤 */

/* 过流保护：iout > s_i_limit 时按比例压占空比；s_i_limit < 0 关闭 */
static double s_i_limit = -1.0;
static double s_i_kp    = 0.0;

/* ================= PID 内部状态 ================= */
static double s_integ     = 0.0;  /* 积分项 */
static double s_prev_vout = 0.0;  /* 上一拍输出电压（用于微分） */
static double s_prev_d    = 0.0;  /* 滤波后的微分项 */
static int    s_first     = 1;    /* 首次调用标志 */

void pid_reset(void)
{
    s_integ     = 0.0;
    s_prev_vout = 0.0;
    s_prev_d    = 0.0;
    s_first     = 1;
}

void pid_set_tunings(double kp, double ki, double kd)
{
    s_kp = kp;
    s_ki = ki;
    s_kd = kd;
}

void pid_set_limits(double out_min, double out_max)
{
    if (out_min < out_max) {
        s_out_min = out_min;
        s_out_max = out_max;
    }
}

void pid_set_dfilter(double tau)
{
    s_d_tau = (tau > 0.0) ? tau : 0.0;
}

void pid_set_overcurrent(double i_limit, double i_kp)
{
    s_i_limit = i_limit;
    s_i_kp    = i_kp;
}

double pid_step(double vref, double vout, double iout, double dt)
{
    double err = vref - vout;
    double u;

    if (s_first || dt <= 0.0) {
        /* 首拍或步长无效：仅比例输出，避免除零与初始冲击 */
        s_integ     = 0.0;
        s_prev_vout = vout;
        s_prev_d    = 0.0;
        s_first     = 0;
        u = s_kp * err;
    } else {
        double d_raw;

        /* 微分用测量值 -kd*dV/dt，避免给定突变引起微分冲击 */
        d_raw = -s_kd * (vout - s_prev_vout) / dt;

        /* 微分项一阶低通滤波 */
        if (s_d_tau > 0.0) {
            double alpha = dt / (dt + s_d_tau);
            s_prev_d += alpha * (d_raw - s_prev_d);
        } else {
            s_prev_d = d_raw;
        }

        u = s_kp * err + s_integ + s_prev_d;

        /* 反算抗积分饱和：先限幅，若饱和则把积分项拉回 */
        {
            double u_lim = u;
            if (u_lim > s_out_max) u_lim = s_out_max;
            if (u_lim < s_out_min) u_lim = s_out_min;
            if (u_lim != u) {
                s_integ = u_lim - s_kp * err - s_prev_d;
            }
        }

        s_integ += s_ki * err * dt;

        s_prev_vout = vout;
    }

    /* 输出限幅 */
    if (u > s_out_max) u = s_out_max;
    if (u < s_out_min) u = s_out_min;

    /* 过流保护 */
    if (s_i_limit > 0.0 && iout > s_i_limit) {
        u -= s_i_kp * (iout - s_i_limit);
        if (u < s_out_min) u = s_out_min;
    }

    return u;
}

/* ================= 兼容旧接口（直通） ================= */
double get_ir(double ir) { return ir; }
double get_vr(double vr) { return vr; }
double set_duty(double duty) { return duty; }
double set_period(double period) { return period; }

