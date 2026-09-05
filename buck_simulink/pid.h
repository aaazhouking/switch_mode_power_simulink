#ifndef PID_H
#define PID_H

#ifdef __cplusplus
extern "C" {
#endif

/* 复位 PID 状态（积分项、微分滤波、上一拍记忆） */
void pid_reset(void);

/* 设置 PID 参数：kp 比例，ki 积分 (1/s)，kd 微分 (s) */
void pid_set_tunings(double kp, double ki, double kd);

/* 设置输出限幅（默认 [0, 1]，即占空比 0~100%） */
void pid_set_limits(double out_min, double out_max);

/* 微分项一阶低通滤波时间常数 tau (s)，0 表示不过滤 */
void pid_set_dfilter(double tau);

/* 过流保护：i_limit < 0 关闭；iout 超限后按 i_kp 比例压占空比 */
void pid_set_overcurrent(double i_limit, double i_kp);

/* 离散 PID 单步计算（供 Simulink C Caller 块调用）
   vref : 输出电压给定 (V)
   vout : 采样输出电压 (V)
   iout : 采样输出电流 (A)
   dt   : 本拍步长 (s)，<= 0 时仅做比例控制
   返回 : 占空比 duty，已限幅 */
double pid_step(double vref, double vout, double iout, double dt);

/* 兼容旧接口（直通） */
double get_ir(double ir);
double get_vr(double vr);
double set_duty(double duty);
double set_period(double period);

#ifdef __cplusplus
}
#endif

#endif /* PID_H */
