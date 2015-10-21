#ifndef _PWM_OUT_H_
#define _PWM_OUT_H_

#define TIMER_CLK_DIV   (8)

#define PWM_MIN_FREQ    (50)
#define PWM_MAX_FREQ    (30000)

typedef enum {
    PWM_OUT_0,
    PWM_OUT_1
}   PWM_OUT_CH;

bool pwm_out_init(void);
bool pwm_out_start(PWM_OUT_CH ch, uint32_t clock, float duty);
bool pwm_out_stop(PWM_OUT_CH ch);

#endif
