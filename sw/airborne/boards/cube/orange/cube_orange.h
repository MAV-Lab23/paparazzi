#ifndef CONFIG_CUBE_ORANGE_H
#define CONFIG_CUBE_ORANGE_H

#define BOARD_CUBE_ORANGE

/**
 * ChibiOS board file
 */
#include "board.h"

/**
 * PPRZ definitions
 */

/*
 * AHB_CLK
 */
#define AHB_CLK STM32_HCLK

/*
 * Concat macro
 */
#define _CONCAT_BOARD_PARAM(_s1, _s2) _s1 ## _s2
#define CONCAT_BOARD_PARAM(_s1, _s2) _CONCAT_BOARD_PARAM(_s1, _s2)

/*
 * PWM defines
 */

#ifndef USE_PWM1
#define USE_PWM1 1
#endif
#if USE_PWM1
#define PWM_SERVO_1 1
#define PWM_SERVO_1_GPIO PAL_PORT(LINE_SERVO1)
#define PWM_SERVO_1_PIN PAL_PAD(LINE_SERVO1)
#define PWM_SERVO_1_AF AF_LINE_SERVO1
#define PWM_SERVO_1_DRIVER CONCAT_BOARD_PARAM(PWMD, SERVO1_TIM)
#define PWM_SERVO_1_CHANNEL (SERVO1_TIM_CH-1)
#define PWM_SERVO_1_CONF CONCAT_BOARD_PARAM(pwmcfg, SERVO1_TIM)
#endif

#ifndef USE_PWM2
#define USE_PWM2 1
#endif
#if USE_PWM2
#define PWM_SERVO_2 2
#define PWM_SERVO_2_GPIO PAL_PORT(LINE_SERVO2)
#define PWM_SERVO_2_PIN PAL_PAD(LINE_SERVO2)
#define PWM_SERVO_2_AF AF_LINE_SERVO2
#define PWM_SERVO_2_DRIVER CONCAT_BOARD_PARAM(PWMD, SERVO2_TIM)
#define PWM_SERVO_2_CHANNEL (SERVO2_TIM_CH-1)
#define PWM_SERVO_2_CONF CONCAT_BOARD_PARAM(pwmcfg, SERVO2_TIM)
#endif

#ifndef USE_PWM3
#define USE_PWM3 1
#endif
#if USE_PWM3
#define PWM_SERVO_3 3
#define PWM_SERVO_3_GPIO PAL_PORT(LINE_SERVO3)
#define PWM_SERVO_3_PIN PAL_PAD(LINE_SERVO3)
#define PWM_SERVO_3_AF AF_LINE_SERVO3
#define PWM_SERVO_3_DRIVER CONCAT_BOARD_PARAM(PWMD, SERVO3_TIM)
#define PWM_SERVO_3_CHANNEL (SERVO3_TIM_CH-1)
#define PWM_SERVO_3_CONF CONCAT_BOARD_PARAM(pwmcfg, SERVO3_TIM)
#endif

#ifndef USE_PWM4
#define USE_PWM4 1
#endif
#if USE_PWM4
#define PWM_SERVO_4 4
#define PWM_SERVO_4_GPIO PAL_PORT(LINE_SERVO4)
#define PWM_SERVO_4_PIN PAL_PAD(LINE_SERVO4)
#define PWM_SERVO_4_AF AF_LINE_SERVO4
#define PWM_SERVO_4_DRIVER CONCAT_BOARD_PARAM(PWMD, SERVO4_TIM)
#define PWM_SERVO_4_CHANNEL (SERVO4_TIM_CH-1)
#define PWM_SERVO_4_CONF CONCAT_BOARD_PARAM(pwmcfg, SERVO4_TIM)
#endif

#ifndef USE_PWM5
#define USE_PWM5 1
#endif
#if USE_PWM5
#define PWM_SERVO_5 5
#define PWM_SERVO_5_GPIO PAL_PORT(LINE_SERVO5)
#define PWM_SERVO_5_PIN PAL_PAD(LINE_SERVO5)
#define PWM_SERVO_5_AF AF_LINE_SERVO5
#define PWM_SERVO_5_DRIVER CONCAT_BOARD_PARAM(PWMD, SERVO5_TIM)
#define PWM_SERVO_5_CHANNEL (SERVO5_TIM_CH-1)
#define PWM_SERVO_5_CONF CONCAT_BOARD_PARAM(pwmcfg, SERVO5_TIM)
#endif

// servo index starting at 1 + regular servos
// so NB = 1+5
#define ACTUATORS_PWM_NB 6

/*
 * Actuators for fixedwing
 */
 /* Default actuators driver */
#define DEFAULT_ACTUATORS "modules/actuators/actuators_pwm.h"
#define ActuatorDefaultSet(_x,_y) ActuatorPwmSet(_x,_y)
#define ActuatorsDefaultInit() ActuatorsPwmInit()
#define ActuatorsDefaultCommit() ActuatorsPwmCommit()

#endif /* CONFIG_CUBE_ORANGE_H */

