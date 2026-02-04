/*
 * Copyright (c) 2023, Texas Instruments Incorporated - http://www.ti.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ============ ti_msp_dl_config.h =============
 *  Configured MSPM0 DriverLib module declarations
 *
 *  DO NOT EDIT - This file is generated for the MSPM0G350X
 *  by the SysConfig tool.
 */
#ifndef ti_msp_dl_config_h
#define ti_msp_dl_config_h

#define CONFIG_MSPM0G350X
#define CONFIG_MSPM0G3507

#if defined(__ti_version__) || defined(__TI_COMPILER_VERSION__)
#define SYSCONFIG_WEAK __attribute__((weak))
#elif defined(__IAR_SYSTEMS_ICC__)
#define SYSCONFIG_WEAK __weak
#elif defined(__GNUC__)
#define SYSCONFIG_WEAK __attribute__((weak))
#endif

#include <ti/devices/msp/msp.h>
#include <ti/driverlib/driverlib.h>
#include <ti/driverlib/m0p/dl_core.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 *  ======== SYSCFG_DL_init ========
 *  Perform all required MSP DL initialization
 *
 *  This function should be called once at a point before any use of
 *  MSP DL.
 */


/* clang-format off */

#define POWER_STARTUP_DELAY                                                (16)



#define CPUCLK_FREQ                                                     32000000



/* Defines for PWM_SERVO */
#define PWM_SERVO_INST                                                     TIMG8
#define PWM_SERVO_INST_IRQHandler                               TIMG8_IRQHandler
#define PWM_SERVO_INST_INT_IRQN                                 (TIMG8_INT_IRQn)
#define PWM_SERVO_INST_CLK_FREQ                                           100000
/* GPIO defines for channel 0 */
#define GPIO_PWM_SERVO_C0_PORT                                             GPIOA
#define GPIO_PWM_SERVO_C0_PIN                                     DL_GPIO_PIN_21
#define GPIO_PWM_SERVO_C0_IOMUX                                  (IOMUX_PINCM46)
#define GPIO_PWM_SERVO_C0_IOMUX_FUNC                 IOMUX_PINCM46_PF_TIMG8_CCP0
#define GPIO_PWM_SERVO_C0_IDX                                DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_PWM_SERVO_C1_PORT                                             GPIOB
#define GPIO_PWM_SERVO_C1_PIN                                     DL_GPIO_PIN_19
#define GPIO_PWM_SERVO_C1_IOMUX                                  (IOMUX_PINCM45)
#define GPIO_PWM_SERVO_C1_IOMUX_FUNC                 IOMUX_PINCM45_PF_TIMG8_CCP1
#define GPIO_PWM_SERVO_C1_IDX                                DL_TIMER_CC_1_INDEX

/* Defines for PWM_MOTOR */
#define PWM_MOTOR_INST                                                     TIMA1
#define PWM_MOTOR_INST_IRQHandler                               TIMA1_IRQHandler
#define PWM_MOTOR_INST_INT_IRQN                                 (TIMA1_INT_IRQn)
#define PWM_MOTOR_INST_CLK_FREQ                                          8000000
/* GPIO defines for channel 0 */
#define GPIO_PWM_MOTOR_C0_PORT                                             GPIOA
#define GPIO_PWM_MOTOR_C0_PIN                                     DL_GPIO_PIN_17
#define GPIO_PWM_MOTOR_C0_IOMUX                                  (IOMUX_PINCM39)
#define GPIO_PWM_MOTOR_C0_IOMUX_FUNC                 IOMUX_PINCM39_PF_TIMA1_CCP0
#define GPIO_PWM_MOTOR_C0_IDX                                DL_TIMER_CC_0_INDEX
/* GPIO defines for channel 1 */
#define GPIO_PWM_MOTOR_C1_PORT                                             GPIOA
#define GPIO_PWM_MOTOR_C1_PIN                                     DL_GPIO_PIN_16
#define GPIO_PWM_MOTOR_C1_IOMUX                                  (IOMUX_PINCM38)
#define GPIO_PWM_MOTOR_C1_IOMUX_FUNC                 IOMUX_PINCM38_PF_TIMA1_CCP1
#define GPIO_PWM_MOTOR_C1_IDX                                DL_TIMER_CC_1_INDEX



/* Defines for TIMER_0 */
#define TIMER_0_INST                                                     (TIMG0)
#define TIMER_0_INST_IRQHandler                                 TIMG0_IRQHandler
#define TIMER_0_INST_INT_IRQN                                   (TIMG0_INT_IRQn)
#define TIMER_0_INST_LOAD_VALUE                                         (39999U)




/* Defines for I2C_OLED */
#define I2C_OLED_INST                                                       I2C0
#define I2C_OLED_INST_IRQHandler                                 I2C0_IRQHandler
#define I2C_OLED_INST_INT_IRQN                                     I2C0_INT_IRQn
#define I2C_OLED_BUS_SPEED_HZ                                             400000
#define GPIO_I2C_OLED_SDA_PORT                                             GPIOA
#define GPIO_I2C_OLED_SDA_PIN                                      DL_GPIO_PIN_0
#define GPIO_I2C_OLED_IOMUX_SDA                                   (IOMUX_PINCM1)
#define GPIO_I2C_OLED_IOMUX_SDA_FUNC                    IOMUX_PINCM1_PF_I2C0_SDA
#define GPIO_I2C_OLED_SCL_PORT                                             GPIOA
#define GPIO_I2C_OLED_SCL_PIN                                      DL_GPIO_PIN_1
#define GPIO_I2C_OLED_IOMUX_SCL                                   (IOMUX_PINCM2)
#define GPIO_I2C_OLED_IOMUX_SCL_FUNC                    IOMUX_PINCM2_PF_I2C0_SCL


/* Defines for UART_0 */
#define UART_0_INST                                                        UART0
#define UART_0_INST_FREQUENCY                                            4000000
#define UART_0_INST_IRQHandler                                  UART0_IRQHandler
#define UART_0_INST_INT_IRQN                                      UART0_INT_IRQn
#define GPIO_UART_0_RX_PORT                                                GPIOA
#define GPIO_UART_0_TX_PORT                                                GPIOA
#define GPIO_UART_0_RX_PIN                                        DL_GPIO_PIN_11
#define GPIO_UART_0_TX_PIN                                        DL_GPIO_PIN_10
#define GPIO_UART_0_IOMUX_RX                                     (IOMUX_PINCM22)
#define GPIO_UART_0_IOMUX_TX                                     (IOMUX_PINCM21)
#define GPIO_UART_0_IOMUX_RX_FUNC                      IOMUX_PINCM22_PF_UART0_RX
#define GPIO_UART_0_IOMUX_TX_FUNC                      IOMUX_PINCM21_PF_UART0_TX
#define UART_0_BAUD_RATE                                                (115200)
#define UART_0_IBRD_4_MHZ_115200_BAUD                                        (2)
#define UART_0_FBRD_4_MHZ_115200_BAUD                                       (11)





/* Port definition for Pin Group LED1 */
#define LED1_PORT                                                        (GPIOB)

/* Defines for PIN_22: GPIOB.22 with pinCMx 50 on package pin 21 */
#define LED1_PIN_22_PIN                                         (DL_GPIO_PIN_22)
#define LED1_PIN_22_IOMUX                                        (IOMUX_PINCM50)
/* Port definition for Pin Group KEY */
#define KEY_PORT                                                         (GPIOB)

/* Defines for PIN_21: GPIOB.21 with pinCMx 49 on package pin 20 */
#define KEY_PIN_21_PIN                                          (DL_GPIO_PIN_21)
#define KEY_PIN_21_IOMUX                                         (IOMUX_PINCM49)
/* Port definition for Pin Group TB6612 */
#define TB6612_PORT                                                      (GPIOA)

/* Defines for AIN1: GPIOA.14 with pinCMx 36 on package pin 7 */
#define TB6612_AIN1_PIN                                         (DL_GPIO_PIN_14)
#define TB6612_AIN1_IOMUX                                        (IOMUX_PINCM36)
/* Defines for AIN2: GPIOA.15 with pinCMx 37 on package pin 8 */
#define TB6612_AIN2_PIN                                         (DL_GPIO_PIN_15)
#define TB6612_AIN2_IOMUX                                        (IOMUX_PINCM37)
/* Defines for BIN1: GPIOA.12 with pinCMx 34 on package pin 5 */
#define TB6612_BIN1_PIN                                         (DL_GPIO_PIN_12)
#define TB6612_BIN1_IOMUX                                        (IOMUX_PINCM34)
/* Defines for BIN2: GPIOA.13 with pinCMx 35 on package pin 6 */
#define TB6612_BIN2_PIN                                         (DL_GPIO_PIN_13)
#define TB6612_BIN2_IOMUX                                        (IOMUX_PINCM35)
/* Port definition for Pin Group Track */
#define Track_PORT                                                       (GPIOA)

/* Defines for DO: GPIOA.24 with pinCMx 54 on package pin 25 */
#define Track_DO_PIN                                            (DL_GPIO_PIN_24)
#define Track_DO_IOMUX                                           (IOMUX_PINCM54)
/* Defines for D1: GPIOA.25 with pinCMx 55 on package pin 26 */
#define Track_D1_PIN                                            (DL_GPIO_PIN_25)
#define Track_D1_IOMUX                                           (IOMUX_PINCM55)
/* Defines for D2: GPIOA.26 with pinCMx 59 on package pin 30 */
#define Track_D2_PIN                                            (DL_GPIO_PIN_26)
#define Track_D2_IOMUX                                           (IOMUX_PINCM59)
/* Defines for D3: GPIOA.27 with pinCMx 60 on package pin 31 */
#define Track_D3_PIN                                            (DL_GPIO_PIN_27)
#define Track_D3_IOMUX                                           (IOMUX_PINCM60)
/* Defines for D4: GPIOA.28 with pinCMx 3 on package pin 35 */
#define Track_D4_PIN                                            (DL_GPIO_PIN_28)
#define Track_D4_IOMUX                                            (IOMUX_PINCM3)
/* Defines for D5: GPIOA.29 with pinCMx 4 on package pin 36 */
#define Track_D5_PIN                                            (DL_GPIO_PIN_29)
#define Track_D5_IOMUX                                            (IOMUX_PINCM4)
/* Defines for D6: GPIOA.30 with pinCMx 5 on package pin 37 */
#define Track_D6_PIN                                            (DL_GPIO_PIN_30)
#define Track_D6_IOMUX                                            (IOMUX_PINCM5)
/* Defines for D7: GPIOA.31 with pinCMx 6 on package pin 39 */
#define Track_D7_PIN                                            (DL_GPIO_PIN_31)
#define Track_D7_IOMUX                                            (IOMUX_PINCM6)
/* Port definition for Pin Group ENCODERA */
#define ENCODERA_PORT                                                    (GPIOB)

/* Defines for E1A: GPIOB.13 with pinCMx 30 on package pin 1 */
// groups represented: ["ENCODERB","ENCODERA"]
// pins affected: ["E2A","E2B","E1A","E1B"]
#define GPIO_MULTIPLE_GPIOB_INT_IRQN                            (GPIOB_INT_IRQn)
#define GPIO_MULTIPLE_GPIOB_INT_IIDX            (DL_INTERRUPT_GROUP1_IIDX_GPIOB)
#define ENCODERA_E1A_IIDX                                   (DL_GPIO_IIDX_DIO13)
#define ENCODERA_E1A_PIN                                        (DL_GPIO_PIN_13)
#define ENCODERA_E1A_IOMUX                                       (IOMUX_PINCM30)
/* Defines for E1B: GPIOB.14 with pinCMx 31 on package pin 2 */
#define ENCODERA_E1B_IIDX                                   (DL_GPIO_IIDX_DIO14)
#define ENCODERA_E1B_PIN                                        (DL_GPIO_PIN_14)
#define ENCODERA_E1B_IOMUX                                       (IOMUX_PINCM31)
/* Port definition for Pin Group ENCODERB */
#define ENCODERB_PORT                                                    (GPIOB)

/* Defines for E2A: GPIOB.15 with pinCMx 32 on package pin 3 */
#define ENCODERB_E2A_IIDX                                   (DL_GPIO_IIDX_DIO15)
#define ENCODERB_E2A_PIN                                        (DL_GPIO_PIN_15)
#define ENCODERB_E2A_IOMUX                                       (IOMUX_PINCM32)
/* Defines for E2B: GPIOB.16 with pinCMx 33 on package pin 4 */
#define ENCODERB_E2B_IIDX                                   (DL_GPIO_IIDX_DIO16)
#define ENCODERB_E2B_PIN                                        (DL_GPIO_PIN_16)
#define ENCODERB_E2B_IOMUX                                       (IOMUX_PINCM33)

/* clang-format on */

void SYSCFG_DL_init(void);
void SYSCFG_DL_initPower(void);
void SYSCFG_DL_GPIO_init(void);
void SYSCFG_DL_SYSCTL_init(void);
void SYSCFG_DL_PWM_SERVO_init(void);
void SYSCFG_DL_PWM_MOTOR_init(void);
void SYSCFG_DL_TIMER_0_init(void);
void SYSCFG_DL_I2C_OLED_init(void);
void SYSCFG_DL_UART_0_init(void);


bool SYSCFG_DL_saveConfiguration(void);
bool SYSCFG_DL_restoreConfiguration(void);

#ifdef __cplusplus
}
#endif

#endif /* ti_msp_dl_config_h */
