/*
	Copyright 2018 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */


#ifndef HW_PDESC_CORE_H_
#define HW_PDESC_CORE_H_

#define HW_NAME					"PDESC_NHWL"
//#define HW_PROTECTION
#define DISABLE_HW_LIMITS

#define HW_MAJOR				1
#define HW_MINOR				0


// HW properties
#define HW_HAS_3_SHUNTS
#define HW_HAS_PHASE_SHUNTS
#define LED_GREEN_GPIO			GPIOB
#define LED_GREEN_PIN			2
#define LED_RED_GPIO			GPIOB
#define LED_RED_PIN				3

#define LED_GREEN_ON()			palSetPad(LED_GREEN_GPIO, LED_GREEN_PIN)
#define LED_GREEN_OFF()			palClearPad(LED_GREEN_GPIO, LED_GREEN_PIN)
#define LED_RED_ON()			palSetPad(LED_RED_GPIO, LED_RED_PIN)
#define LED_RED_OFF()			palClearPad(LED_RED_GPIO, LED_RED_PIN)

#define AUX_GPIO				GPIOB
#define AUX_PIN					4
#define AUX_ON()				palClearPad(AUX_GPIO, AUX_PIN)
#define AUX_OFF()				palSetPad(AUX_GPIO, AUX_PIN)

#define AUX2_GPIO				GPIOB
#define AUX2_PIN					12
#define AUX2_ON()				palClearPad(AUX2_GPIO, AUX2_PIN)
#define AUX2_OFF()				palSetPad(AUX2_GPIO, AUX2_PIN)



#define ENABLE_GATE()			palClearPad(GPIOB, 5)
#define DISABLE_GATE()			palSetPad(GPIOB, 5)

#ifdef HW_PROTECTION
#define IS_DRV_FAULT()			(palReadPad(GPIOB, 7))
#define PALTA_OC_CLR_PORT		GPIOC
#define PALTA_OC_CLR_PIN		12
#define HW_RESET_DRV_FAULTS()   hw_palta_reset_oc()
#endif

/*
 * ADC Vector
 *
 * 0  (1):	IN0		SENS1
 * 1  (2):	IN1		SENS2
 * 2  (3):	IN2		SENS3
 * 3  (1):	IN10	CURR1
 * 4  (2):	IN11	CURR2
 * 5  (3):	IN12	CURR3
 * 6  (1):	IN5		ADC_EXT1
 * 7  (2):	IN6		ADC_EXT2
 * 8  (3):	IN3		TEMP_MOS
 * 9  (1):	IN14	TEMP_MOTOR
 * 10 (2):	IN15	ADC_EXT3
 * 11 (3):	IN13	AN_IN
 * 12 (1):	Vrefint
 * 13 (2):	IN0		SENS1
 * 14 (3):	IN1		SENS2
 * 15 (1):  IN8		TEMP_MOS_2
 * 16 (2):  IN9		TEMP_MOS_3
 * 17 (3):  IN3		SENS3
 */

#define HW_ADC_CHANNELS			18
#define HW_ADC_INJ_CHANNELS		3
#define HW_ADC_NBR_CONV			6

// ADC Indexes
#define ADC_IND_SENS1			3
#define ADC_IND_SENS2			4
#define ADC_IND_SENS3			5
#define ADC_IND_CURR1			0
#define ADC_IND_CURR2			1
#define ADC_IND_CURR3			2
#define ADC_IND_VIN_SENS		11
#define ADC_IND_EXT				6
#define ADC_IND_EXT2			7
#define ADC_IND_EXT3			10
#define ADC_IND_TEMP_MOS		8
#define ADC_IND_TEMP_MOS_2		15
#define ADC_IND_TEMP_MOS_3		16
#define ADC_IND_TEMP_MOTOR		9
#define ADC_IND_VREFINT			12

// ADC macros and settings

// Component parameters (can be overridden)
#ifndef V_REG
#define V_REG					3.3
#endif
#ifndef VIN_R1
#define VIN_R1					150000.0
#endif
#ifndef VIN_R2
#define VIN_R2					3300.0
#endif
#ifndef CURRENT_AMP_GAIN
//#define CURRENT_AMP_GAIN		0.001443 //0.0075 gap
#define CURRENT_AMP_GAIN		0.001204 //0.009 gap
#endif
#ifndef CURRENT_SHUNT_RES
#define CURRENT_SHUNT_RES		1.0
#endif

// Input voltage
#define GET_INPUT_VOLTAGE()		((V_REG / 4095.0) * (float)ADC_Value[ADC_IND_VIN_SENS] * ((VIN_R1 + VIN_R2) / VIN_R2))

// NTC Termistors
//#define NTC_RES(adc_val)		((4095.0 * 10000.0) / adc_val - 10000.0)

#define NTC_RES(adc_val)  (10000.0 / ((4095.0 / (float)adc_val) - 1.0))
#define NTC_TEMP(adc_ind)		hw75_300_get_temp()

#define NTC_RES_MOTOR(adc_val)	(10000.0 / ((4095.0 / (float)adc_val) - 1.0)) // Motor temp sensor on low side
#define NTC_TEMP_MOTOR(beta)	(1.0 / ((logf(NTC_RES_MOTOR(ADC_Value[ADC_IND_TEMP_MOTOR]) / 10000.0) / beta) + (1.0 / 298.15)) - 273.15)

#define NTC_TEMP_MOS1()			(1.0 / ((logf(NTC_RES(ADC_Value[ADC_IND_TEMP_MOS]) / 10000.0) / 3380.0) + (1.0 / 298.15)) - 273.15)
#define NTC_TEMP_MOS2()			(1.0 / ((logf(NTC_RES(ADC_Value[ADC_IND_TEMP_MOS_2]) / 10000.0) / 3380.0) + (1.0 / 298.15)) - 273.15)
#define NTC_TEMP_MOS3()			(1.0 / ((logf(NTC_RES(ADC_Value[ADC_IND_TEMP_MOS_3]) / 10000.0) / 3380.0) + (1.0 / 298.15)) - 273.15)


// Voltage on ADC channel
#define ADC_VOLTS(ch)			((float)ADC_Value[ch] / 4096.0 * V_REG)

// Double samples in beginning and end for positive current measurement.
// Useful when the shunt sense traces have noise that causes offset.
#ifndef CURR1_DOUBLE_SAMPLE
#define CURR1_DOUBLE_SAMPLE		0
#endif
#ifndef CURR2_DOUBLE_SAMPLE
#define CURR2_DOUBLE_SAMPLE		0
#endif
#ifndef CURR3_DOUBLE_SAMPLE
#define CURR3_DOUBLE_SAMPLE		0
#endif

// COMM-port ADC GPIOs
#define HW_ADC_EXT_GPIO			GPIOA
#define HW_ADC_EXT_PIN			5
#define HW_ADC_EXT2_GPIO		GPIOA
#define HW_ADC_EXT2_PIN			6

// UART Peripheral
#define HW_UART_DEV				SD3
#define HW_UART_GPIO_AF			GPIO_AF_USART3
#define HW_UART_TX_PORT			GPIOB
#define HW_UART_TX_PIN			10
#define HW_UART_RX_PORT			GPIOB
#define HW_UART_RX_PIN			11

// Permanent UART Peripheral (for NRF51)
#define HW_UART_P_BAUD			115200
#define HW_UART_P_DEV			SD4
#define HW_UART_P_GPIO_AF		GPIO_AF_UART4
#define HW_UART_P_TX_PORT		GPIOC
#define HW_UART_P_TX_PIN		10
#define HW_UART_P_RX_PORT		GPIOC
#define HW_UART_P_RX_PIN		11
#endif

/* // NRF SWD
#define NRF5x_SWDIO_GPIO		GPIOA
#define NRF5x_SWDIO_PIN			15
#define NRF5x_SWCLK_GPIO		GPIOB
#define NRF5x_SWCLK_PIN			3
#endif
*/

// ICU Peripheral for servo decoding
#define HW_USE_SERVO_TIM4
#define HW_ICU_TIMER			TIM4
#define HW_ICU_TIM_CLK_EN()		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE)
#define HW_ICU_DEV				ICUD4
#define HW_ICU_CHANNEL			ICU_CHANNEL_1
#define HW_ICU_GPIO_AF			GPIO_AF_TIM4
#define HW_ICU_GPIO				GPIOB
#define HW_ICU_PIN				6

// I2C Peripheral
#define HW_I2C_DEV				I2CD2
#define HW_I2C_GPIO_AF			GPIO_AF_I2C2
#define HW_I2C_SCL_PORT			GPIOB
#define HW_I2C_SCL_PIN			10
#define HW_I2C_SDA_PORT			GPIOB
#define HW_I2C_SDA_PIN			11

// Hall/encoder pins
#define HW_HALL_ENC_GPIO1		GPIOC
#define HW_HALL_ENC_PIN1		6
#define HW_HALL_ENC_GPIO2		GPIOC
#define HW_HALL_ENC_PIN2		7
#define HW_HALL_ENC_GPIO3		GPIOC
#define HW_HALL_ENC_PIN3		8
#define HW_ENC_TIM				TIM3
#define HW_ENC_TIM_AF			GPIO_AF_TIM3
#define HW_ENC_TIM_CLK_EN()		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE)
#define HW_ENC_EXTI_PORTSRC		EXTI_PortSourceGPIOC
#define HW_ENC_EXTI_PINSRC		EXTI_PinSource8
#define HW_ENC_EXTI_CH			EXTI9_5_IRQn
#define HW_ENC_EXTI_LINE		EXTI_Line8
#define HW_ENC_EXTI_ISR_VEC		EXTI9_5_IRQHandler
#define HW_ENC_TIM_ISR_CH		TIM3_IRQn
#define HW_ENC_TIM_ISR_VEC		TIM3_IRQHandler

// SPI pins
#define HW_SPI_DEV				SPID1
#define HW_SPI_GPIO_AF			GPIO_AF_SPI1
#define HW_SPI_PORT_NSS			GPIOA
#define HW_SPI_PIN_NSS			4
#define HW_SPI_PORT_SCK			GPIOA
#define HW_SPI_PIN_SCK			5
#define HW_SPI_PORT_MOSI		GPIOA
#define HW_SPI_PIN_MOSI			7
#define HW_SPI_PORT_MISO		GPIOA
#define HW_SPI_PIN_MISO			6

// Measurement macros
#define ADC_V_L1				ADC_Value[ADC_IND_SENS1]
#define ADC_V_L2				ADC_Value[ADC_IND_SENS2]
#define ADC_V_L3				ADC_Value[ADC_IND_SENS3]
#define ADC_V_ZERO				(ADC_Value[ADC_IND_VIN_SENS] / 2)

// Macros
#define READ_HALL1()			palReadPad(HW_HALL_ENC_GPIO1, HW_HALL_ENC_PIN1)
#define READ_HALL2()			palReadPad(HW_HALL_ENC_GPIO2, HW_HALL_ENC_PIN2)
#define READ_HALL3()			palReadPad(HW_HALL_ENC_GPIO3, HW_HALL_ENC_PIN3)

// Override dead time. See the stm32f4 reference manual for calculating this value.
#define HW_DEAD_TIME_NSEC		800.0

// Default setting overrides
#define MCCONF_L_MIN_VOLTAGE			40.0		// Minimum input voltage
#define MCCONF_L_MAX_VOLTAGE			125.0	// Maximum input voltage
#define MCCONF_DEFAULT_MOTOR_TYPE		MOTOR_TYPE_FOC
#define MCCONF_FOC_F_ZV					16000.0
#define MCCONF_L_MAX_ABS_CURRENT		350.0	// The maximum absolute current above which a fault is generated
#define MCCONF_L_IN_CURRENT_MAX			250.0	// Input current limit in Amperes (Upper)
#define MCCONF_L_IN_CURRENT_MIN			-0.0	// Input current limit in Amperes (Lower)
//#define MCCONF_FOC_SAMPLE_V0_V7			false	// Run control loop in both v0 and v7 (requires phase shunts)

#define MCCONF_L_MIN_DUTY 0.005 // Minimum Duty Cycle
#define MCCONF_L_MAX_DUTY 0.95 // Maximum Duty Cycle

#define MCCONF_FOC_DT_US 0.2


// Setting limits
#define HW_LIM_CURRENT			-950.0, 950.0
#define HW_LIM_CURRENT_IN		-250.0, 500.0
#define HW_LIM_CURRENT_ABS		0.0, 1100.0
#define HW_LIM_VIN				39.0, 125.0
#define HW_LIM_ERPM				-200e3, 200e3
#define HW_LIM_DUTY_MIN			0.0, 0.1
#define HW_LIM_DUTY_MAX			0.0, 0.95
#define HW_LIM_TEMP_FET			-40.0, 90.0
#define HW_LIM_FOC_CTRL_LOOP_FREQ	3000.0, 30000.0


#define MCCONF_FOC_START_CURR_DEC_RPM	2500.0	// At this RPM the full current is available. Default config

//#define MCCONF_FOC_OBSERVER_TYPE		FOC_OBSERVER_MXLEMMING_LAMBDA_COMP // Position observer type for FOC. Default config

// Observer Type
#define MCCONF_FOC_OBSERVER_TYPE 0

// Control Sample Mode
#define MCCONF_FOC_CONTROL_SAMPLE_MODE 1

// Current Sample Mode
#define MCCONF_FOC_CURRENT_SAMPLE_MODE 0

#define MCCONF_M_ENCODER_COUNTS			1024	// The number of encoder counts


#define MCCONF_FOC_SL_ERPM_HFI 300000

// Sensorless ERPM
#define MCCONF_FOC_SL_ERPM 350000

// D Axis Gain Scaling Start
#define MCCONF_FOC_D_GAIN_SCALE_START 0.6

// D Axis Gain Scaling at Max Mod
#define MCCONF_FOC_D_GAIN_SCALE_MAX_MOD 0.4

// MOSFET Temp Cutoff Start
#define MCCONF_L_LIM_TEMP_FET_START 65

// MOSFET Temp Cutoff End
#define MCCONF_L_LIM_TEMP_FET_END 75

// Motor Temp Cutoff Start
#define MCCONF_L_LIM_TEMP_MOTOR_START 85

// Motor Temp Cutoff End
#define MCCONF_L_LIM_TEMP_MOTOR_END 100

// Temp Comp
#define MCCONF_FOC_TEMP_COMP 1

// Enable Phase Filters
#define MCCONF_FOC_PHASE_FILTER_ENABLE 0

// HW-specific functions
float hw75_300_get_temp(void);
void hw_palta_reset_oc(void);


