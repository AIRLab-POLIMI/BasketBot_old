#include "ch.h"
#include "hal.h"

//#include "config.h"

#include <r2p/common.hpp>
#include <r2p/Middleware.hpp>
#include <r2p/Node.hpp>
#include <r2p/Publisher.hpp>
#include <r2p/Subscriber.hpp>

#include <r2p/node/pid.hpp>

#include "CurrentLoop.h"

namespace r2p {

extern PWMConfig pwmcfg;

#define M1 72
#define M2 73
#define M3 89

/*===========================================================================*/
/* Motor parameters.                                                         */
/*===========================================================================*/
#define ADC_NUM_CHANNELS   1
#define ADC_BUF_DEPTH      32

#define _R                 0.117
#define _L                 2.5e-5
#define _Kt                0.0164
#define _Tinv              26.0*10.0/3.0

#define _OmegaC            6000.0
#define _Kp                _OmegaC*_L
#define _Ti                (_L/_R)
#define _Ts                (252.0/72.0e6*(double)ADC_BUF_DEPTH)
#define _maxV              24.0

#define _maxI              25.0
#define _precision         4096.0
#define _currentPrecision  2.0*_maxI/_precision

#define _pwmTicks          4096.0

static PID current_pid;
static int index = 0;
static int pwm = 0;

/*===========================================================================*/
/* Utility functions.                                                        */
/*===========================================================================*/

static uint8_t stm32_id8(void) {
	const unsigned long * uid = (const unsigned long *) 0x1FFFF7E8;

	return (uid[2] & 0xFF);
}

/*===========================================================================*/
/* Current sense related.                                                    */
/*===========================================================================*/

adcsample_t adc_samples[ADC_NUM_CHANNELS * ADC_BUF_DEPTH];

void current_callback(ADCDriver *adcp, adcsample_t *buffer, size_t n) {

	//Compute current
	double current = 0.0;
	for(unsigned int i = 0; i < n; i++)
	{
		current += buffer[i];
	}

	current /= (double) n;

	current *= _currentPrecision;
	current -= _maxI;

	//compute control signal
	double voltage = current_pid.update(current);

	//Compute pwm signal and apply
	pwm = _pwmTicks/_maxV*voltage;

	chSysLock()
	;

	if (pwm > 0) {
		pwm_lld_enable_channel(&PWM_DRIVER, 1, pwm);
		pwm_lld_enable_channel(&PWM_DRIVER, 0, 0);
	} else {
		pwm_lld_enable_channel(&PWM_DRIVER, 1, 0);
		pwm_lld_enable_channel(&PWM_DRIVER, 0, -pwm);
	}
	chSysUnlock();

	palTogglePad(LED2_GPIO, LED2);

}

/*
 * ADC conversion group.
 * Mode:        Circular buffer, 8 samples of 1 channel.
 * Channels:    IN10.
 */
static const ADCConversionGroup adcgrpcfg = {
TRUE,
ADC_NUM_CHANNELS,
current_callback,
NULL, 0, 0, /* CR1, CR2 */
0, ADC_SMPR2_SMP_AN3(ADC_SAMPLE_239P5), /* SMPR2 */
ADC_SQR1_NUM_CH(ADC_NUM_CHANNELS), 0, /* SQR2 */
ADC_SQR3_SQ1_N(ADC_CHANNEL_IN3) };


/*===========================================================================*/
/* Motor control nodes.                                                      */
/*===========================================================================*/

/*
 * PID node.
 */

msg_t current_pid2_node(void * arg) {
	Node node("pid");
	Subscriber<Current2Msg, 5> current_sub;
	Current2Msg * msgp;
	Time last_setpoint(0);

	(void) arg;

	chRegSetThreadName("pid_current");

	current_pid.config(_Kp, _Ti, 0.0, _Ts, -_maxV, _maxV);

	switch (stm32_id8()) {
	case M1:
		index = 0;
		break;
	case M2:
		index = 1;
		break;
	default:
		break;
	}

	// Init motor driver
	palSetPad(DRIVER_GPIO, DRIVER_RESET);
	chThdSleepMilliseconds(500);
	pwmStart(&PWM_DRIVER, &pwmcfg);

	// Start the ADC driver and conversion
	adcStart(&ADC_DRIVER, NULL);
	adcStartConversion(&ADC_DRIVER, &adcgrpcfg, adc_samples, ADC_BUF_DEPTH);


	/*node.subscribe(current_sub, "current2");

	for (;;) {
		if (node.spin(Time::ms(100))) {
			if (current_sub.fetch(msgp)) {
				current_pid.set(msgp->value[index]);
				last_setpoint = Time::now();
				current_sub.release(*msgp);

				palTogglePad(LED3_GPIO, LED3);

			} else if (Time::now() - last_setpoint > Time::ms(100)) {
				current_pid.set(0);
			}
		} else {
			// Stop motor if no messages for 100 ms
			pwm_lld_disable_channel(&PWM_DRIVER, 0);
			pwm_lld_disable_channel(&PWM_DRIVER, 1);

			palTogglePad(LED4_GPIO, LED4);
		}
	}*/

	current_pid.set(0.5);

	for (;;) {
		if (node.spin(Time::ms(100))) {
			palTogglePad(LED3_GPIO, LED3);
		}
		else {
			palTogglePad(LED4_GPIO, LED4);
		}
	}

	return CH_SUCCESS;
}

} /* namespace r2p */
