
//****************************************************//
//                                                    //
//                 ADC_Comparator                     //     
//                                                    // 
//****************************************************//

/*
Name:		ADC_Comparator
Created:	09/08/2016 08:17:16 AM
Author:		Carlos Beltramello
Company:	QL Labor - Continental AG
*/


#include <pwm_lib.h>
#include <pwm_defs.h>
#include <tc_lib.h>
#include <tc_defs.h>
#include "Arduino.h"
#include "variant.h"


#define PWM_PERIOD_PIN_6 70000  // 10000 µsecs = 10ms 10kHz
#define PWM_DUTY_PIN_6 35000// 1000 msecs in hundredth of usecs (1e-8 secs)

using namespace arduino_due::pwm_lib;

int count = 0;
float voltage = 0;

#define Serial SerialUSB
pwm<pwm_pin::PWML0_PC2> pwm_pin20;
void Config_PWM(void)
{
	/* Enable PWMC peripheral clock. */
	pmc_enable_periph_clk(ID_PWM);

	//pwm<pwm_pin::PWMH0_PB12> pwm_pin20;

	PWMC_ConfigureEventLineMode(PWM, 0, 1);
	PWMC_ConfigureComparisonUnit(PWM, 0, PWM_DUTY_PIN_6, 1);

	PWMC_EnableChannel(PWM, 0);

	pwm_pin20.start(PWM_PERIOD_PIN_6, PWM_DUTY_PIN_6);

	//adc_configure_trigger(ADC, ADC_TRIG_PWM_EVENT_LINE_0, 0);
	Serial.println("PWM-OK");
}



void Config_ADC(void)
{

	/* Enable peripheral clock. */
	pmc_enable_periph_clk(ID_ADC);

	/* Initialize ADC. */
	adc_init(ADC, SystemCoreClock, ADC_FREQ_MAX, 0); //ADC_STARTUP_FAST

			 /* Set ADC timing. */
			 /* Formula:
			 *     Transfer Time = (TRANSFER * 2 + 3) / ADCClock
			 *     Tracking Time = (TRACKTIM + 1) / ADCClock
			 *     Settling Time = settling value / ADCClock
			 *
			 *     Transfer Time = (1 * 2 + 3) / 6.4MHz = 781 ns
			 *     Tracking Time = (1 + 1) / 6.4MHz = 312 ns
			 *     Settling Time = 3 / 6.4MHz = 469 ns
			 */
	adc_configure_timing(ADC, 0, ADC_SETTLING_TIME_0, 0);

	adc_disable_interrupt(ADC, 0xFFFFFFFF);
	adc_set_resolution(ADC, ADC_12_BITS);
	adc_configure_power_save(ADC, ADC_MR_SLEEP_NORMAL, ADC_MR_FWUP_OFF);

	/* Disable sequencer. */
	adc_stop_sequencer(ADC);

	/* Enable channels. */
	adc_enable_channel(ADC, ADC_CHANNEL_7);

	adc_configure_trigger(ADC, ADC_TRIG_SW, 1);
	//adc_configure_trigger(ADC, ADC_TRIG_PWM_EVENT_LINE_0, 0);
	/*Comparison Mode setup*/
	adc_set_comparison_channel(ADC, ADC_CHANNEL_7);
	adc_set_comparison_mode(ADC, ADC_EMR_CMPMODE_HIGH);
	adc_set_comparison_window(ADC, 0, 2555);

	/*ADC Setup (continue) */
	adc_set_bias_current(ADC, 1);
	adc_disable_tag(ADC);
	adc_disable_ts(ADC);
	adc_stop_sequencer(ADC);


	/* Enable Data ready interrupt. */
	adc_enable_interrupt(ADC, ADC_IER_COMPE);

	/* Enable ADC interrupt. */
	NVIC_EnableIRQ(ADC_IRQn);
	adc_start(ADC);


	Serial.println("ADC-OK");
}

int interval = 700;
uint32_t previousMillis = 0;
uint32_t currentMillis = 0;
uint16_t us_adc[4];
void ADC_Handler(void)
{

	uint32_t ul_mode;
	
	uint32_t currentMillis = micros();
	while ((ADC->ADC_ISR & 0x80) == 0); // wait for conversion
	//delayMicroseconds(700);
	
	if (currentMillis - previousMillis >= interval)
	{
		digitalWrite(8, HIGH);
		digitalWrite(8, LOW);
		previousMillis = currentMillis;
	}
	if ((adc_get_status(ADC) & ADC_ISR_COMPE) == ADC_ISR_COMPE)
	{
		switch (count)
		{
		case 3:
			us_adc[count] = adc_get_channel_value(ADC, ADC_CHANNEL_7);
			Serial.println(us_adc[0] * (3.3 / 4096.0));
			Serial.println(us_adc[1] * (3.3 / 4096.0));
			Serial.println(us_adc[2] * (3.3 / 4096.0));
			Serial.println(us_adc[3] * (3.3 / 4096.0));
			count = 0;
			break;
		default:
			us_adc[count] = adc_get_channel_value(ADC, ADC_CHANNEL_7);
			count++;
			break;
		}

		//ul_mode = adc_get_comparison_mode(ADC);
		
	/*	digitalWrite(8, HIGH);
		digitalWrite(8, LOW);*/
	}
}


void setup()
{
	Serial.begin(115200);
	Config_ADC();
	//Config_PWM();
	Serial.print("CONFIG-OK ");
	pinMode(8, OUTPUT);
}

void loop()
{

}

