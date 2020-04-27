#include <zephyr.h>
#include <device.h>
#include <drivers/gpio.h>
#include <sys/printk.h>
#include <drivers/pwm.h>
#include <kernel.h>

#define ENGINE_PORT "GPIOA"
#define IN1_PIN 5
#define IN2_PIN 2
#define IN3_PIN 3
#define IN4_PIN 4

#define SONIC_SENSOR_PORT "GPIOE"
#define TRIG1_PIN 1
#define ECHO1_PIN 0
#define TRIG2_PIN 3
#define ECHO2_PIN 2
#define TRIG3_PIN 5
#define ECHO3_PIN 4

#define TRANSCEIVER_PORT "GPIOB"
#define CE_PIN 0
#define SPI_SCK_PIN 3
#define MOSI_PIN 5
#define MISO_PIN 4
#define IRQ_PIN 1

#define FLAGS 0

#define DT_ALIAS_PWM_2_LABEL "PWM_2"
#define DT_ALIAS_PWM_3_LABEL "PWM_3"

#ifndef DT_ALIAS_PWM_2_LABEL
#error "PWM_2 device label not defined"
#endif

#ifndef DT_ALIAS_PWM_3_LABEL
#error "PWM_3 device label not defined"
#endif

/* period of servo motor signal ->  2.4ms */
#define PERIOD (USEC_PER_SEC / 490U)

/* all in micro second */
#define STEP 1
#define MINPULSEWIDTH 50 /* Servo 0 degrees */
//#define MIDDLEPULSEWIDTH 1500 /* Servo 90 degrees */
#define MAXPULSEWIDTH 255 /* Servo 180 degrees */

struct device *gpio_engine_dev;
struct device *gpio_sonic_sensor_dev;
struct device *gpio_transceiver_dev;
struct device *pwm2_dev, *pwm3_dev;

uint8_t gpio_init(void){
	uint8_t ret = 0;
	gpio_engine_dev = device_get_binding(ENGINE_PORT);
	if (gpio_engine_dev == NULL) {
		return 1;
	}

	/* Engines IN1 - IN4 pins cofiguration */
	gpio_pin_configure(gpio_engine_dev, IN1_PIN, GPIO_OUTPUT_ACTIVE   | FLAGS);
	gpio_pin_configure(gpio_engine_dev, IN2_PIN, GPIO_OUTPUT_INACTIVE | FLAGS);
	gpio_pin_configure(gpio_engine_dev, IN3_PIN, GPIO_OUTPUT_ACTIVE   | FLAGS);
	gpio_pin_configure(gpio_engine_dev, IN4_PIN, GPIO_OUTPUT_INACTIVE | FLAGS);

	return ret;
}

uint8_t pwm_init(void){
   uint8_t ret = 0;
   pwm2_dev = device_get_binding(DT_ALIAS_PWM_2_LABEL);
	
	if (pwm2_dev == NULL){
		return 1;
	}

	return ret;
}


void main(void)
{
	gpio_init();
	pwm2_dev = device_get_binding(DT_ALIAS_PWM_2_LABEL);
	int fill = 500;
	
	while (1) {
		pwm_pin_set_usec(pwm2_dev, 1, PERIOD, fill, 0);
		pwm_pin_set_usec(pwm2_dev, 2, PERIOD, fill, 0);
		if (fill < 2040) {
			fill += STEP;
		} else {
			pwm_pin_set_usec(pwm2_dev, 1, PERIOD, 0, 0);
			k_sleep(K_MSEC(800));
			fill = 0;
		}
		k_sleep(K_MSEC(2));
	}
}