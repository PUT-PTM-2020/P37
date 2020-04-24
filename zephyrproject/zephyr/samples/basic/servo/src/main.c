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
#define FLAGS 0

#define DT_ALIAS_PWM_0_LABEL "PWM_2"

#ifndef DT_ALIAS_PWM_0_LABEL
#error "PWM device label not defined"
#endif

/* period of servo motor signal ->  2.4ms */
#define PERIOD (USEC_PER_SEC / 490U)

/* all in micro second */
#define STEP 1
#define MINPULSEWIDTH 50 /* Servo 0 degrees */
//#define MIDDLEPULSEWIDTH 1500 /* Servo 90 degrees */
#define MAXPULSEWIDTH 255 /* Servo 180 degrees */

struct device *gpio_Engine_dev;

uint8_t gpio_init(void)
{
	uint8_t ret = 0;
	gpio_Engine_dev = device_get_binding(ENGINE_PORT);
	if (gpio_Engine_dev == NULL) {
		return 1;
	}

	gpio_pin_configure(gpio_Engine_dev, IN1_PIN, GPIO_OUTPUT_ACTIVE | FLAGS);
	gpio_pin_configure(gpio_Engine_dev, IN2_PIN, GPIO_OUTPUT_INACTIVE | FLAGS);
	gpio_pin_configure(gpio_Engine_dev, IN3_PIN, GPIO_OUTPUT_ACTIVE | FLAGS);
	gpio_pin_configure(gpio_Engine_dev, IN4_PIN, GPIO_OUTPUT_INACTIVE | FLAGS);



	return ret;
}

void main(void)
{
	gpio_init();
	struct device *dev;

	int fill = 500;
	dev = device_get_binding(DT_ALIAS_PWM_0_LABEL);

	while (1) {
		pwm_pin_set_usec(dev, 2, PERIOD, fill, 0);
		pwm_pin_set_usec(dev, 1, PERIOD, fill, 0);
		if (fill < 2040) {
			fill += STEP;
		} else {
			fill = 300;
		}
		k_sleep(K_MSEC(10));
	}
}