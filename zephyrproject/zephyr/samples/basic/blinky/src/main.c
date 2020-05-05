/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>

#define IN1_NODE DT_ALIAS(in1)
#define IN2_NODE DT_ALIAS(in2)

#if DT_HAS_NODE(IN1_NODE)
#define IN1 DT_GPIO_LABEL(IN1_NODE, gpios)
#define PIN1 DT_GPIO_PIN(IN1_NODE, gpios)
#endif

#if DT_HAS_NODE(IN2_NODE)
#define IN2 DT_GPIO_LABEL(IN2_NODE, gpios)
#define PIN2 DT_GPIO_PIN(IN2_NODE, gpios)
#endif

#ifndef FLAGS
#define FLAGS 0
#endif

void main(void)
{
	struct device *dev;
	struct device *dev1;

	dev = device_get_binding(IN1);
	dev1 = device_get_binding(IN2);
	gpio_pin_set(dev, PIN1, true);
	gpio_pin_set(dev1, PIN2, true);
	printk("dupa");
	//pwm_dev = device_get_binding(DT_ALIAS_PWM_1_LABEL);
}
