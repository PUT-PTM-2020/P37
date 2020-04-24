/*
 * Copyright (c) 2017, NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT nxp_imx_gpio

#include <errno.h>
#include <device.h>
#include <drivers/gpio.h>
#include <soc.h>
#include <fsl_common.h>
#include <fsl_gpio.h>

#include "gpio_utils.h"

struct mcux_igpio_config {
	/* gpio_driver_config needs to be first */
	struct gpio_driver_config common;
	GPIO_Type *base;
};

struct mcux_igpio_data {
	/* gpio_driver_data needs to be first */
	struct gpio_driver_data general;
	/* port ISR callback routine address */
	sys_slist_t callbacks;
	/* pin callback routine enable flags, by pin number */
	u32_t pin_callback_enables;
};

static int mcux_igpio_configure(struct device *dev,
				gpio_pin_t pin, gpio_flags_t flags)
{
	const struct mcux_igpio_config *config = dev->config->config_info;
	GPIO_Type *base = config->base;

	if (((flags & GPIO_INPUT) != 0) && ((flags & GPIO_OUTPUT) != 0)) {
		return -ENOTSUP;
	}

	if ((flags & GPIO_SINGLE_ENDED) != 0) {
		return -ENOTSUP;
	}

	if (((flags & GPIO_PULL_UP) != 0) || ((flags & GPIO_PULL_DOWN) != 0)) {
		return -ENOTSUP;
	}

	if (flags & GPIO_OUTPUT_INIT_HIGH) {
		base->DR_SET = BIT(pin);
	}

	if (flags & GPIO_OUTPUT_INIT_LOW) {
		base->DR_CLEAR = BIT(pin);
	}

	WRITE_BIT(base->GDIR, pin, flags & GPIO_OUTPUT);

	return 0;
}

static int mcux_igpio_port_get_raw(struct device *dev, u32_t *value)
{
	const struct mcux_igpio_config *config = dev->config->config_info;
	GPIO_Type *base = config->base;

	*value = base->DR;

	return 0;
}

static int mcux_igpio_port_set_masked_raw(struct device *dev, u32_t mask,
		u32_t value)
{
	const struct mcux_igpio_config *config = dev->config->config_info;
	GPIO_Type *base = config->base;

	base->DR = (base->DR & ~mask) | (mask & value);

	return 0;
}

static int mcux_igpio_port_set_bits_raw(struct device *dev, u32_t mask)
{
	const struct mcux_igpio_config *config = dev->config->config_info;
	GPIO_Type *base = config->base;

	base->DR_SET = mask;

	return 0;
}

static int mcux_igpio_port_clear_bits_raw(struct device *dev, u32_t mask)
{
	const struct mcux_igpio_config *config = dev->config->config_info;
	GPIO_Type *base = config->base;

	base->DR_CLEAR = mask;

	return 0;
}

static int mcux_igpio_port_toggle_bits(struct device *dev, u32_t mask)
{
	const struct mcux_igpio_config *config = dev->config->config_info;
	GPIO_Type *base = config->base;

	base->DR_TOGGLE = mask;

	return 0;
}

static int mcux_igpio_pin_interrupt_configure(struct device *dev,
		gpio_pin_t pin, enum gpio_int_mode mode,
		enum gpio_int_trig trig)
{
	const struct mcux_igpio_config *config = dev->config->config_info;
	struct mcux_igpio_data *data = dev->driver_data;
	GPIO_Type *base = config->base;
	unsigned int key;
	u8_t icr;
	int shift;

	if (mode == GPIO_INT_MODE_DISABLED) {
		key = irq_lock();

		WRITE_BIT(base->IMR, pin, 0);
		WRITE_BIT(data->pin_callback_enables, pin, 0);

		irq_unlock(key);

		return 0;
	}

	if ((mode == GPIO_INT_MODE_EDGE) && (trig == GPIO_INT_TRIG_LOW)) {
		icr = 3;
	} else if ((mode == GPIO_INT_MODE_EDGE) &&
		   (trig == GPIO_INT_TRIG_HIGH)) {
		icr = 2;
	} else if ((mode == GPIO_INT_MODE_LEVEL) &&
		   (trig == GPIO_INT_TRIG_HIGH)) {
		icr = 1;
	} else {
		icr = 0;
	}

	if (pin < 16) {
		shift = 2 * pin;
		base->ICR1 = (base->ICR1 & ~(3 << shift)) | (icr << shift);
	} else if (pin < 32) {
		shift = 2 * (pin - 16);
		base->ICR2 = (base->ICR2 & ~(3 << shift)) | (icr << shift);
	} else {
		return -EINVAL;
	}

	key = irq_lock();

	WRITE_BIT(base->EDGE_SEL, pin, trig == GPIO_INT_TRIG_BOTH);
	WRITE_BIT(base->ISR, pin, 1);
	WRITE_BIT(base->IMR, pin, 1);
	WRITE_BIT(data->pin_callback_enables, pin, 1);

	irq_unlock(key);

	return 0;
}

static int mcux_igpio_manage_callback(struct device *dev,
				     struct gpio_callback *callback, bool set)
{
	struct mcux_igpio_data *data = dev->driver_data;

	return gpio_manage_callback(&data->callbacks, callback, set);
}

static int mcux_igpio_enable_callback(struct device *dev,
				     gpio_pin_t pin)
{
	struct mcux_igpio_data *data = dev->driver_data;

	data->pin_callback_enables |= BIT(pin);

	return 0;
}

static int mcux_igpio_disable_callback(struct device *dev,
				      gpio_pin_t pin)
{
	struct mcux_igpio_data *data = dev->driver_data;

	data->pin_callback_enables &= ~BIT(pin);

	return 0;
}

static void mcux_igpio_port_isr(void *arg)
{
	struct device *dev = (struct device *)arg;
	const struct mcux_igpio_config *config = dev->config->config_info;
	struct mcux_igpio_data *data = dev->driver_data;
	GPIO_Type *base = config->base;
	u32_t enabled_int, int_flags;

	int_flags = base->ISR;
	enabled_int = int_flags & data->pin_callback_enables;
	base->ISR = enabled_int;

	gpio_fire_callbacks(&data->callbacks, dev, enabled_int);
}

static const struct gpio_driver_api mcux_igpio_driver_api = {
	.pin_configure = mcux_igpio_configure,
	.port_get_raw = mcux_igpio_port_get_raw,
	.port_set_masked_raw = mcux_igpio_port_set_masked_raw,
	.port_set_bits_raw = mcux_igpio_port_set_bits_raw,
	.port_clear_bits_raw = mcux_igpio_port_clear_bits_raw,
	.port_toggle_bits = mcux_igpio_port_toggle_bits,
	.pin_interrupt_configure = mcux_igpio_pin_interrupt_configure,
	.manage_callback = mcux_igpio_manage_callback,
	.enable_callback = mcux_igpio_enable_callback,
	.disable_callback = mcux_igpio_disable_callback,
};

#define MCUX_IGPIO_IRQ_INIT(n, i)					\
	do {								\
		IRQ_CONNECT(DT_INST_IRQ_BY_IDX(n, i, irq),		\
			    DT_INST_IRQ_BY_IDX(n, i, priority),		\
			    mcux_igpio_port_isr,			\
			    DEVICE_GET(mcux_igpio_##n), 0);		\
									\
		irq_enable(DT_INST_IRQ_BY_IDX(n, i, irq));		\
	} while (0)

#define MCUX_IGPIO_INIT(n)						\
	static int mcux_igpio_##n##_init(struct device *dev);		\
									\
	static const struct mcux_igpio_config mcux_igpio_##n##_config = {\
		.common = {						\
			.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_INST(n),\
		},							\
		.base = (GPIO_Type *)DT_INST_REG_ADDR(n),		\
	};								\
									\
	static struct mcux_igpio_data mcux_igpio_##n##_data;		\
									\
	DEVICE_AND_API_INIT(mcux_igpio_##n, DT_INST_LABEL(n),		\
			    mcux_igpio_##n##_init,			\
			    &mcux_igpio_##n##_data,			\
			    &mcux_igpio_##n##_config,			\
			    POST_KERNEL,				\
			    CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,	\
			    &mcux_igpio_driver_api);			\
									\
	static int mcux_igpio_##n##_init(struct device *dev)		\
	{								\
		MCUX_IGPIO_IRQ_INIT(n, 0);				\
									\
		IF_ENABLED(DT_INST_IRQ_HAS_IDX(n, 1),			\
			   (MCUX_IGPIO_IRQ_INIT(n, 1);))		\
									\
		return 0;						\
	}

DT_INST_FOREACH(MCUX_IGPIO_INIT)
