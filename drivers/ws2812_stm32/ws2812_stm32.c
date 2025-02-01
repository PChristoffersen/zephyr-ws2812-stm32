/*
 * Copyright (c) 2025 Peter Christoffersen
 * SPDX-License-Identifier: Apache-2.0
 */


#define DT_DRV_COMPAT worldsemi_ws2812_stm32

#include <soc.h>
#include <stm32_ll_rcc.h>
#include <stm32_ll_tim.h>
#include <stm32_ll_dma.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/dma/dma_stm32.h>
#include <zephyr/dt-bindings/led/led.h>
#include <zephyr/logging/log.h>


LOG_MODULE_REGISTER(ws2812_stm32, CONFIG_WS2812_STM32_LOG_LEVEL);


typedef uint16_t buffer_value_t;

struct ws2812_stm32_cfg {
	TIM_TypeDef *timer;
	const struct pwm_dt_spec pwm_dt;
	const struct device *dma_dev;
	uint32_t dma_channel;
	uint8_t dma_priority;

	uint16_t one_pulse_ns;
	uint16_t zero_pulse_ns;
	uint16_t reset_delay_us;

    uint8_t num_colors;
    const uint8_t *color_mapping;
    size_t chain_length;

	buffer_value_t *dma_buffer;
	size_t dma_buffer_size;

	bool async;
	struct led_rgb *pixel_buffer;
};

struct ws2812_stm32_data {
	buffer_value_t period;
	buffer_value_t one_pulse;
	buffer_value_t zero_pulse;
	size_t reset_delay_periods;

	struct led_rgb *transfer_pixels;
	size_t transfer_pixels_remaining;
	size_t transfer_blocks_remaining;

	struct k_sem completion_sem;
	int dma_error;

	struct dma_config dma_config;
	struct dma_block_config dma_blk_cfg;
};


/** Maximum number of timer channels : some stm32 soc have 6 else only 4 */
#define TIMER_MAX_CH 4u

/** Channel to DMA burst base address */
static const uint32_t ch2dmaburst[TIMER_MAX_CH] = {
	LL_TIM_DMABURST_BASEADDR_CCR1, LL_TIM_DMABURST_BASEADDR_CCR2,
	LL_TIM_DMABURST_BASEADDR_CCR3, LL_TIM_DMABURST_BASEADDR_CCR4,
};

/** Channel to enable DMA request */
static void (*const enable_dma_req[TIMER_MAX_CH])(TIM_TypeDef *) = {
	LL_TIM_EnableDMAReq_CC1, LL_TIM_EnableDMAReq_CC2,
	LL_TIM_EnableDMAReq_CC3, LL_TIM_EnableDMAReq_CC4,
};


#define PIXEL_SIZE_BITS(num_colors) (num_colors*8)



static inline const struct ws2812_stm32_cfg *dev_cfg(const struct device *dev)
{
	return dev->config;
}

static inline struct ws2812_stm32_data *dev_data(const struct device *dev)
{
	return dev->data;
}


/**
 * Calculate the number of DMA blocks (half buffers) needed to transfer the given number of pixels
 * including the reset delay.
 */
static size_t calculate_dma_blocks(const struct device *dev, size_t num_pixels)
{
	const struct ws2812_stm32_cfg *cfg = dev_cfg(dev);
	const struct ws2812_stm32_data *data = dev_data(dev);
	size_t block_size = cfg->dma_buffer_size/2;
	size_t num_colors = cfg->num_colors;
	size_t num_bits = num_pixels * PIXEL_SIZE_BITS(num_colors);
	size_t num_blocks = (num_bits + data->reset_delay_periods + (block_size - 1)) / block_size;

	#if 0
	LOG_DBG("Calculating DMA blocks for %d pixels", num_pixels);
	LOG_DBG("Block size: %d", block_size);
	LOG_DBG("DMA buffer size: %d", cfg->dma_buffer_size);
	LOG_DBG("DMA buffer size (bits): %d", cfg->dma_buffer_size * sizeof(buffer_value_t));
	LOG_DBG("Number of colors: %d", num_colors);
	LOG_DBG("Number of bits: %d+%d = %d", num_bits, data->reset_delay_periods, num_bits + data->reset_delay_periods);
	LOG_DBG("Number of blocks: %d", num_blocks);
	#endif

	return num_blocks;
}


static inline void strip_dma_fill_pixel(const struct device *dev, buffer_value_t *buffer, const struct led_rgb *pixel)
{
	const struct ws2812_stm32_cfg *cfg = dev_cfg(dev);
	struct ws2812_stm32_data *data = dev_data(dev);

	uint8_t i;
	uint8_t j;
	uint8_t color;
	uint8_t num_colors = cfg->num_colors;

	for (i = 0; i < num_colors; i++) {
		switch (cfg->color_mapping[i]) {
		case LED_COLOR_ID_WHITE:
			color = 0;
			break;
		case LED_COLOR_ID_RED:
			color = pixel->r;
			break;
		case LED_COLOR_ID_GREEN:
			color = pixel->g;
			break;
		case LED_COLOR_ID_BLUE:
			color = pixel->b;
			break;
		default:
			color = 0;
			break;
		}

		for (j = 0; j < 8; j++) {
			*buffer = (color & BIT(7-j)) ? data->one_pulse: data->zero_pulse;
			buffer++;
		}
	}
}


static void strip_dma_fill_block(const struct device *dev, bool first_half)
{
	const struct ws2812_stm32_cfg *cfg = dev_cfg(dev);
	struct ws2812_stm32_data *data = dev_data(dev);
	buffer_value_t *buffer = cfg->dma_buffer;
	size_t buffer_size = cfg->dma_buffer_size/2;

	if (!first_half) {
		buffer += buffer_size;
	}

	while (data->transfer_pixels_remaining && buffer_size) {
		strip_dma_fill_pixel(dev, buffer, data->transfer_pixels);
		data->transfer_pixels++;
		data->transfer_pixels_remaining--;
		buffer += PIXEL_SIZE_BITS(cfg->num_colors);
		buffer_size -= PIXEL_SIZE_BITS(cfg->num_colors);
	}
	if (buffer_size) {
		memset(buffer, 0, buffer_size * sizeof(buffer_value_t));
	}
}


static void strip_dma_callback(const struct device *dev, void *user_data, uint32_t channel, int status)
{
	struct ws2812_stm32_data *data = dev_data(user_data);

	if (status < 0) {
		data->dma_error = status;
		dma_stop(dev, channel);
		k_sem_give(&data->completion_sem);
		return;
	}

	data->transfer_blocks_remaining--;

	if (status == DMA_STATUS_BLOCK) {
		strip_dma_fill_block(user_data, true);
	}
	else if (status == DMA_STATUS_COMPLETE) {
		strip_dma_fill_block(user_data, false);
	}

	if (data->transfer_blocks_remaining == 0) {
		data->dma_error = dma_stop(dev, channel);
		k_sem_give(&data->completion_sem);
	}

}


static int strip_dma_start(const struct device *dev, struct led_rgb *pixels,size_t num_pixels)
{
	const struct ws2812_stm32_cfg *cfg = dev_cfg(dev);
	struct ws2812_stm32_data *data = dev->data;
	int ret;

	ret = dma_config(cfg->dma_dev, cfg->dma_channel, &data->dma_config);
	if (ret != 0) {
		LOG_ERR("Problem setting up DMA: %d", ret);
		return ret;
	}

	data->transfer_pixels = pixels;
	data->transfer_pixels_remaining = num_pixels;
	data->transfer_blocks_remaining = calculate_dma_blocks(dev, num_pixels);
	strip_dma_fill_block(dev, true);
	strip_dma_fill_block(dev, false);

	ret = dma_start(cfg->dma_dev, cfg->dma_channel);
	if (ret != 0) {
		LOG_ERR("Problem starting DMA: %d", ret);
		return ret;
	}

	return ret;
}


static int strip_dma_wait(const struct device *dev)
{
	struct ws2812_stm32_data *data = dev_data(dev);
	int ret;

	ret = k_sem_take(&data->completion_sem, K_FOREVER);
	if (ret != 0) {
		LOG_ERR("Problem waiting for DMA: %d", ret);
	}
	
	return ret;
}


static int strip_dma_configure(const struct device *dev)
{
	const struct ws2812_stm32_cfg *cfg = dev_cfg(dev);
	struct ws2812_stm32_data *data = dev->data;
	TIM_TypeDef *timer = cfg->timer;
	struct dma_block_config *blk_cfg;

	LOG_DBG("Configuring DMA");

	blk_cfg = &data->dma_blk_cfg;

	blk_cfg->block_size = cfg->dma_buffer_size * sizeof(buffer_value_t);

	blk_cfg->source_address = (uint32_t)cfg->dma_buffer;
	blk_cfg->source_addr_adj = DMA_ADDR_ADJ_INCREMENT;
	blk_cfg->source_reload_en = 1;

	blk_cfg->dest_address = (uint32_t)(&(timer->DMAR));
	blk_cfg->dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE;
	blk_cfg->dest_reload_en = 1;

	data->dma_config.head_block = blk_cfg;
	data->dma_config.user_data = (void*)dev;

	return 0;
}


static int strip_timer_configure(const struct device *dev)
{
	const struct ws2812_stm32_cfg *cfg = dev_cfg(dev);
	TIM_TypeDef *timer = cfg->timer;

	LOG_DBG("Configuring Timer");

	// Setup the timer
	size_t channel_idx = cfg->pwm_dt.channel - 1;
	uint32_t ll_dma_burst_addr = ch2dmaburst[channel_idx];	

	LL_TIM_DisableCounter(timer);
	LL_TIM_SetCounter(timer, 0);

	LL_TIM_ConfigDMABurst(timer, ll_dma_burst_addr, LL_TIM_DMABURST_LENGTH_1TRANSFER);
	enable_dma_req[channel_idx](timer);

	LL_TIM_EnableUpdateEvent(timer);
	LL_TIM_GenerateEvent_UPDATE(timer);

	LL_TIM_EnableCounter(timer);

	return 0;
}




static int ws2812_stm32_update_rgb(const struct device *dev,
				   struct led_rgb *pixels,
				   size_t num_pixels)
{
	const struct ws2812_stm32_cfg *cfg = dev_cfg(dev);
	int ret;
    LOG_DBG("Updating %d pixels", num_pixels);

	if (num_pixels > cfg->chain_length) {
		LOG_ERR("Too many pixels: %d > %d", num_pixels, cfg->chain_length);
		return -EINVAL;
	}

	ret = strip_dma_start(dev, pixels, num_pixels);
	if (ret != 0) {
		LOG_ERR("Problem starting DMA: %d", ret);
		return ret;
	}

	// Wait for transfer to complete
	ret = strip_dma_wait(dev);
	if (ret != 0) {
		LOG_ERR("Problem waiting for DMA: %d", ret);
		return ret;
	}

    return ret;
}


static int ws2812_stm32_update_rgb_async(const struct device *dev,
				   struct led_rgb *pixels,
				   size_t num_pixels)
{
	const struct ws2812_stm32_cfg *cfg = dev_cfg(dev);
	int ret;
    LOG_DBG("Updating %d pixels (async)", num_pixels);

	if (num_pixels > cfg->chain_length) {
		LOG_ERR("Too many pixels: %d > %d", num_pixels, cfg->chain_length);
		return -EINVAL;
	}

	// Wait for previous transfer
	ret = strip_dma_wait(dev);
	if (ret != 0) {
		LOG_ERR("Problem waiting for DMA: %d", ret);
		return ret;
	}

	memcpy(cfg->pixel_buffer, pixels, num_pixels * sizeof(struct led_rgb));

	ret = strip_dma_start(dev, cfg->pixel_buffer, num_pixels);
	if (ret != 0) {
		LOG_ERR("Problem starting DMA: %d", ret);
		return ret;
	}

    return ret;
}



static size_t ws2812_stm32_length(const struct device *dev)
{
	const struct ws2812_stm32_cfg *cfg = dev_cfg(dev);
	return cfg->chain_length;
}


static int ws2812_stm32_init(const struct device *dev)
{
	struct ws2812_stm32_data *data = dev->data;
	const struct ws2812_stm32_cfg *cfg = dev_cfg(dev);
	int ret;

	if (!device_is_ready(cfg->dma_dev)) {
		LOG_ERR("DMA device not ready");
		return -ENODEV;
	}
	if (!device_is_ready(cfg->pwm_dt.dev)) {
		LOG_ERR("PWM device not ready");
		return -ENODEV;
	}

	k_sem_init(&data->completion_sem, cfg->async?1:0, 1);

	for (uint8_t i = 0; i < cfg->num_colors; i++) {
		switch (cfg->color_mapping[i]) {
		case LED_COLOR_ID_WHITE:
		case LED_COLOR_ID_RED:
		case LED_COLOR_ID_GREEN:
		case LED_COLOR_ID_BLUE:
			break;
		default:
			LOG_ERR("%s: invalid channel to color mapping."
				"Check the color-mapping DT property",
				dev->name);
			return -EINVAL;
		}
	}
	if (cfg->dma_buffer_size<1 || cfg->dma_buffer_size & 1) {
		LOG_ERR("DMA buffer size must be larger than 1 and even");
		return -EINVAL;
	}


	// Convert ns times to cycles
	uint64_t cycles_per_sec;
	ret = pwm_get_cycles_per_sec(cfg->pwm_dt.dev, cfg->pwm_dt.channel, &cycles_per_sec);
	if (ret != 0) {
		LOG_ERR("Problem getting cycles per second: %d", ret);
		return ret;
	}
	data->period = (cfg->pwm_dt.period * cycles_per_sec) / NSEC_PER_SEC;
	data->one_pulse = (cfg->one_pulse_ns * cycles_per_sec) / NSEC_PER_SEC;
	data->zero_pulse = (cfg->zero_pulse_ns * cycles_per_sec) / NSEC_PER_SEC;
	// Calculate the number of periods needed for the reset delay
	uint64_t reset_delay_cycles = (cfg->reset_delay_us * cycles_per_sec) / USEC_PER_SEC;
	data->reset_delay_periods = (reset_delay_cycles + data->period - 1) / data->period;
	#if 0
	LOG_DBG("Reset delay: cycles=%lld, periods=%d", reset_delay_cycles, data->reset_delay_periods);
	LOG_DBG("Timings (ns): period=%d, one_pulse=%d, zero_pulse=%d", cfg->pwm_dt.period, cfg->one_pulse_ns, cfg->zero_pulse_ns);
	LOG_DBG("Timings (cycles): period=%d, one_pulse=%d, zero_pulse=%d", data->period, data->one_pulse, data->zero_pulse);
	#endif

	// Let the PWM driver initialize the timer to generate 0 pulses
	// That saves us from configuring the timer ourselves
	ret = pwm_set_pulse_dt(&cfg->pwm_dt, 0);
	if (ret != 0) {
		LOG_ERR("Problem configuring strip (PWM): %d", ret);
		return ret;
	}
	ret = strip_dma_configure(dev);
	if (ret != 0) {
		LOG_ERR("Problem configuring strip (DMA): %d", ret);
		return ret;
	}
	ret = strip_timer_configure(dev);
	if (ret != 0) {
		LOG_ERR("Problem configuring strip (Timer): %d", ret);
		return ret;
	}

    return 0;
};


static DEVICE_API(led_strip, ws2812_stm32_api) = {
	.update_rgb = ws2812_stm32_update_rgb,
	.length = ws2812_stm32_length,
};

static DEVICE_API(led_strip, ws2812_stm32_api_async) = {
	.update_rgb = ws2812_stm32_update_rgb_async,
	.length = ws2812_stm32_length,
};


#define WS2812_STM32_TIMER(idx)	\
	DT_PARENT(DT_PWMS_CTLR(DT_DRV_INST(idx)))

#define WS2812_COLOR_MAPPING_NAME(idx)	\
	ws2812_##idx##_color_mapping

#define WS2812_COLOR_MAPPING(idx)	\
    static const uint8_t WS2812_COLOR_MAPPING_NAME(idx)[] = DT_INST_PROP(idx, color_mapping);

#define WS2812_NUM_COLORS(idx) \
	(DT_INST_PROP_LEN(idx, color_mapping))

#define WS2812_CHAIN_LENGTH(idx) \
	(DT_INST_PROP(idx, chain_length))

#define WS2812_STM32_DMA_PIXELS(idx) \
	(DT_INST_PROP(idx, dma_buffer_size))

#define WS2812_STM32_DMA_PIXEL_BITS(idx) \
	(DT_INST_PROP(idx, dma_buffer_size)*PIXEL_SIZE_BITS(WS2812_NUM_COLORS(idx)))

#define WS2812_STM32_DMA_BUFFER_NAME(idx) \
	ws2812_stm32_##idx##_dma_buf

#define WS2812_STM32_DMA_BUFFER(idx) \
	static buffer_value_t WS2812_STM32_DMA_BUFFER_NAME(idx)[WS2812_STM32_DMA_PIXEL_BITS(idx)]

#define WS2312_STM32_ASYNC(idx) \
	DT_INST_PROP(idx, async)

#define WS2812_STM32_PIXEL_BUFFER_NAME(idx) \
	ws2812_stm32_##idx##_pixel_buf

#define WS2812_STM32_PIXEL_BUFFER(idx) 		\
	COND_CODE_1(WS2312_STM32_ASYNC(idx),	\
		(static struct led_rgb WS2812_STM32_PIXEL_BUFFER_NAME(idx)[WS2812_CHAIN_LENGTH(idx)]),\
		())

#define WS2812_STM32_PIXEL_BUFFER_PTR(idx)	\
	COND_CODE_1(WS2312_STM32_ASYNC(idx),	\
		(WS2812_STM32_PIXEL_BUFFER_NAME(idx)),\
		(NULL))

#define WS2812_STM32_API(idx) \
	COND_CODE_1(WS2312_STM32_ASYNC(idx),	\
		(ws2812_stm32_api_async),\
		(ws2812_stm32_api))


#define WS2812_DMA_CONFIG(index, src_dev, dest_dev)						\
	{																	\
			.dma_slot = STM32_DMA_SLOT_BY_IDX(index, 0, slot),			\
			.channel_direction = MEMORY_TO_PERIPHERAL,					\
			.source_data_size = sizeof(buffer_value_t),					\
			.dest_data_size = sizeof(buffer_value_t),					\
			.source_burst_length = 1,									\
			.dest_burst_length = 1,										\
			.channel_priority = STM32_DMA_CONFIG_PRIORITY(				\
				STM32_DMA_CHANNEL_CONFIG_BY_IDX(index, 0)),				\
			.dma_callback = strip_dma_callback,							\
			.block_count = 1,											\
	}


#define WS2812_STM32_DEVICE(idx)										\
	WS2812_STM32_DMA_BUFFER(idx);										\
	WS2812_STM32_PIXEL_BUFFER(idx);										\
	WS2812_COLOR_MAPPING(idx);											\
																		\
	static const struct ws2812_stm32_cfg ws2812_stm32_##idx##_cfg = { 	\
		.timer = (TIM_TypeDef *)DT_REG_ADDR(WS2812_STM32_TIMER(idx)),	\
		.pwm_dt = PWM_DT_SPEC_INST_GET(idx),							\
		.dma_dev = DEVICE_DT_GET(DT_INST_DMAS_CTLR_BY_IDX(idx, 0)),		\
		.dma_channel = DT_INST_DMAS_CELL_BY_IDX(idx, 0, channel),		\
		.one_pulse_ns = DT_INST_PROP(idx, one_pulse),					\
		.zero_pulse_ns = DT_INST_PROP(idx, zero_pulse),					\
		.reset_delay_us = DT_INST_PROP(idx, reset_delay),				\
		.num_colors = WS2812_NUM_COLORS(idx),							\
		.color_mapping = WS2812_COLOR_MAPPING_NAME(idx),				\
		.chain_length = WS2812_CHAIN_LENGTH(idx),						\
		.dma_buffer = WS2812_STM32_DMA_BUFFER_NAME(idx),				\
		.dma_buffer_size = WS2812_STM32_DMA_PIXEL_BITS(idx),			\
		.async = WS2312_STM32_ASYNC(idx),								\
		.pixel_buffer = WS2812_STM32_PIXEL_BUFFER_PTR(idx),				\
	};																	\
																		\
	static struct ws2812_stm32_data ws2812_stm32_##idx##_data = {		\
		.dma_config = WS2812_DMA_CONFIG(idx, MEMORY, PERIPHERAL),		\
	};								       								\
																		\
	DEVICE_DT_INST_DEFINE(idx,											\
			    ws2812_stm32_init,										\
			    NULL,													\
			    &ws2812_stm32_##idx##_data,								\
			    &ws2812_stm32_##idx##_cfg, POST_KERNEL,					\
			    CONFIG_WS2812_STM32_INIT_PRIORITY,				\
			    &WS2812_STM32_API(idx));


DT_INST_FOREACH_STATUS_OKAY(WS2812_STM32_DEVICE)

