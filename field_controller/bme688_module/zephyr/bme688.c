/* bme688.c - Driver for Bosch Sensortec's BME688 temperature, pressure,
 * humidity and gas sensor
 *
 * https://www.bosch-sensortec.com/bst/products/all_products/bme688
 */

/*
 * Copyright (c) 2018 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT bosch_bme688

#include "bme688.h"
#include <drivers/gpio.h>
#include <drivers/i2c.h>
#include <init.h>
#include <kernel.h>
#include <sys/byteorder.h>
#include <sys/__assert.h>
#include <drivers/sensor.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(bme688, CONFIG_SENSOR_LOG_LEVEL);

static int bme688_reg_read(const struct device *dev, uint8_t start,
						   uint8_t *buf, int size)
{
	const struct bme688_config *config = dev->config;

	return i2c_burst_read_dt(&config->bus, start, buf, size);
}

static int bme688_reg_write(const struct device *dev, uint8_t reg, uint8_t val)
{
	const struct bme688_config *config = dev->config;

	return i2c_reg_write_byte_dt(&config->bus, reg, val);
}

static void bme688_calc_temp(struct bme688_data *data, uint32_t adc_temp)
{
	int64_t var1, var2, var3;

	var1 = ((int32_t)adc_temp >> 3) - ((int32_t)data->par_t1 << 1);
	var2 = (var1 * (int32_t)data->par_t2) >> 11;
	var3 = ((var1 >> 1) * (var1 >> 1)) >> 12;
	var3 = ((var3) * ((int32_t)data->par_t3 << 4)) >> 14;
	data->t_fine = var2 + var3;
	data->calc_temp = ((data->t_fine * 5) + 128) >> 8;
}

static void bme688_calc_press(struct bme688_data *data, uint32_t adc_press)
{
	int32_t var1, var2, var3, press_comp;

	var1 = ((int32_t)data->t_fine >> 1) - 64000;
	var2 = ((((var1 >> 2) * (var1 >> 2)) >> 11) * (int32_t)data->par_p6) >> 2;
	var2 = var2 + ((var1 * (int32_t)data->par_p5) << 1);
	var2 = (var2 >> 2) + ((int32_t)data->par_p4 << 16);
	var1 = (((((var1 >> 2) * (var1 >> 2)) >> 13) * ((int32_t)data->par_p3 << 5)) >> 3) + (((int32_t)data->par_p2 * var1) >> 1);
	var1 = var1 >> 18;
	var1 = ((32768 + var1) * (int32_t)data->par_p1) >> 15;
	press_comp = 1048576 - adc_press;
	press_comp = (uint32_t)((press_comp - (var2 >> 12)) * ((uint32_t)3125));
	if (press_comp >= (1 << 30))
		press_comp = ((press_comp / (uint32_t)var1) << 1);
	else
		press_comp = ((press_comp << 1) / (uint32_t)var1);
	var1 = ((int32_t)data->par_p9 * (int32_t)(((press_comp >> 3) * (press_comp >> 3)) >> 13)) >> 12;
	var2 = ((int32_t)(press_comp >> 2) * (int32_t)data->par_p8) >> 13;
	var3 = ((int32_t)(press_comp >> 8) * (int32_t)(press_comp >> 8) *
			(int32_t)(press_comp >> 8) * (int32_t)data->par_p10) >>
		   17;
	data->calc_press = (int32_t)(press_comp) +
					   ((var1 + var2 + var3 + ((int32_t)data->par_p7 << 7)) >> 4);
}

static void bme688_calc_humidity(struct bme688_data *data, uint16_t adc_humidity)
{
	int32_t var1, var2, var3, var4, var5, var6;
	int32_t temp_scaled, hum;

	temp_scaled = (int32_t)data->calc_temp;
	var1 = (int32_t)adc_humidity - (int32_t)((int32_t)data->par_h1 << 4) - (((temp_scaled * (int32_t)data->par_h3) / ((int32_t)100)) >> 1);
	var2 = ((int32_t)data->par_h2 * (((temp_scaled * (int32_t)data->par_h4) / ((int32_t)100)) + (((temp_scaled * ((temp_scaled * (int32_t)data->par_h5) / ((int32_t)100))) >> 6) / ((int32_t)100)) + ((int32_t)(1 << 14)))) >> 10;

	var3 = var1 * var2;
	var4 = (((int32_t)data->par_h6 << 7) + ((temp_scaled * (int32_t)data->par_h7) / ((int32_t)100))) >> 4;
	var5 = ((var3 >> 14) * (var3 >> 14)) >> 10;
	var6 = (var4 * var5) >> 1;

	hum = ((((var3 + var6) >> 10) * ((int32_t)1000)) >> 12);

	if (hum > 100000)
	{
		data->calc_humidity = 100000;
	}
	else if (hum < 0)
	{
		data->calc_humidity = 0;
	}
	else
	{
		data->calc_humidity = hum;
	}
}

static void bme688_calc_gas_resistance(struct bme688_data *data, uint8_t gas_range,
									   uint16_t adc_gas_res)
{
	uint32_t var1 = UINT32_C(262144) >> gas_range;
	int32_t var2 = (int32_t)adc_gas_res - INT32_C(512);
	var2 *= INT32_C(3);
	var2 = INT32_C(4096) + var2;

	uint32_t calc_gas_res = (UINT32_C(10000) * var1) / (uint32_t)var2;
	data->calc_gas_resistance = calc_gas_res * 100;
}

static uint8_t bme688_calc_res_heat(struct bme688_data *data, uint16_t heatr_temp)
{
	int32_t var1, var2, var3, var4, var5;
	int32_t res_heat_x100, res_heat_x;

	if (heatr_temp > 400)
	{
		heatr_temp = 400;
	}

	var1 = (((int32_t)data->calc_temp * data->par_gh3) / 10) << 8;
	var2 = (data->par_gh1 + 784) * (((((data->par_gh2 + 154009) * heatr_temp * 5) / 100) + 3276800) / 10);
	var3 = var1 + (var2 >> 1);
	var4 = (var3 / (data->res_heat_range + 4));
	var5 = (131 * data->res_heat_val) + 65536;
	res_heat_x100 = (int32_t)(((var4 / var5) - 250) * 34);
	res_heat_x = (uint8_t)((res_heat_x100 + 50) / 100);

	return res_heat_x;
}

static int bme688_sample_fetch(const struct device *dev,
							   enum sensor_channel chan)
{
	struct bme688_data *data = dev->data;
	uint8_t buff[3] = {0};
	uint8_t gas_range;
	uint32_t adc_temp, adc_press;
	uint16_t adc_hum, adc_gas_res;
	int ret;

	__ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL);

	bme688_reg_read(dev, 0x1f, buff, 1);
	adc_press = (uint32_t)((uint32_t)buff[0] << 12);
	bme688_reg_read(dev, 0x20, buff, 1);
	adc_press |= ((uint32_t)buff[0] << 4);
	bme688_reg_read(dev, 0x21, buff, 1);
	adc_press |= ((uint32_t)buff[0] >> 4);

	bme688_reg_read(dev, 0x22, buff, 3);
	adc_temp = (uint32_t)(((uint32_t)buff[0] << 12) | ((uint32_t)buff[1] << 4) | ((uint32_t)buff[2] >> 4));

	bme688_reg_read(dev, 0x25, buff, 2);
	adc_hum = (uint16_t)(((uint32_t)buff[0] << 8) | (uint32_t)buff[1]);

	bme688_reg_read(dev, 0x2C, buff, 2);
	adc_gas_res = (uint16_t)((uint32_t)buff[0] << 2 | (((uint32_t)buff[1]) >> 6));

	bme688_reg_read(dev, 0x2D, buff, 1);
	gas_range = buff[0] & BME688_MSK_GAS_RANGE;

	bme688_calc_temp(data, adc_temp);
	bme688_calc_press(data, adc_press);
	bme688_calc_humidity(data, adc_hum);
	bme688_calc_gas_resistance(data, gas_range, adc_gas_res);

	/* Trigger the next measurement */
	ret = bme688_reg_write(dev, BME688_REG_CTRL_MEAS, BME688_CTRL_MEAS_VAL);
	if (ret < 0)
	{
		return ret;
	}

	return 0;
}

static int bme688_channel_get(const struct device *dev,
							  enum sensor_channel chan,
							  struct sensor_value *val)
{
	struct bme688_data *data = dev->data;

	switch (chan)
	{
	case SENSOR_CHAN_AMBIENT_TEMP:
		/*
		 * data->calc_temp has a resolution of 0.01 degC.
		 * So 5123 equals 51.23 degC.
		 */
		val->val1 = data->calc_temp / 100;
		val->val2 = data->calc_temp % 100 * 10000;
		break;
	case SENSOR_CHAN_PRESS:
		/*
		 * data->calc_press has a resolution of 1 Pa.
		 * So 96321 equals 96.321 kPa.
		 */
		val->val1 = data->calc_press / 1000;
		val->val2 = (data->calc_press % 1000) * 1000;
		break;
	case SENSOR_CHAN_HUMIDITY:
		/*
		 * data->calc_humidity has a resolution of 0.001 %RH.
		 * So 46333 equals 46.333 %RH.
		 */
		val->val1 = data->calc_humidity / 1000;
		val->val2 = (data->calc_humidity % 1000) * 1000;
		break;
	case SENSOR_CHAN_GAS_RES:
		/*
		 * data->calc_gas_resistance has a resolution of 1 ohm.
		 * So 100000 equals 100000 ohms.
		 */
		val->val1 = data->calc_gas_resistance;
		val->val2 = 0;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int bme688_read_compensation(const struct device *dev)
{
	struct bme688_data *data = dev->data;
	uint8_t buff[2];

	/* Temperature related coefficients */
	bme688_reg_read(dev, 0xe9, buff, 2);
	data->par_t1 = (uint16_t)(BME688_CONCAT_BYTES(buff[1], buff[0]));
	bme688_reg_read(dev, 0x8a, buff, 2);
	data->par_t2 = (int16_t)(BME688_CONCAT_BYTES(buff[1], buff[0]));
	bme688_reg_read(dev, 0x8c, buff, 1);
	data->par_t3 = (uint8_t)(buff[0]);

	/* Pressure related coefficients */
	bme688_reg_read(dev, 0x8e, buff, 2);
	data->par_p1 = (uint16_t)(BME688_CONCAT_BYTES(buff[1], buff[0]));
	bme688_reg_read(dev, 0x90, buff, 2);
	data->par_p2 = (int16_t)(BME688_CONCAT_BYTES(buff[1], buff[0]));
	bme688_reg_read(dev, 0x92, buff, 1);
	data->par_p3 = (int8_t)buff[0];
	bme688_reg_read(dev, 0x94, buff, 2);
	data->par_p4 = (int16_t)(BME688_CONCAT_BYTES(buff[1], buff[0]));
	bme688_reg_read(dev, 0x96, buff, 2);
	data->par_p5 = (int16_t)(BME688_CONCAT_BYTES(buff[1], buff[0]));
	bme688_reg_read(dev, 0x99, buff, 1);
	data->par_p6 = (int8_t)(buff[0]);
	bme688_reg_read(dev, 0x98, buff, 1);
	data->par_p7 = (int8_t)(buff[0]);
	bme688_reg_read(dev, 0x9c, buff, 2);
	data->par_p8 = (int16_t)(BME688_CONCAT_BYTES(buff[1], buff[0]));
	bme688_reg_read(dev, 0x9e, buff, 2);
	data->par_p9 = (int16_t)(BME688_CONCAT_BYTES(buff[1], buff[0]));
	bme688_reg_read(dev, 0xa0, buff, 2);
	data->par_p10 = (uint8_t)(buff[0]);

	/* Humidity related coefficients */
	bme688_reg_read(dev, 0xe2, buff, 2);
	data->par_h1 = (uint16_t)(((uint16_t)buff[1] << 4) | (buff[0] & 0x0f));
	bme688_reg_read(dev, 0xe2, buff, 2);
	data->par_h2 = (uint16_t)(((uint16_t)buff[0] << 4) | ((buff[1]) >> 4));
	bme688_reg_read(dev, 0xe4, buff, 1);
	data->par_h3 = (int8_t)buff[0];
	bme688_reg_read(dev, 0xe5, buff, 1);
	data->par_h4 = (int8_t)buff[0];
	bme688_reg_read(dev, 0xe6, buff, 1);
	data->par_h5 = (int8_t)buff[0];
	bme688_reg_read(dev, 0xe7, buff, 1);
	data->par_h6 = (uint8_t)buff[0];
	bme688_reg_read(dev, 0xe8, buff, 1);
	data->par_h7 = (int8_t)buff[0];

	/* Gas heater related coefficients */
	bme688_reg_read(dev, 0xed, buff, 1);
	data->par_gh1 = (int8_t)buff[0];
	bme688_reg_read(dev, 0xeb, buff, 2);
	data->par_gh2 = (int16_t)(BME688_CONCAT_BYTES(buff[1], buff[0]));
	bme688_reg_read(dev, 0xee, buff, 1);
	data->par_gh3 = (int8_t)buff[0];

	bme688_reg_read(dev, 0x00, buff, 1);
	data->res_heat_val = (int8_t)buff[0];
	bme688_reg_read(dev, 0x02, buff, 1);
	data->res_heat_range = ((buff[0] & BME688_MSK_RH_RANGE) >> 4);

	return 0;
}

static int bme688_chip_init(const struct device *dev)
{
	struct bme688_data *data = dev->data;
	int err;

	err = bme688_reg_read(dev, BME688_REG_CHIP_ID, &data->chip_id, 1);
	if (err < 0)
	{
		return err;
	}

	if (data->chip_id == BME688_CHIP_ID)
	{
		LOG_DBG("BME688 chip detected");
	}
	else
	{
		LOG_ERR("Bad BME688 chip id 0x%x", data->chip_id);
		return -ENOTSUP;
	}

	err = bme688_read_compensation(dev);
	if (err < 0)
	{
		return err;
	}

	err = bme688_reg_write(dev, BME688_REG_CTRL_HUM, BME688_HUMIDITY_OVER);
	if (err < 0)
	{
		return err;
	}

	err = bme688_reg_write(dev, BME688_REG_CONFIG, BME688_CONFIG_VAL);
	if (err < 0)
	{
		return err;
	}

	err = bme688_reg_write(dev, BME688_REG_CTRL_GAS_1, 0x0b1);
	if (err < 0)
	{
		return err;
	}

	err = bme688_reg_write(dev, BME688_REG_RES_HEAT0,
						   bme688_calc_res_heat(data, BME688_HEATR_TEMP));
	if (err < 0)
	{
		return err;
	}

	err = bme688_reg_write(dev, BME688_REG_GAS_WAIT0, 0x59);
	if (err < 0)
	{
		return err;
	}

	err = bme688_reg_write(dev, BME688_REG_CTRL_MEAS, BME688_CTRL_MEAS_VAL);
	if (err < 0)
	{
		return err;
	}

	return 0;
}

static int bme688_init(const struct device *dev)
{
	const struct bme688_config *config = dev->config;

	if (!device_is_ready(config->bus.bus))
	{
		LOG_ERR("I2C master %s not ready", config->bus.bus->name);
		return -EINVAL;
	}

	if (bme688_chip_init(dev) < 0)
	{
		return -EINVAL;
	}

	return 0;
}

static const struct sensor_driver_api bme688_api_funcs = {
	.sample_fetch = bme688_sample_fetch,
	.channel_get = bme688_channel_get,
};

static struct bme688_data bme688_data;

static const struct bme688_config bme688_config = {
	.bus = I2C_DT_SPEC_INST_GET(0)};

DEVICE_DT_INST_DEFINE(0, bme688_init, NULL, &bme688_data,
					  &bme688_config, POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,
					  &bme688_api_funcs);
