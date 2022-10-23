#include <zephyr.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/logging/log.h>
#include <zephyr/usb/usb_device.h>

#include "zigbee.hpp"
#include "sensors.hpp"
#include "controllers.hpp"
#include "classifier.hpp"
#include "config/hardware.hpp"
#include "config/software.hpp"

LOG_MODULE_REGISTER(main, MAIN_LOG_LEVEL);

namespace
{
	using DigOut = app::io::digital::Output;
	using AnOut = app::io::analog::PWM;
	using AnIn = app::io::analog::ADC;
	using ClosedController = app::io::controllers::ClosedController;
	using SensorResult = app::networking::zigbee::SensorResult;
	using EnvSensor = app::sensors::environmental::BME688;
	using Zigbee = app::networking::zigbee::ZigbeeInterface;
	using Classifier = app::inference::classifiers::Classifier;

	/**
	 * @brief Thread function to control growth lights.
	 * Control takes place only if ClosedControlStrategy == SELF.
	 */
	void lightCtrlThread();

	/**
	 * @brief Thread function to read sensors and publish them to Zigbee network.
	 */
	SensorResult sensorReadCallback();

	/**
	 * @brief Set control strategy for growth lights.
	 * Strat should be castable to ClosedControlStrategy.
	 */
	void lightStrategyCallback(uint16_t strat);

	/**
	 * @brief Set forced state of given ClosedController.
	 * This state will take effect only when ClosedControlStrategy is set to ZIGBEE.
	 *
	 * @tparam &controller
	 * @param val Value to set PWM output to (0 - 1023).
	 */
	template <ClosedController &controller>
	void lightControlCallback(uint16_t val);

	/**
	 * @brief Set state of given output.
	 *
	 * @tparam &actuator Output to use.
	 * @param val 0 - off, on otherwise.
	 */
	template <DigOut &actuator>
	void OutputControlCallback(uint16_t val);

	/**
	 * @brief Initialize all used hardware.
	 *
	 * @return True on success.
	 */
	bool initHardware();

	/**
	 * @brief Check whether all app's components are in good state.
	 */
	bool hardwareGood();

	/**
	 * @brief Signalize erroneous behavior to the user and reset the device.
	 */
	void handleCriticalError();
}

//****************
// Implementation.
//****************

namespace
{
	constexpr const gpio_dt_spec errPin = GPIO_DT_SPEC_GET(ERROR_PIN, gpios);
	constexpr const gpio_dt_spec blinkPin = GPIO_DT_SPEC_GET(BLINK_PIN, gpios);
	constexpr const gpio_dt_spec waterPin = GPIO_DT_SPEC_GET(WATER_PIN, gpios);
	constexpr const gpio_dt_spec nutriPin = GPIO_DT_SPEC_GET(NUTRI_PIN, gpios);
	constexpr const gpio_dt_spec fanPin = GPIO_DT_SPEC_GET(FAN_PIN, gpios);

	EnvSensor envSensor(DEVICE_DT_GET(SENSOR_DEVICE));
	Classifier classifier;

	constexpr Zigbee &zigbee = Zigbee::getInterface();

	DigOut errLed = DigOut(errPin, "Error LED");
	DigOut blinkLed = DigOut(blinkPin, "Blink LED");

	DigOut relayWater = DigOut(waterPin, "Water Valve");
	DigOut relayNutri = DigOut(nutriPin, "Nutri Valve");
	DigOut fan = DigOut(fanPin, "Air Fan");

	constexpr const device *ledDevice = DEVICE_DT_GET(LED_PWM);
	constexpr const device *ledSensorDevice = DEVICE_DT_GET(LED_ADC);

	ClosedController led1 = ClosedController(*ledDevice, LED1_CHANNEL,
											 *ledSensorDevice,
											 DT_IO_CHANNELS_INPUT_BY_IDX(DT_PATH(zephyr_user), LED1_ADC_CHANNEL));
	ClosedController led2 = ClosedController(*ledDevice, LED2_CHANNEL,
											 *ledSensorDevice,
											 DT_IO_CHANNELS_INPUT_BY_IDX(DT_PATH(zephyr_user), LED2_ADC_CHANNEL));

	K_THREAD_DEFINE(lightThreadId, LIGHT_CTRL_THREAD_STACK_SIZE,
					lightCtrlThread, NULL, NULL, NULL,
					1, 0, 10000);
}

#ifdef DATASET_COLLECTOR
int main()
{
	if (usb_enable(NULL))
	{
		LOG_ERR("Failed to initialize USB");
		handleCriticalError();
	}

	LOG_INF("Application core starting");

	k_sleep(K_SECONDS(5));

	if (!initHardware())
	{
		LOG_ERR("Hardware initialization failed");
		handleCriticalError();
	}

	while (hardwareGood())
	{
		k_sleep(K_MSEC(100));
		SensorResult res = sensorReadCallback();
		printk("%.3f, %.3f, %.3f, %.3f\n", res.temperature, res.pressure, res.humidity, res.gasResistance / 10000.0);
	}

	handleCriticalError();

	return 0;
}
#else
int main()
{
	if (usb_enable(NULL))
	{
		LOG_ERR("Failed to initialize USB");
		handleCriticalError();
	}

	LOG_INF("Application core starting");

	k_sleep(K_SECONDS(5));

	using ZigbeeCallbackType = app::networking::zigbee::CallbackType::Values;
	zigbee.registerCallback(ZigbeeCallbackType::FAN, OutputControlCallback<fan>);
	zigbee.registerCallback(ZigbeeCallbackType::RELAY_WATER, OutputControlCallback<relayWater>);
	zigbee.registerCallback(ZigbeeCallbackType::RELAY_NUTRI, OutputControlCallback<relayNutri>);
	zigbee.registerCallback(ZigbeeCallbackType::LED_SIDE_1, lightControlCallback<led1>);
	zigbee.registerCallback(ZigbeeCallbackType::LED_SIDE_2, lightControlCallback<led2>);
	zigbee.registerCallback(ZigbeeCallbackType::LED_STRATEGY, lightStrategyCallback);
	zigbee.registerSensorCallback(sensorReadCallback);

	if (settings_subsys_init())
	{
		LOG_ERR("Settings initialization failed");
		handleCriticalError();
	}

	if (!initHardware())
	{
		LOG_ERR("Hardware initialization failed");
		handleCriticalError();
	}

	classifier.init();

	if (settings_load())
	{
		LOG_ERR("Settings load failed");
		handleCriticalError();
	}

	zigbee.doWorkDetached();

	while (hardwareGood())
	{
		k_sleep(K_SECONDS(5));
		LOG_DBG("Blinking");

		if (!zigbee.isIdentifying())
		{
			blinkLed.setState(1);
			k_sleep(K_MSEC(100));
			blinkLed.setState(0);
		}
	}
	handleCriticalError();
	return 0;
}
#endif

namespace
{
	void lightCtrlThread()
	{
		while (hardwareGood())
		{
			if (led1.getControlStrategy() == ClosedController::ControlStrategy::SELF)
			{
				LOG_DBG("Updating growth light intensity");
				led1.update();
				led2.update();
				LOG_DBG("Growth light control updated");
			}
			else
			{
				LOG_DBG("Growth lights are in ZIGBEE state - skipping update ");
			}
			k_sleep(K_MSEC(100));
		}
		LOG_ERR("Hardware failure detected - stopping light control thread");
	}

	SensorResult sensorReadCallback()
	{
		SensorResult res;

		LOG_INF("Performing sensor read");
		if (!envSensor.read())
		{
			LOG_ERR("Failed to read environmental sensor");
			res.ok = false;
			return res;
		}

		res.temperature = envSensor.getTemperature();
		res.humidity = envSensor.getHumidity();
		res.pressure = envSensor.getPressure();
		res.gasResistance = envSensor.getGasResistance();
		res.classification = static_cast<uint8_t>(classifier.readNetwork(res.temperature, res.pressure, res.humidity, res.gasResistance));

		if (hardwareGood())
		{
			res.ok = true;
			LOG_INF("Sensor read done");
		}
		else
		{
			LOG_ERR("Hardware failure detected - skipping sensor read");
			res.ok = false;
		}

		return res;
	}

	void lightStrategyCallback(uint16_t strat)
	{
		if (strat > 1)
		{
			LOG_WRN("Invalid control strategy - ignoring");
			return;
		}
		bool stratBool = strat;
		auto stratEnum = static_cast<ClosedController::ControlStrategy>(stratBool);

		led1.setControlStrategy(stratEnum);
		led2.setControlStrategy(stratEnum);

		if (stratEnum == ClosedController::ControlStrategy::SELF)
		{
			LOG_INF("Growth light control set to SELF");
		}
		else
		{
			LOG_INF("Growth light control set to ZIGBEE");
		}
	}

	template <ClosedController &controller>
	void lightControlCallback(uint16_t val)
	{
		if (val > 1023)
		{
			LOG_WRN("Value %d passed to %s is out of allowed bounds [0 - 1023] - ignoring", val, "TODO");
			return;
		}
		LOG_INF("Setpoint for %s set to %d", "TODO", val);
		controller.setState(val);
	}

	template <DigOut &actuator>
	void OutputControlCallback(uint16_t val)
	{
		LOG_INF("Value of %s set to %d", actuator.getLabel(), val);
		actuator.setState(val);
	}

	bool initHardware()
	{
		LOG_INF("Initializing hardware");
		if (!errLed.init())
		{
			LOG_ERR("Ironic - error LED init failed");
			return false;
		}
		if (!blinkLed.init())
		{
			LOG_ERR("Blink LED init failed");
			return false;
		}
		if (!led1.init())
		{
			LOG_ERR("Growth LED 1 init failed");
			return false;
		}
		if (!led2.init())
		{
			LOG_ERR("Growth LED 2 init failed");
			return false;
		}
		if (!relayWater.init())
		{
			LOG_ERR("Water relay init failed");
			return false;
		}
		if (!relayNutri.init())
		{
			LOG_ERR("Nutri relay init failed");
			return false;
		}
		if (!fan.init())
		{
			LOG_ERR("Fan init failed");
			return false;
		}
		if (!envSensor.init())
		{
			LOG_ERR("Environmental sensor init failed");
			return false;
		}
		if (!zigbee.init())
		{
			LOG_ERR("Zigbee init failed");
			return false;
		}
		LOG_INF("Hardware initialized properly");
		return true;
	}

	bool hardwareGood()
	{
		LOG_DBG("Checking status of application's hardware");
		if (!errLed.ok())
		{
			LOG_ERR("Ironic - error LED failed");
			return false;
		}
		if (!blinkLed.ok())
		{
			LOG_ERR("Blink LED failed");
			return false;
		}
		if (!led1.ok())
		{
			LOG_ERR("Growth LED 1 failed");
			return false;
		}
		if (!led2.ok())
		{
			LOG_ERR("Growth LED 2 failed");
			return false;
		}
		if (!relayWater.ok())
		{
			LOG_ERR("Water relay failed");
			return false;
		}
		if (!relayNutri.ok())
		{
			LOG_ERR("Nutri relay failed");
			return false;
		}
		if (!fan.ok())
		{
			LOG_ERR("Fan failed");
			return false;
		}
		if (!envSensor.ok())
		{
			LOG_ERR("Environmental sensor failed");
			return false;
		}
		if (!zigbee.ok())
		{
			LOG_ERR("Zigbee failed");
			return false;
		}
		LOG_DBG("Hardware good");
		return true;
	}

	void handleCriticalError()
	{
		if (blinkLed.ok())
		{
			blinkLed.setState(0);
		}
		if (zigbee.ok())
		{
			zigbee.cleanup();
		}
		if (errLed.ok())
		{
			for (uint8_t i = 0; i < 10; i++)
			{
				k_sleep(K_MSEC(200));
				errLed.setState(1);
				k_sleep(K_MSEC(200));
				errLed.setState(0);
			}
		}
		LOG_ERR("System error encountered, performing hard reboot");

		k_sleep(K_SECONDS(1));
		sys_reboot(SYS_REBOOT_WARM);
	}
}
