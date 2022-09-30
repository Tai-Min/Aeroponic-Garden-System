#include <zephyr.h>
#include <zephyr/settings/settings.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/logging/log.h>
#include <zephyr/usb/usb_device.h>

#include "zigbee.hpp"
#include "controllers.hpp"
#include "config/hardware.hpp"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

namespace
{
	using DigOut = app::io::digital::Output;
	using ClosedController = app::io::controllers::ClosedController;
	using Zigbee = app::networking::zigbee::ZigbeeInterface;

	constexpr struct gpio_dt_spec blinkPin = GPIO_DT_SPEC_GET(BLINK_PIN, gpios);
	constexpr struct gpio_dt_spec waterPin = GPIO_DT_SPEC_GET(WATER_PIN, gpios);
	constexpr struct gpio_dt_spec nutriPin = GPIO_DT_SPEC_GET(NUTRI_PIN, gpios);
	constexpr struct gpio_dt_spec fanPin = GPIO_DT_SPEC_GET(FAN_PIN, gpios);

	Zigbee &zigbee = Zigbee::getInterface();

	DigOut blinkLed = DigOut();

	DigOut relayWater = DigOut();
	DigOut relayNutri = DigOut();
	DigOut fan = DigOut();

	ClosedController led1 = ClosedController();
	ClosedController led2 = ClosedController();
}

/**
 * @brief Control LEDs via light sensors.
 *
 */
void lightCtrlThread()
{
	// LOG_INFO();

	led1.update();
	led2.update();
	k_sleep(K_SECONDS(30));
}

void sensorReadThread()
{
	LOG_INF("Sensors read");
	k_sleep(K_SECONDS(60));
}

void lightStrategyCallback(uint16_t strat)
{
	// LOG_INFO();

	bool stratBool = strat;
	auto stratEnum = static_cast<ClosedController::ControlStrategy>(stratBool);

	led1.setControlStrategy(stratEnum);
	led2.setControlStrategy(stratEnum);
}

template <ClosedController &controller>
void closedControlCallback(uint16_t val)
{
	// LOG_INFO();
	controller.setState(val);
}

/**
 * @brief
 *
 * @tparam &actuator
 * @param val
 */
template <DigOut &actuator>
void relayControlCallback(uint16_t val)
{
	// LOG_INFO();
	actuator.setState(val);
}

/**
 * @brief Initialize all app's hardware.
 *
 * @return True on success.
 */
bool initHardware()
{
	if (!blinkLed.init(blinkPin))
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
	if (!relayWater.init(waterPin))
	{
		LOG_ERR("Water relay init failed");
		return false;
	}
	if (!relayNutri.init(waterPin))
	{
		LOG_ERR("Nutri relay init failed");
		return false;
	}
	if (!fan.init(waterPin))
	{
		LOG_ERR("Fan init failed");
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

/**
 * @brief Check whether all app's components are in working state.
 */
bool systemGood()
{
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
	if (!zigbee.ok())
	{
		LOG_ERR("Zigbee failed");
		return false;
	}
	LOG_DBG("System good");
	return true;
}

/**
 * @brief Signalize erroneous behavior to the user.
 */
void handleCriticalError()
{
	if (blinkLed.ok())
	{
		for (uint8_t i = 0; i < 10; i++)
		{
			k_sleep(K_MSEC(200));
			blinkLed.setState(1);
			k_sleep(K_MSEC(200));
			blinkLed.setState(0);
		}
	}
	LOG_ERR("System error encountered, performing hard reboot");

	k_sleep(K_SECONDS(1));
	sys_reboot(SYS_REBOOT_WARM);
}

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
	zigbee.registerCallback(ZigbeeCallbackType::FAN, relayControlCallback<fan>);
	zigbee.registerCallback(ZigbeeCallbackType::RELAY_WATER, relayControlCallback<relayWater>);
	zigbee.registerCallback(ZigbeeCallbackType::RELAY_NUTRI, relayControlCallback<relayNutri>);
	zigbee.registerCallback(ZigbeeCallbackType::LED_SIDE_1, closedControlCallback<led1>);
	zigbee.registerCallback(ZigbeeCallbackType::LED_SIDE_2, closedControlCallback<led2>);
	zigbee.registerCallback(ZigbeeCallbackType::LED_STRATEGY, lightStrategyCallback);

	if (!initHardware())
	{
		LOG_ERR("Hardware initialization failed");
		handleCriticalError();
	}

	if (settings_subsys_init())
	{
		LOG_ERR("Settings initialization failed");
		handleCriticalError();
	}

	if (settings_load())
	{
		LOG_ERR("Settings loading failed");
		handleCriticalError();
	}

	zigbee.doWorkDetached();

	while (systemGood())
	{
		k_sleep(K_SECONDS(5));
		LOG_DBG("Blinking");
		blinkLed.setState(1);
		k_sleep(K_MSEC(100));
		blinkLed.setState(0);
	}
	handleCriticalError();
	return 0; // Won't reach hopefully.
}
