#include <zephyr.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>

#include "zigbee.hpp"
#include "controllers.hpp"

LOG_MODULE_REGISTER(app, LOG_LEVEL_DBG);

namespace
{
	using DigOut = app::io::digital::Output;
	using ClosedController = app::io::controllers::ClosedController;
	using Zigbee = app::networking::zigbee::ZigbeeInterface;

	Zigbee &zigbee = Zigbee::getInterface();

	DigOut relayWater = DigOut();
	DigOut relayNutri = DigOut();

	ClosedController led1 = ClosedController();
	ClosedController led2 = ClosedController();

	DigOut fan = DigOut();
}

/**
 * @brief Control LEDs via light sensors.
 *
 */
void lightCtrlThread()
{
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
	bool stratBool = strat;
	auto stratEnum = static_cast<ClosedController::ControlStrategy>(stratBool);

	led1.setControlStrategy(stratEnum);
	led2.setControlStrategy(stratEnum);
}

template <ClosedController &controller>
void closedControlCallback(uint16_t val)
{
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
	actuator.setState(val);
}

/**
 * @brief Initialize all app's hardware.
 *
 * @return True on success.
 */
bool initHardware()
{
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
	if (!zigbee.init())
	{
		LOG_ERR("Zigbee init failed");
		return false;
	}
	return true;
}

/**
 * @brief Check whether all app's components are in working state.
 */
bool systemGood()
{
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
	if (!zigbee.ok())
	{
		LOG_ERR("Zigbee failed");
		return false;
	}
	return true;
}

/**
 * @brief Signalize erroneous behavior to the user.
 */
void signalError()
{
}

/**
 * @brief Cleanup the app.
 */
void cleanup()
{
}

void main(void)
{
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
		goto on_error;
	}

	if (settings_subsys_init())
	{
		LOG_ERR("Settings initialization failed");
		goto on_error;
	}

	if (settings_load())
	{
		LOG_ERR("Settings loading failed");
		goto on_error;
	}

	zigbee.doWorkDetached();

	while (systemGood())
	{
		k_sleep(K_SECONDS(1));
	}

on_error:
	signalError();
	cleanup();
	k_sleep(K_SECONDS(5));
	// TODO: reset.
}
