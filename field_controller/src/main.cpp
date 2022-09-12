#include <zephyr.h>
#include "zigbee.hpp"
#include "controllers.hpp"

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

	ClosedController heater = ClosedController();
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
	k_sleep(K_SECONDS(60));
}

void heaterStrategyCallback(uint16_t strat)
{
	bool stratBool = strat;
	auto stratEnum = static_cast<ClosedController::ControlStrategy>(stratBool);

	heater.setControlStrategy(stratEnum);
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

template <DigOut &actuator>
void relayControlCallback(uint16_t val)
{
	actuator.setState(val);
}

bool initHardware()
{
	if (!led1.init())
	{
		return false;
	}
	if (!led2.init())
	{
		return false;
	}
	if (!relayWater.init())
	{
		return false;
	}
	if (!relayNutri.init())
	{
		return false;
	}
	if (!zigbee.init())
	{
		return false;
	}
	return true;
}

bool systemGood()
{
	if (!led1.ok())
	{
		return false;
	}
	if (!led2.ok())
	{
		return false;
	}
	if (!relayWater.ok())
	{
		return false;
	}
	if (!relayNutri.ok())
	{
		return false;
	}
	if (!zigbee.ok())
	{
		return false;
	}
	return true;
}

void main(void)
{
	using ZigbeeCallbackType = app::networking::zigbee::CallbackType::Values;
	zigbee.registerCallback(ZigbeeCallbackType::LED_SIDE_2, closedControlCallback<heater>);
	zigbee.registerCallback(ZigbeeCallbackType::LED_SIDE_1, closedControlCallback<led1>);
	zigbee.registerCallback(ZigbeeCallbackType::LED_SIDE_2, closedControlCallback<led2>);
	zigbee.registerCallback(ZigbeeCallbackType::RELAY_WATER, relayControlCallback<relayWater>);
	zigbee.registerCallback(ZigbeeCallbackType::RELAY_NUTRI, relayControlCallback<relayNutri>);

	if (!initHardware())
	{
		return;
	}

	zigbee.doWorkDetached();

	while (systemGood())
	{
		k_sleep(K_SECONDS(1));
	}
}
