#ifndef YOUR_CODE_H_
#define YOUR_CODE_H_

#include <stdbool.h>
#include <stdint.h>

#include <math.h>

#include "FreeRTOS.h"
#include "task.h"

#include "system.h"
#include "log.h"
#include "param.h"
//#include "debug.h"
#include "motors.h"
#include "pm.h"

#include "sensors.h"
#include "commander.h"
#include "crtp_commander_high_level.h"
#include "crtp_localization_service.h"
#include "controller.h"
#include "power_distribution.h"
#include "collision_avoidance.h"
#include "health.h"
#include "supervisor.h"

#include "estimator.h"
#include "usddeck.h"
#include "quatcompress.h"
#include "statsCnt.h"
#include "static_mem.h"
#include "rateSupervisor.h"

#define EMERGENCY_STOP_TIMEOUT_DISABLED (-1)

void yourCodeInit(void);

/**
 * Enable emergency stop, will shut-off energy to the motors.
 */
void stabilizerSetEmergencyStop();

/**
 * Disable emergency stop, will enable energy to the motors.
 */
void stabilizerResetEmergencyStop();

/**
 * Restart the countdown until emergercy stop will be enabled.
 *
 * @param timeout Timeout in stabilizer loop tick. The stabilizer loop rate is
 *                RATE_MAIN_LOOP.
 */
void stabilizerSetEmergencyStopTimeout(int timeout);

#endif /* YOUR_CODE_H_ */
