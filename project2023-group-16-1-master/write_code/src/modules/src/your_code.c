#include "your_code.h"

#define RAD2DEG (180.0 / 3.14159265358979323846)
#define DEG2RAD (3.14159265358979323846 / 180.0)
#define PWM_CONST (0.06 * 9.81 / 65536.0 / 4.0)

/***
 *
 * This file is where you should add you tasks. You already know the structure
 * Required to do so from the work with the simulator.
 *
 * The function yourCodeInit() is set to automatically execute when the
 * quadrotor is started. This is where you need to create your tasks. The
 * scheduler that runs the tasks is already up and running so you should
 * NOT make a call to vTaskStartScheduler();.
 *
 * Below that you can find a few examples of useful function calls and code snippets.
 *
 * For further reference on how this is done: Look into the file stabilizer.c
 * which usually handles the control of the crazyflie.
 *
 ***/

// NO NEED TO CHANGE THIS
static bool emergencyStop = false;
static int emergencyStopTimeout = EMERGENCY_STOP_TIMEOUT_DISABLED;
static StateEstimatorType estimatorType;
// END NO NEED TO CHANGE THIS
static SemaphoreHandle_t motors_sem;
static SemaphoreHandle_t references_sem;
static SemaphoreHandle_t sensors_sem;
static SemaphoreHandle_t estimate_sem;

sensorData_t sensorData;
state_t state;
setpoint_t setpoint;

typedef struct error_float_s
{
  float e_roll;
  float e_droll;
  float e_pitch;
  float e_dpitch;
  float e_dyaw;
} error_float_t;
error_float_t error_float;
typedef struct motor_output_s
{
  uint32_t m1;
  uint32_t m2;
  uint32_t m3;
  uint32_t m4;
} motor_output_t;
motor_output_t motor_output;

STATIC_MEM_TASK_ALLOC(filterTask, configMINIMAL_STACK_SIZE);
void filterTask(void *pvParameters);
STATIC_MEM_TASK_ALLOC(controlSystemTask, configMINIMAL_STACK_SIZE);
void controlSystemTask(void *pvParameters);

void yourCodeInit(void)
{
  estimatorType = getStateEstimator();

  /*
   * CREATE AND EXECUTE YOUR TASKS FROM HERE
   */
  motors_sem = xSemaphoreCreateBinary();
  xSemaphoreGive(motors_sem);
  references_sem = xSemaphoreCreateBinary();
  xSemaphoreGive(references_sem);
  sensors_sem = xSemaphoreCreateBinary();
  xSemaphoreGive(sensors_sem);
  estimate_sem = xSemaphoreCreateBinary();
  xSemaphoreGive(estimate_sem);

  STATIC_MEM_TASK_CREATE(filterTask, filterTask, "FILTER_TASK", NULL, 1);
  STATIC_MEM_TASK_CREATE(controlSystemTask, controlSystemTask, "CONTROL_SYSTEM_TASK", NULL, 2);
}
/*
 * ADD TASKS HERE
 */
void filterTask(void *pvParameters)
{
  TickType_t lastWakeTime = xTaskGetTickCount();
  while (!sensorsAreCalibrated())
  {
    vTaskDelayUntil(&lastWakeTime, F2T(RATE_MAIN_LOOP));
  }
  uint32_t tick = 1;

  // local vars
  double acc_data[3], gyro_data[3], acc_angle[3];
  double gamma = 0.99; // just enough to remove drift
  double h = 1.0 / 250;
  double estimate[3] = {0.0};

  while (1)
  {
    // read sensor data
    xSemaphoreTake(sensors_sem, portMAX_DELAY);
    sensorsAcquire(&sensorData, tick);
    acc_data[0] = (double)sensorData.acc.x;
    acc_data[1] = (double)sensorData.acc.y;
    acc_data[2] = (double)sensorData.acc.z;
    gyro_data[0] = (double)sensorData.gyro.x;
    gyro_data[1] = (double)sensorData.gyro.y;
    gyro_data[2] = (double)sensorData.gyro.z;
    xSemaphoreGive(sensors_sem);

    // apply filter
    acc_angle[0] = RAD2DEG * atan2(acc_data[1], acc_data[2]);
    acc_angle[1] = RAD2DEG * atan2(-acc_data[0], sqrt(pow(acc_data[1], 2) + pow(acc_data[2], 2)));

    for (int i = 0; i < 3; i++)
    {
      estimate[i] = (1 - gamma) * acc_angle[i] + gamma * (estimate[i] + h * gyro_data[i]);
    }

    // write estimates output
    xSemaphoreTake(estimate_sem, portMAX_DELAY);
    state.attitude.roll = (float)estimate[0];
    state.attitude.pitch = (float)estimate[1];
    state.attitude.yaw = (float)estimate[2];
    xSemaphoreGive(estimate_sem);

    vTaskDelayUntil(&lastWakeTime, F2T(RATE_250_HZ));
  }
}

void controlSystemTask(void *pvParameters)
{
  TickType_t lastWakeTime = xTaskGetTickCount();
  while (!sensorsAreCalibrated())
  {
    vTaskDelayUntil(&lastWakeTime, F2T(RATE_MAIN_LOOP));
  }
  uint32_t tick = 1;

  // local vars
  double gyro_data[3], r_rpdy[3];
  unsigned short motors[4];
  double estimate[3] = {0.0};
  double error[5] = {0.0};
  // double gain[4][5] = {
  //     {-0.240498254993081, -0.010010849697055, -0.356389954344025, -0.014836037608414, -0.057607712247082},
  //     {-0.240498254993080, -0.010010849697055, 0.356389954344011, 0.014836037608413, 0.057607712247082},
  //     {0.240498254993084, 0.010010849697057, 0.356389954343960, 0.014836037608413, -0.057607712247082},
  //     {0.240498254993077, 0.010010849697057, -0.356389954343947, -0.014836037608413, 0.057607712247082}}; // ModelForCrazyfile.m, Q_x = diag([1000 1 1000 1 1]), Q_u = eye(4)
  // double gain[4][5] = {
  //     {-0.0838825629549667, -0.00922732989120921, -0.124303276208609, -0.0136741675775943, -0.0576077122470818},
  //     {-0.0838825629549611, -0.00922732989120912, 0.124303276208612, 0.0136741675775948, 0.0576077122470825},
  //     {0.0838825629549692, 0.00922732989121019, 0.124303276208623, 0.0136741675775948, -0.0576077122470818},
  //     {0.0838825629549663, 0.00922732989121028, -0.124303276208617, -0.0136741675775943, 0.0576077122470826}}; // ModelForCrazyfile.m, Q_x = diag([100 1 100 1 1]), Q_u = eye(4)
  // double gain[4][5] = {
  //     {-0.083908280773827, -0.009229913366403, -0.124386995156545, -0.013682577550042, -0.004790477521638},
  //     {-0.083908280775013, -0.009229913366404, 0.124386995156571, 0.013682577550042, 0.004790477521638},
  //     {0.083908280774983, 0.009229913366398, 0.124386995151557, 0.013682577550011, -0.004790477521638},
  //     {0.083908280773706, 0.009229913366397, -0.124386995151535, -0.013682577550011, 0.004790477521638}}; // linearization.m, Q = diag([1e4 1e2 1e4 1e2 1e-4]), R = eye(4)
  double gain[4][5] = {
      {-0.083882562954953, -0.009227329891210, -0.124303276208566, -0.013674167577594, -0.001559873339462},
      {-0.083882562954979, -0.009227329891210, 0.124303276208564, 0.013674167577595, 0.001559873339462},
      {0.083882562954980, 0.009227329891210, 0.124303276208647, 0.013674167577595, -0.001559873339462},
      {0.083882562954949, 0.009227329891210, -0.124303276208652, -0.013674167577594, 0.001559873339462}}; // linearization.m, Q = diag([1e3 1e1 1e3 1e1 1e-4]), R = eye(4) .* 1e1
  unsigned short base_thrust = 45000;

  // LOG_GROUP_START(estimate)
  // LOG_ADD(LOG_FLOAT, roll, &state.attitude.roll)
  // LOG_ADD(LOG_FLOAT, pitch, &state.attitude.pitch)
  // LOG_ADD(LOG_FLOAT, yaw, &state.attitude.yaw)
  // LOG_GROUP_STOP(estimate)

  LOG_GROUP_START(state_error)
  LOG_ADD(LOG_FLOAT, e_roll, &error_float.e_roll)
  LOG_ADD(LOG_FLOAT, e_droll, &error_float.e_droll)
  LOG_ADD(LOG_FLOAT, e_pitch, &error_float.e_pitch)
  LOG_ADD(LOG_FLOAT, e_dpitch, &error_float.e_dpitch)
  LOG_ADD(LOG_FLOAT, e_dyaw, &error_float.e_dyaw)
  LOG_GROUP_STOP(state_error)

  LOG_GROUP_START(motor_output)
  LOG_ADD(LOG_UINT16, M1, &motor_output.m1)
  LOG_ADD(LOG_UINT16, M2, &motor_output.m2)
  LOG_ADD(LOG_UINT16, M3, &motor_output.m3)
  LOG_ADD(LOG_UINT16, M4, &motor_output.m4)
  LOG_GROUP_STOP(motor_output)

  while (1)
  {
    // read sensor data (gyro)
    xSemaphoreTake(sensors_sem, portMAX_DELAY);
    sensorsAcquire(&sensorData, tick);
    gyro_data[0] = (double)sensorData.gyro.x;
    gyro_data[1] = (double)sensorData.gyro.y;
    gyro_data[2] = (double)sensorData.gyro.z;
    xSemaphoreGive(sensors_sem);

    // read filter data (angle estimates)
    xSemaphoreTake(estimate_sem, portMAX_DELAY);
    stateEstimator(&state, tick);
    estimate[0] = (double)state.attitude.roll;
    estimate[1] = (double)state.attitude.pitch;
    estimate[2] = (double)state.attitude.yaw;
    xSemaphoreGive(estimate_sem);

    // read latest references
    xSemaphoreTake(references_sem, portMAX_DELAY);
    commanderGetSetpoint(&setpoint, &state);
    r_rpdy[0] = (double)setpoint.attitude.roll;
    r_rpdy[1] = (double)setpoint.attitude.pitch;
    r_rpdy[2] = (double)setpoint.attitudeRate.yaw;
    xSemaphoreGive(references_sem);

    // compute error
    error[0] = DEG2RAD * (r_rpdy[0] - estimate[0]);
    error[1] = DEG2RAD * (0.0 - gyro_data[0]);
    error[2] = DEG2RAD * (r_rpdy[1] - estimate[1]);
    error[3] = DEG2RAD * (0.0 - gyro_data[1]);
    error[4] = DEG2RAD * (r_rpdy[2] - gyro_data[2]);

    // log error
    error_float.e_roll = (float)(RAD2DEG * error[0]);
    error_float.e_droll = (float)(RAD2DEG * error[1]);
    error_float.e_pitch = (float)(RAD2DEG * error[2]);
    error_float.e_dpitch = (float)(RAD2DEG * error[3]);
    error_float.e_dyaw = (float)(RAD2DEG * error[4]);

    // compute motor outputs without base thrust (debug: checking for overflow)
    motors[0] = (unsigned short)((1.0 / PWM_CONST) * (error[0] * gain[0][0] + error[1] * gain[0][1] + error[2] * gain[0][2] + error[3] * gain[0][3] + error[4] * gain[0][4]));
    motors[1] = (unsigned short)((1.0 / PWM_CONST) * (error[0] * gain[1][0] + error[1] * gain[1][1] + error[2] * gain[1][2] + error[3] * gain[1][3] + error[4] * gain[1][4]));
    motors[2] = (unsigned short)((1.0 / PWM_CONST) * (error[0] * gain[2][0] + error[1] * gain[2][1] + error[2] * gain[2][2] + error[3] * gain[2][3] + error[4] * gain[2][4]));
    motors[3] = (unsigned short)((1.0 / PWM_CONST) * (error[0] * gain[3][0] + error[1] * gain[3][1] + error[2] * gain[3][2] + error[3] * gain[3][3] + error[4] * gain[3][4]));

    // adding base thrust with saturation
    motors[0] = (base_thrust + motors[0] >= 65535) ? 65535 : base_thrust + motors[0];
    motors[1] = (base_thrust + motors[1] >= 65535) ? 65535 : base_thrust + motors[1];
    motors[2] = (base_thrust + motors[2] >= 65535) ? 65535 : base_thrust + motors[2];
    motors[3] = (base_thrust + motors[3] >= 65535) ? 65535 : base_thrust + motors[3];

    // log motor output
    motor_output.m1 = motors[0];
    motor_output.m2 = motors[1];
    motor_output.m3 = motors[2];
    motor_output.m4 = motors[3];

    // write motor output
    xSemaphoreTake(motors_sem, portMAX_DELAY);
    motorsSetRatio(MOTOR_M1, motors[0]);
    motorsSetRatio(MOTOR_M2, motors[1]);
    motorsSetRatio(MOTOR_M3, motors[2]);
    motorsSetRatio(MOTOR_M4, motors[3]);
    xSemaphoreGive(motors_sem);

    vTaskDelayUntil(&lastWakeTime, F2T(RATE_250_HZ));
  }
}

void stabilizerSetEmergencyStop()
{
  emergencyStop = true;
}

void stabilizerResetEmergencyStop()
{
  emergencyStop = false;
}

void stabilizerSetEmergencyStopTimeout(int timeout)
{
  emergencyStop = false;
  emergencyStopTimeout = timeout;
}

/*************************************************
 * CREATE A TASK (Example, a task with priority 5)
 ************************************************/
// STATIC_MEM_TASK_ALLOC(yourTaskName, configMINIMAL_STACK_SIZE);
// STATIC_MEM_TASK_CREATE(yourTaskName, yourTaskName,
//    "TASK_IDENTIFIER_NAME", NULL, 5);

/*************************************************
 * WAIT FOR SENSORS TO BE CALIBRATED
 ************************************************/
// lastWakeTime = xTaskGetTickCount ();
// while(!sensorsAreCalibrated()) {
//     vTaskDelayUntil(&lastWakeTime, F2T(RATE_MAIN_LOOP));
// }

/*************************************************
 * RETRIEVE THE MOST RECENT SENSOR DATA
 *
 * The code creates a variable called sensorData and then calls a function
 * that fills this variable with the latest data from the sensors.
 *
 * sensorData_t sensorData = struct {
 *     Axis3f acc;
 *     Axis3f gyro;
 *     Axis3f mag;
 *     baro_t baro;
 *     zDistance_t zrange;
 *     point_t position;
 * }
 *
 * Before starting the loop, initialize tick:
 * uint32_t tick;
 * tick = 1;
 ************************************************/
// sensorData_t sensorData;
// sensorsAcquire(&sensorData, tick);

/*************************************************
 * RETRIEVE THE SET POINT FROM ANY EXTERNAL COMMAND INTERFACE
 *
 * The code creates a variable called setpoint and then calls a function
 * that fills this variable with the latest command input.
 *
 * setpoint_t setpoint = struct {
 *     uint32_t timestamp;
 *
 *     attitude_t attitude;      // deg
 *     attitude_t attitudeRate;  // deg/s
 *     quaternion_t attitudeQuaternion;
 *     float thrust;
 *     point_t position;         // m
 *     velocity_t velocity;      // m/s
 *     acc_t acceleration;       // m/s^2
 *     bool velocity_body;       // true if velocity is given in body frame; false if velocity is given in world frame
 *
 *     struct {
 *         stab_mode_t x;
 *         stab_mode_t y;
 *         stab_mode_t z;
 *         stab_mode_t roll;
 *         stab_mode_t pitch;
 *         stab_mode_t yaw;
 *         stab_mode_t quat;
 *     } mode;
 * }
 *
 ************************************************/
// state_t state;
// setpoint_t setpoint;
// stateEstimator(&state, tick);
// commanderGetSetpoint(&setpoint, &state);

/*************************************************
 * SENDING OUTPUT TO THE MOTORS
 *
 * The code sends an output to each motor. The output should have the be
 * of the typ unsigned 16-bit integer, i.e. use variables such as:
 * uint16_t value_i
 *
 ************************************************/
// motorsSetRatio(MOTOR_M1, value_1);
// motorsSetRatio(MOTOR_M2, value_2);
// motorsSetRatio(MOTOR_M3, value_3);
// motorsSetRatio(MOTOR_M4, value_4);

/*************************************************
 * LOGGING VALUES THAT CAN BE PLOTTEN IN PYTHON CLIENT
 *
 * We have already set up three log blocks to for the accelerometer data, the
 * gyro data and the setpoints, just uncomment the block to start logging. Use
 * them as reference if you want to add custom blocks.
 *
 ************************************************/
// LOG_GROUP_START(estimate)
// LOG_ADD(LOG_FLOAT, roll, &state.attitude.roll)
// LOG_ADD(LOG_FLOAT, pitch, &state.attitude.pitch)
// LOG_ADD(LOG_FLOAT, yaw, &state.attitude.yaw)
// LOG_GROUP_STOP(estimate)