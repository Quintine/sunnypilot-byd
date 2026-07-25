#include "opendbc/safety/declarations.h"

// MPC_LKAS_CMD (790), little endian
#define BYD_GET_LKAS_OUTPUT(msg)  (to_signed((((msg)->data[3] & 0x7U) << 8U) | (msg)->data[2], 11))
#define BYD_LKAS_ACTIVE_BIT       28U
// STEERING_TORQUE (792, HAN), little endian
#define BYD_GET_MAIN_TORQUE(msg)  (to_signed((((msg)->data[2] & 0xFU) << 8U) | (msg)->data[1], 12))
#define BYD_GET_DRIVER_TORQUE(msg) (to_signed((((msg)->data[4] & 0xFU) << 8U) | (msg)->data[3], 12))
// STEERING_TORQUE_ANGLE (508, SEAL), little endian
#define BYD_GET_MAIN_TORQUE_ANGLE(msg)  (to_signed((((msg)->data[5] & 0xFU) << 8U) | (msg)->data[4], 12))
#define BYD_GET_DRIVER_TORQUE_ANGLE(msg) (to_signed((((msg)->data[1] & 0xFU) << 4U) | ((msg)->data[0] >> 4U), 12))
// PCM_BUTTONS (944)
#define BYD_LKAS_ON_BTN_BIT       14U

static bool byd_tx_hook(const CANPacket_t *msg) {
  const TorqueSteeringLimits BYD_STEERING_LIMITS = {
    .max_torque = 300,
    .max_rate_up = 6,
    .max_rate_down = 6,
    .max_rt_delta = 112, // 75 per 250ms at 50Hz steer rate, plus 50% buffer
    .type = TorqueMotorLimited,
    .driver_torque_allowance = 46,
    .driver_torque_multiplier = 1,
    .max_torque_error = 46,
  };

  const LongitudinalLimits BYD_LONG_LIMITS = {
    .max_accel = 140,      // 2.0 m/s^2  (raw = (m/s^2 + 5) / 0.05)
    .min_accel = 20,       // -4.0 m/s^2
    .inactive_accel = 100, // 0 m/s^2, sent when longitudinal control is inactive
  };

  bool tx = true;

  // MPC_LKAS_CMD: steering actuation towards the EPS
  if (msg->addr == 790U) {
    int desired_torque = BYD_GET_LKAS_OUTPUT(msg);
    bool steer_req = GET_BIT(msg, BYD_LKAS_ACTIVE_BIT);

    if (steer_torque_cmd_checks(desired_torque, steer_req, BYD_STEERING_LIMITS)) {
      tx = false;
    }
  }

  // STEERING_TORQUE: feedback message towards the camera, torque echoes the
  // camera's own request. Only sanity-check the absolute limit so stock LKAS
  // requests don't fault when openpilot is inactive.
  if (msg->addr == 792U) {
    int main_torque = BYD_GET_MAIN_TORQUE(msg);
    if (safety_max_limit_check(main_torque, BYD_STEERING_LIMITS.max_torque, -BYD_STEERING_LIMITS.max_torque)) {
      tx = false;
    }
  }

  // ACC_CMD: longitudinal actuation
  if (msg->addr == 814U) {
    int desired_accel = msg->data[0];

    if (longitudinal_accel_checks(desired_accel, BYD_LONG_LIMITS)) {
      tx = false;
    }
  }

  return tx;
}

static void byd_rx_hook(const CANPacket_t *msg) {
  // Main Bus
  if (msg->bus == 0U) {
    // PEDAL
    if (msg->addr == 834U) {
      gas_pressed = msg->data[0] != 0U;
      brake_pressed = msg->data[1] != 0U;
    }
    // ESC
    if (msg->addr == 289U) {
      vehicle_moving = (((msg->data[1] & 0x0FU) << 8U) | msg->data[0]) != 0U;
    }
    // STEERING_TORQUE (HAN): sample EPS output torque and driver torque
    if (msg->addr == 792U) {
      int torque_meas_new = BYD_GET_MAIN_TORQUE(msg);
      update_sample(&torque_meas, torque_meas_new);
      torque_meas.min--;
      torque_meas.max++;

      int torque_driver_new = BYD_GET_DRIVER_TORQUE(msg);
      update_sample(&torque_driver, torque_driver_new);
    }
    // STEERING_TORQUE_ANGLE (SEAL): same feedback on a different message/layout
    if (msg->addr == 508U) {
      int torque_meas_new = BYD_GET_MAIN_TORQUE_ANGLE(msg);
      update_sample(&torque_meas, torque_meas_new);
      torque_meas.min--;
      torque_meas.max++;

      // driver torque factor is 0.1 on this message
      int torque_driver_new = (BYD_GET_DRIVER_TORQUE_ANGLE(msg) + 5) / 10;
      update_sample(&torque_driver, torque_driver_new);
    }
    // PCM_BUTTONS: dedicated LKAS steering wheel button for MADS
    if (msg->addr == 944U) {
      mads_button_press = GET_BIT(msg, BYD_LKAS_ON_BTN_BIT) ? MADS_BUTTON_PRESSED : MADS_BUTTON_NOT_PRESSED;
    }
  }
  // Cam Bus
  else if (msg->bus == 2U) {
    // ACC_HUD_ADAS
    if (msg->addr == 813U) {
      // CRUISE_STATE in (3, 5) == (Active, Override)
      uint8_t cruise_state = (msg->data[5] >> 4U) & 0x0FU;
      pcm_cruise_check((cruise_state == 3U) || (cruise_state == 5U));
    }
  } else {
  }
}

static safety_config byd_init(uint16_t param) {
  static const CanMsg BYD_TX_MSGS[] = {
    {790, 0, 8, .check_relay = true}, // MPC_LKAS_CMD
    {792, 2, 8, .check_relay = true}, // STEERING_TORQUE
  };

  static const CanMsg BYD_TX_LONG_MSGS[] = {
    {790, 0, 8, .check_relay = true}, // MPC_LKAS_CMD
    {792, 2, 8, .check_relay = true}, // STEERING_TORQUE
    {814, 0, 8, .check_relay = true}, // ACC_CMD
  };

  static RxCheck byd_rx_checks[] = {
    {.msg = {{578, 0, 8, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},  // DRIVE_STATE
    {.msg = {{544, 0, 8, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},  // BRAKE_APPLIED
    {.msg = {{834, 0, 8, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},  // PEDAL
    {.msg = {{289, 0, 8, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},  // ESC
    {.msg = {{287, 0, 5, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},  // STEER_MODULE
    {.msg = {{792, 0, 8, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true},              // STEERING_TORQUE (HAN)
             {508, 0, 8, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }}},   // STEERING_TORQUE_ANGLE (SEAL)
    {.msg = {{944, 0, 8, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},  // PCM_BUTTONS
    {.msg = {{813, 2, 8, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},  // ACC_HUD_ADAS
    {.msg = {{307, 0, 8, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},  // STALKS
    {.msg = {{660, 0, 8, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},  // METER_CLUSTER
    {.msg = {{790, 2, 8, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},  // MPC_LKAS_CMD
  };

  const uint16_t BYD_FLAG_LONG_CONTROL = 1;
  const bool byd_longitudinal = GET_FLAG(param, BYD_FLAG_LONG_CONTROL);

  safety_config ret;
  if (byd_longitudinal) {
    ret = BUILD_SAFETY_CFG(byd_rx_checks, BYD_TX_LONG_MSGS);
  } else {
    ret = BUILD_SAFETY_CFG(byd_rx_checks, BYD_TX_MSGS);
  }
  return ret;
}

const safety_hooks byd_hooks = {
  .init = byd_init,
  .rx = byd_rx_hook,
  .tx = byd_tx_hook,
};
