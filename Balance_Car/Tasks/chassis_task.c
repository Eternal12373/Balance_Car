/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       chassis.c/h
  * @brief      chassis control task,
  *             ���̿�������
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add chassis power control
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#include "chassis_task.h"
#include "chassis_behaviour.h"

#include "cmsis_os.h"
#include "user_lib.h"
#include "arm_math.h"
#include "pid.h"
#include "remote_control.h"
#include "CAN_receive.h"
#include "detect_task.h"
#include "INS_task.h"
// #include "chassis_power_control.h"

#define rc_deadband_limit(input, output, dealine)    \
  {                                                  \
    if ((input) > (dealine) || (input) < -(dealine)) \
    {                                                \
      (output) = (input);                            \
    }                                                \
    else                                             \
    {                                                \
      (output) = 0;                                  \
    }                                                \
  }

static void speed_loop(void);
static void balance_loop(void);
static void pid_control_loop(void);
static void lqr_control_loop(void);

static void chassis_init(chassis_move_t *chassis_move_init);
static void chassis_set_mode(chassis_move_t *chassis_move_mode);
static void chassis_feedback_update(chassis_move_t *chassis_move_update);
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop);
static void chassis_set_contorl(chassis_move_t *chassis_move_control);

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t chassis_high_water;
#endif

// �����˶�����
chassis_move_t chassis_move;

/**
 * @brief          �������񣬼�� CHASSIS_CONTROL_TIME_MS 2ms
 * @param[in]      pvParameters: ��
 * @retval         none
 */
void chassis_task(void const *pvParameters)
{
  // ����һ��ʱ��
  vTaskDelay(CHASSIS_TASK_INIT_TIME);
  // ���̳�ʼ��
  chassis_init(&chassis_move);

  // �жϵ��̵���Ƿ�����
  //    while (toe_is_error(CHASSIS_MOTOR1_TOE) || toe_is_error(CHASSIS_MOTOR2_TOE) || toe_is_error(CHASSIS_MOTOR3_TOE) || toe_is_error(CHASSIS_MOTOR4_TOE) || toe_is_error(DBUS_TOE))
  //    {
  //        vTaskDelay(CHASSIS_CONTROL_TIME_MS);
  //    }

  while (1)
  {
    // ���õ��̿���ģʽ
    chassis_set_mode(&chassis_move);
    // �������ݸ���
    chassis_feedback_update(&chassis_move);
    // ���̿���������
    chassis_set_contorl(&chassis_move);
    // ���̿���PID����
    chassis_control_loop(&chassis_move);

    // ȷ������һ��������ߣ� ����CAN���ư����Ա����յ�
    if (!(toe_is_error(CHASSIS_MOTOR1_TOE) && toe_is_error(CHASSIS_MOTOR2_TOE)))
    {
      // ��ң�������ߵ�ʱ�򣬷��͸����̵�������.
      if (toe_is_error(DBUS_TOE))
      {
        CAN_cmd_chassis(0, 0, 0, 0);
      }
      else
      {
        // ���Ϳ��Ƶ���
        //CAN_cmd_chassis(0, 0, 0, 0);
        CAN_cmd_chassis(chassis_move.motor_chassis[0].give_current, chassis_move.motor_chassis[1].give_current, 0, 0);
      }
    }
    // ϵͳ��ʱ
    vTaskDelay(CHASSIS_CONTROL_TIME_MS);

#if INCLUDE_uxTaskGetStackHighWaterMark
    chassis_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
  }
}

/**
 * @brief          ��ʼ��"chassis_move"����������pid��ʼ���� ң����ָ���ʼ����3508���̵��ָ���ʼ������̨�����ʼ���������ǽǶ�ָ���ʼ��
 * @param[out]     chassis_move_init:"chassis_move"����ָ��.
 * @retval         none
 */
static void chassis_init(chassis_move_t *chassis_move_init)
{
  if (chassis_move_init == NULL)
  {
    return;
  }

  const static fp32 chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
  const static fp32 chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};
  uint8_t i;

  // ��ȡң����ָ��
  chassis_move_init->chassis_RC = get_remote_control_point();
  // ��ȡ��������̬��ָ��
  chassis_move_init->chassis_INS_angle = get_INS_angle_point();
  chassis_move_init->chassis_INS_angle_speed = get_gyro_data_point();
  // ��ȡ��̨�������ָ��
  chassis_move_init->chassis_yaw_motor = get_yaw_motor_point();
  chassis_move_init->chassis_pitch_motor = get_pitch_motor_point();

  // ��ȡ���̵������ָ��
  for (i = 0; i < 2; i++)
  {
    chassis_move_init->motor_chassis[i].chassis_motor_measure = get_chassis_motor_measure_point(i);
    PID_init(&chassis_move_init->motor_speed_pid[i], 0, 0, 0, 0, 0);
  }
  PID_init(&chassis_move_init->balance_loop_pid, BALANCE_LOOP_PID_KP, BALANCE_LOOP_PID_KI, BALANCE_LOOP_PID_KD, BALANCE_LOOP_PID_MAX_OUT, BALANCE_LOOP_PID_MAX_IOUT);

  PID_init(&chassis_move_init->speed_loop_pid, SPEED_LOOP_PID_KP, SPEED_LOOP_PID_KI, SPEED_LOOP_PID_KD, SPEED_LOOP_PID_MAX_OUT, SPEED_LOOP_PID_MAX_IOUT);

  // ��ʼ���Ƕ�PID
  PID_init(&chassis_move_init->chassis_angle_pid, CHASSIS_FOLLOW_GIMBAL_PID_KP, CHASSIS_FOLLOW_GIMBAL_PID_KI, CHASSIS_FOLLOW_GIMBAL_PID_KD, CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT);

  // ��һ���˲�����б����������
  first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vx, CHASSIS_CONTROL_TIME, chassis_x_order_filter);
  first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vy, CHASSIS_CONTROL_TIME, chassis_y_order_filter);

  // ��� ��С�ٶ�
  chassis_move_init->vx_max_speed = NORMAL_MAX_CHASSIS_SPEED_X;
  chassis_move_init->vx_min_speed = -NORMAL_MAX_CHASSIS_SPEED_X;

  // ����һ������
  chassis_feedback_update(chassis_move_init);
}

/**
 * @brief          ���õ��̿���ģʽ����Ҫ��'chassis_behaviour_mode_set'�����иı�
 * @param[out]     chassis_move_mode:"chassis_move"����ָ��.
 * @retval         none
 */
static void chassis_set_mode(chassis_move_t *chassis_move_mode)
{
  if (chassis_move_mode == NULL)
  {
    return;
  }
  // in file "chassis_behaviour.c"
  chassis_behaviour_mode_set(chassis_move_mode);
}

/**
 * @brief          ���̲������ݸ��£���������ٶȣ�ŷ���Ƕȣ��������ٶ�
 * @param[out]     chassis_move_update:"chassis_move"����ָ��.
 * @retval         none
 */
static void chassis_feedback_update(chassis_move_t *chassis_move_update)
{
  if (chassis_move_update == NULL)
  {
    return;
  }

  uint8_t i = 0;
  for (i = 0; i < 2; i++)
  {
    // update motor speed, accel is differential of speed PID
    // ���µ���ٶȣ����ٶ����ٶȵ�PID΢��
    chassis_move_update->motor_chassis[i].speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * chassis_move_update->motor_chassis[i].chassis_motor_measure->speed_rpm;
    chassis_move_update->motor_chassis[i].accel = chassis_move_update->motor_speed_pid[i].Dbuf[0] * CHASSIS_CONTROL_FREQUENCE;
  }

  // calculate vertical speed, horizontal speed ,rotation speed, left hand rule
  chassis_move_update->vx = 0.5f * (chassis_move_update->motor_chassis[0].speed - chassis_move_update->motor_chassis[1].speed);

  // ���������̬�Ƕ�, �����������������������ⲿ�ִ���
  // chassis_move_update->chassis_yaw = rad_format(*(chassis_move_update->chassis_INS_angle + INS_YAW_ADDRESS_OFFSET) - chassis_move_update->chassis_yaw_motor->relative_angle);
  // chassis_move_update->chassis_pitch = rad_format(*(chassis_move_update->chassis_INS_angle + INS_PITCH_ADDRESS_OFFSET) - chassis_move_update->chassis_pitch_motor->relative_angle);
  // chassis_move_update->chassis_roll = *(chassis_move_update->chassis_INS_angle + INS_ROLL_ADDRESS_OFFSET);

  /**balance data**/
  chassis_move_update->chassis_yaw = rad_format(*(chassis_move_update->chassis_INS_angle + INS_YAW_ADDRESS_OFFSET));
  chassis_move_update->chassis_pitch = rad_format(*(chassis_move_update->chassis_INS_angle + INS_PITCH_ADDRESS_OFFSET));
  chassis_move_update->chassis_roll = *(chassis_move_update->chassis_INS_angle + INS_ROLL_ADDRESS_OFFSET);

  chassis_move_update->chassis_yaw_speed = *(chassis_move_update->chassis_INS_angle_speed + INS_YAW_ADDRESS_OFFSET);
  chassis_move_update->chassis_pitch_speed = *(chassis_move_update->chassis_INS_angle_speed + INS_PITCH_ADDRESS_OFFSET);
  chassis_move_update->chassis_roll_speed = *(chassis_move_update->chassis_INS_angle_speed + INS_ROLL_ADDRESS_OFFSET);
}
/**
 * @brief          accroding to the channel value of remote control, calculate chassis vertical and horizontal speed set-point
 *
 * @param[out]     vx_set: vertical speed set-point
 * @param[out]     vy_set: horizontal speed set-point
 * @param[out]     chassis_move_rc_to_vector: "chassis_move" valiable point
 * @retval         none
 */
/**
 * @brief          ����ң����ͨ��ֵ����������ͺ����ٶ�
 *
 * @param[out]     vx_set: �����ٶ�ָ��
 * @param[out]     vy_set: �����ٶ�ָ��
 * @param[out]     chassis_move_rc_to_vector: "chassis_move" ����ָ��
 * @retval         none
 */
void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector)
{
  if (chassis_move_rc_to_vector == NULL || vx_set == NULL || vy_set == NULL)
  {
    return;
  }

  int16_t vx_channel;
  fp32 vx_set_channel;
  // deadline, because some remote control need be calibrated,  the value of rocker is not zero in middle place,
  // �������ƣ���Ϊң�������ܴ��ڲ��� ҡ�����м䣬��ֵ��Ϊ0
  rc_deadband_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL], vx_channel, CHASSIS_RC_DEADLINE);

  vx_set_channel = vx_channel * CHASSIS_VX_RC_SEN;

  // keyboard set speed set-point
  // ���̿���
  if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_FRONT_KEY)
  {
    vx_set_channel = chassis_move_rc_to_vector->vx_max_speed;
  }
  else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_BACK_KEY)
  {
    vx_set_channel = chassis_move_rc_to_vector->vx_min_speed;
  }

  // һ�׵�ͨ�˲�����б����Ϊ�����ٶ�����
  first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vx, vx_set_channel);

  // ֹͣ�źţ�����Ҫ�������٣�ֱ�Ӽ��ٵ���
  if (vx_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN && vx_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN)
  {
    chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out = 0.0f;
  }

  *vx_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out;
}
/**
 * @brief          set chassis control set-point, three movement control value is set by "chassis_behaviour_control_set".
 * @param[out]     chassis_move_update: "chassis_move" valiable point
 * @retval         none
 */
/**
 * @brief          ���õ��̿�������ֵ, ���˶�����ֵ��ͨ��chassis_behaviour_control_set�������õ�
 * @param[out]     chassis_move_update:"chassis_move"����ָ��.
 * @retval         none
 */
static void chassis_set_contorl(chassis_move_t *chassis_move_control)
{

  if (chassis_move_control == NULL)
  {
    return;
  }

  chassis_behaviour_control_set(chassis_move_control);

#if 0
  // follow gimbal mode
  // ������̨ģʽ
  if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
  {
    fp32 sin_yaw = 0.0f, cos_yaw = 0.0f;
    // rotate chassis direction, make sure vertial direction follow gimbal
    // ��ת���Ƶ����ٶȷ��򣬱�֤ǰ����������̨�����������˶�ƽ��
    sin_yaw = arm_sin_f32(-chassis_move_control->chassis_yaw_motor->relative_angle);
    cos_yaw = arm_cos_f32(-chassis_move_control->chassis_yaw_motor->relative_angle);
    chassis_move_control->vx_set = cos_yaw * vx_set + sin_yaw * vy_set;
    chassis_move_control->vy_set = -sin_yaw * vx_set + cos_yaw * vy_set;
    // set control relative angle  set-point
    // ���ÿ��������̨�Ƕ�
    chassis_move_control->chassis_relative_angle_set = rad_format(angle_set);
    // calculate ratation speed
    // ������תPID���ٶ�
    chassis_move_control->wz_set = -PID_calc(&chassis_move_control->chassis_angle_pid, chassis_move_control->chassis_yaw_motor->relative_angle, chassis_move_control->chassis_relative_angle_set);
    // speed limit
    // �ٶ��޷�
    chassis_move_control->vx_set = fp32_constrain(chassis_move_control->vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
    chassis_move_control->vy_set = fp32_constrain(chassis_move_control->vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
  }
  else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW)
  {
    fp32 delat_angle = 0.0f;
    // set chassis yaw angle set-point
    // ���õ��̿��ƵĽǶ�
    chassis_move_control->chassis_yaw_set = rad_format(angle_set);
    delat_angle = rad_format(chassis_move_control->chassis_yaw_set - chassis_move_control->chassis_yaw);
    // calculate rotation speed
    // ������ת�Ľ��ٶ�
    chassis_move_control->wz_set = PID_calc(&chassis_move_control->chassis_angle_pid, 0.0f, delat_angle);
    // speed limit
    // �ٶ��޷�
    chassis_move_control->vx_set = fp32_constrain(vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
    chassis_move_control->vy_set = fp32_constrain(vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
  }
  else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW)
  {
    //"angle_set" is rotation speed set-point
    // ��angle_set�� ����ת�ٶȿ���
    chassis_move_control->wz_set = angle_set;
    chassis_move_control->vx_set = fp32_constrain(vx_set, chassis_move_control->vx_min_speed, chassis_move_control->vx_max_speed);
    chassis_move_control->vy_set = fp32_constrain(vy_set, chassis_move_control->vy_min_speed, chassis_move_control->vy_max_speed);
  }
  else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_RAW)
  {
    // in raw mode, set-point is sent to CAN bus
    // ��ԭʼģʽ������ֵ�Ƿ��͵�CAN����
    chassis_move_control->vx_set = vx_set;
    chassis_move_control->vy_set = vy_set;
    chassis_move_control->wz_set = angle_set;
    chassis_move_control->chassis_cmd_slow_set_vx.out = 0.0f;
    chassis_move_control->chassis_cmd_slow_set_vy.out = 0.0f;
  }

#endif
}

/**
 * @brief          control loop, according to control set-point, calculate motor current,
 *                 motor current will be sentto motor
 * @param[out]     chassis_move_control_loop: "chassis_move" valiable point
 * @retval         none
 */
/**
 * @brief          ����ѭ�������ݿ����趨ֵ������������ֵ�����п���
 * @param[out]     chassis_move_control_loop:"chassis_move"����ָ��.
 * @retval         none
 */
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop)
{

  for (uint8_t i = 0; i < 2; i++)
  {
    PID_calc(&chassis_move.motor_speed_pid[i], chassis_move.motor_chassis[i].speed, 0);
  }

  if (chassis_move.chassis_mode == CHASSIS_ZERO_FORCE)
  {
    return;
  }

#if PID_CONTROL_LOOP
  pid_control_loop();
#endif

#if LQR_CONTROL_LOOP
  lqr_control_loop();
#endif

   for (uint8_t i = 0; i < 2; i++)
   {
     chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(chassis_move_control_loop->motor_chassis[i].torque_set / CHASSIS_MOTOR_CURRENT_TO_TORQUE_SEN);
   }
   
  // ����pid
  // for (i = 0; i < 4; i++)
  // {
  //   PID_calc(&chassis_move_control_loop->motor_speed_pid[i], chassis_move_control_loop->motor_chassis[i].speed, chassis_move_control_loop->motor_chassis[i].speed_set);
  // }

  // ���ʿ���
  // chassis_power_control(chassis_move_control_loop);

  // ��ֵ����ֵ
  //  for (i = 0; i < 4; i++)
  //  {
  //    chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(chassis_move_control_loop->motor_speed_pid[i].out);
  //  }
}

static void pid_control_loop()
{
  balance_loop();

  speed_loop();
}
static void speed_loop()
{
}
static void balance_loop()
{
}
static void lqr_control_loop()
{

  chassis_move.motor_chassis[0].torque_set = -(LQR_K2 * (chassis_move.vx - chassis_move.vx_set) - LQR_K3 * chassis_move.chassis_pitch + LQR_K4 * (-chassis_move.chassis_pitch_speed) + LQR_K15 * chassis_move.delta_angle + LQR_K16 * chassis_move.chassis_yaw_speed);
  chassis_move.motor_chassis[1].torque_set = (LQR_K2 * (chassis_move.vx - chassis_move.vx_set) - LQR_K3 * chassis_move.chassis_pitch + LQR_K4 * (-chassis_move.chassis_pitch_speed) + LQR_K25 * chassis_move.delta_angle + LQR_K26 * chassis_move.chassis_yaw_speed);
}