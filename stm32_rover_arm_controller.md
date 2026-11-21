/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : STM32 Blue Pill control loop for 4WD rover, HC-SR04,
  *                   and 4-DOF PWM robotic arm.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include <stdint.h>

extern UART_HandleTypeDef huart1;
extern TIM_HandleTypeDef htim2;  /* PWM channels for 4-DOF servos (CH1..CH4) */
extern TIM_HandleTypeDef htim3;  /* PWM channels for L293D motor speed (CH1..CH2) */

#define OBSTACLE_THRESHOLD_CM  25U

/* Example L293D direction pins (update to match your CubeMX GPIO labels) */
#define IN1_Pin GPIO_PIN_0
#define IN1_GPIO_Port GPIOB
#define IN2_Pin GPIO_PIN_1
#define IN2_GPIO_Port GPIOB
#define IN3_Pin GPIO_PIN_10
#define IN3_GPIO_Port GPIOB
#define IN4_Pin GPIO_PIN_11
#define IN4_GPIO_Port GPIOB

/* Example HC-SR04 pins */
#define TRIG_Pin GPIO_PIN_12
#define TRIG_GPIO_Port GPIOB
#define ECHO_Pin GPIO_PIN_13
#define ECHO_GPIO_Port GPIOB

static void Rover_Stop(void)
{
  HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_RESET);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
}

static void Rover_Forward(uint16_t speed)
{
  HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_RESET);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, speed);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, speed);
}

static void Rover_Backward(uint16_t speed)
{
  HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_SET);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, speed);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, speed);
}

static void Rover_TurnLeft(uint16_t speed)
{
  HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_RESET);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, speed);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, speed);
}

static void Rover_TurnRight(uint16_t speed)
{
  HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_SET);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, speed);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, speed);
}

/* Convert angle (0..180 deg) to 1.0ms..2.0ms pulse width on a 20ms (50Hz) timer */
static void Servo_SetAngle(uint32_t channel, uint8_t angle)
{
  if (angle > 180U) angle = 180U;
  uint32_t pulse = 1000U + ((uint32_t)angle * 1000U) / 180U;
  __HAL_TIM_SET_COMPARE(&htim2, channel, pulse);
}

static void Arm_PickAndPlace(void)
{
  Rover_Stop();
  /* Align shoulder/elbow, close gripper, lift, rotate base, release, return home */
  Servo_SetAngle(TIM_CHANNEL_2, 60);   /* Shoulder lower */
  Servo_SetAngle(TIM_CHANNEL_3, 70);   /* Elbow reach */
  HAL_Delay(600);
  Servo_SetAngle(TIM_CHANNEL_4, 110);  /* Gripper close */
  HAL_Delay(500);
  Servo_SetAngle(TIM_CHANNEL_2, 110);  /* Shoulder lift */
  HAL_Delay(600);
  Servo_SetAngle(TIM_CHANNEL_1, 135);  /* Base rotate to drop zone */
  HAL_Delay(700);
  Servo_SetAngle(TIM_CHANNEL_4, 40);   /* Gripper open */
  HAL_Delay(500);
  Servo_SetAngle(TIM_CHANNEL_1, 90);   /* Base home */
  Servo_SetAngle(TIM_CHANNEL_2, 90);   /* Shoulder home */
  Servo_SetAngle(TIM_CHANNEL_3, 90);   /* Elbow home */
}

/* Placeholder distance reader (replace with DWT/timer echo measurement) */
static uint32_t Ultrasonic_ReadCm(void)
{
  HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);
  return 100U;
}

void Rover_ControlLoop(void)
{
  uint8_t rx_cmd = 'S';

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

  /* Neutral servo positions */
  Servo_SetAngle(TIM_CHANNEL_1, 90);
  Servo_SetAngle(TIM_CHANNEL_2, 90);
  Servo_SetAngle(TIM_CHANNEL_3, 90);
  Servo_SetAngle(TIM_CHANNEL_4, 40);

  while (1)
  {
    uint32_t dist_cm = Ultrasonic_ReadCm();
    if (dist_cm < OBSTACLE_THRESHOLD_CM)
    {
      Rover_Stop();
      HAL_Delay(100);
      continue;
    }

    if (HAL_UART_Receive(&huart1, &rx_cmd, 1, 50) == HAL_OK)
    {
      switch (rx_cmd)
      {
        case 'F': Rover_Forward(600); break;
        case 'B': Rover_Backward(600); break;
        case 'L': Rover_TurnLeft(550); break;
        case 'R': Rover_TurnRight(550); break;
        case 'P': Arm_PickAndPlace(); break;
        case 'S':
        default:  Rover_Stop(); break;
      }
    }
  }
}