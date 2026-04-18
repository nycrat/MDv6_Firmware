/**
 ******************************************************************************
 * @file    interface.c
 * @author  UBC Thunderbots
 * @brief   SPI interface for motor control commands and telemetry
 * 
 * This module implements a simple SPI protocol for controlling the motor.
 * The master can send commands to set the target speed or torque, configure 
 * parameters, and select the type of telemetry response. The slave (this 
 * firmware) processes the commands and responds with telemetry data.
 ******************************************************************************
 */

#include "interface.h"

#include "stm32f0xx_hal.h"
#include "motorcontrol.h"
#include "speed_feed_forward_ctrl.h"
#include "crc.h"

#include <string.h>

/* External variables --------------------------------------------------------*/

extern SPI_HandleTypeDef hspi1;
extern DMA_HandleTypeDef hdma_spi1_rx;
extern DMA_HandleTypeDef hdma_spi1_tx;

extern PID_Handle_t PIDSpeedHandle_M1;
extern PID_Handle_t PIDIqHandle_M1;
extern PID_Handle_t PIDIdHandle_M1;

/* Private variables ---------------------------------------------------------*/

uint8_t rx_bufs[2][FRAME_SIZE];
uint8_t tx_bufs[2][FRAME_SIZE];

volatile bool txrx_half_completed;
volatile bool txrx_completed;

ResponseType_t response_type;

bool motor_enabled;

/* Private function prototypes -----------------------------------------------*/

void ProcessSpiTransaction(uint8_t *rx, uint8_t *tx);

void ProcessRx_SetTargetSpeed(uint8_t *rx);
void ProcessRx_SetTargetTorque(uint8_t *rx);
void ProcessRx_SetResponseType(uint8_t *rx);
void ProcessRx_SetPidTorqueKpKi(uint8_t *rx);
void ProcessRx_SetPidFluxKpKi(uint8_t *rx);
void ProcessRx_SetPidSpeedKpKi(uint8_t *rx);
void ProcessRx_SetSpeedFeedForwardKaKv(uint8_t *rx);

void PopulateTx_ResponseSpeedAndFaults(uint8_t *tx);
void PopulateTx_ResponseIqAndId(uint8_t *tx);
void PopulateTx_ResponseVqAndVd(uint8_t *tx);
void PopulateTx_ResponsePhaseCurrentAndVoltage(uint8_t *tx);
void PopulateTx_ResponseIqAndIqRef(uint8_t *tx);
void PopulateTx_ResponseIdAndIdRef(uint8_t *tx);
void PopulateTx_ResponseSpeedAndSpeedRef(uint8_t *tx);

void SetMotorEnabled(bool enabled);

/* Function implementations --------------------------------------------------*/

/**
 * Initializes the SPI interface module.
 */
void Interface_Init(void)
{
  memset(rx_bufs, 0, sizeof(rx_bufs));
  memset(tx_bufs, 0, sizeof(tx_bufs));

  txrx_half_completed = false;
  txrx_completed = false;

  response_type = SPEED_AND_FAULTS;

  motor_enabled = false;

  ProcessSpiTransaction(rx_bufs[0], tx_bufs[0]);
  ProcessSpiTransaction(rx_bufs[1], tx_bufs[1]);
}

/**
 * Main loop for the SPI interface module. 
 * Continuously waits for SPI transactions to complete and processes them.
 */
void Interface_Loop(void)
{
  HAL_SPI_TransmitReceive_DMA(&hspi1, (uint8_t *)tx_bufs, (uint8_t *)rx_bufs, FRAME_SIZE * 2);

  while (1) 
  {		
    if (txrx_half_completed) 
    {
      ProcessSpiTransaction(rx_bufs[0], tx_bufs[0]);
      txrx_half_completed = false;
    }
    if (txrx_completed) 
    {
      ProcessSpiTransaction(rx_bufs[1], tx_bufs[1]);
      txrx_completed = false;
    }
  }
}

/**
 * Processes a complete SPI transaction by executing the received command
 * in the RX buffer and populating the TX buffer based on the current 
 * response type.
 *
 * @param rx Pointer to the received SPI frame.
 * @param tx Pointer to the transmit SPI frame.
 */
void ProcessSpiTransaction(uint8_t *rx, uint8_t *tx) 
{
  uint8_t rx_copy[FRAME_SIZE];
  memcpy(rx_copy, rx, FRAME_SIZE);

  /* Frame integrity check */
  if (rx_copy[5] == CrcGenerateChecksum(rx_copy, FRAME_SIZE - 1)) 
  {
    const Opcode_t opcode = rx_copy[0];

    switch (opcode) 
    {
      case NO_OP:
        break;
      case SET_TARGET_SPEED:
        ProcessRx_SetTargetSpeed(rx_copy);
        break;
      case SET_TARGET_TORQUE:
        ProcessRx_SetTargetTorque(rx_copy);
        break;
      case SET_RESPONSE_TYPE:
        ProcessRx_SetResponseType(rx_copy);
        break;
      case SET_PID_TORQUE_KP_KI:
        ProcessRx_SetPidTorqueKpKi(rx_copy);
        break;
      case SET_PID_FLUX_KP_KI:
        ProcessRx_SetPidFluxKpKi(rx_copy);
        break;
      case SET_PID_SPEED_KP_KI:
        ProcessRx_SetPidSpeedKpKi(rx_copy);
        break;
      case SET_SPEED_FEED_FORWARD_KA_KV:
        ProcessRx_SetSpeedFeedForwardKaKv(rx_copy);
        break;
    }
  }

  switch (response_type) 
  {
    case SPEED_AND_FAULTS:
      PopulateTx_ResponseSpeedAndFaults(tx);
      break;
    case IQ_AND_ID:
      PopulateTx_ResponseIqAndId(tx);
      break;
    case VQ_AND_VD:
      PopulateTx_ResponseVqAndVd(tx);
      break;
    case PHASE_CURRENT_AND_VOLTAGE:
      PopulateTx_ResponsePhaseCurrentAndVoltage(tx);
      break;
    case IQ_AND_IQ_REF:
      PopulateTx_ResponseIqAndIqRef(tx);
      break;
    case ID_AND_ID_REF:
      PopulateTx_ResponseIdAndIdRef(tx);
      break;
    case SPEED_AND_SPEED_REF:
      PopulateTx_ResponseSpeedAndSpeedRef(tx);
      break;
  }
}

/**
 * Processes the received SPI frame to update the target speed for 
 * the motor and start/stop the motor based on the enable flag.
 *
 *  +--------+--------+---------------+-------+-------+
 *  | Opcode | Enable |     Speed     |       |  CRC  |
 *  +--------+--------+---------------+-------+-------+
 *      0        1        2       3       4       5
 */
void ProcessRx_SetTargetSpeed(uint8_t *rx)
{
  const int16_t motor_target_speed = (rx[2] << 8) | rx[3];
  MC_ProgramSpeedRampMotor1(motor_target_speed, 0);
  SetMotorEnabled(rx[1]);
}

/**
 * Processes the received SPI frame to update the target torque for 
 * the motor and start/stop the motor based on the enable flag.
 *
 *  +--------+--------+---------------+-------+-------+
 *  | Opcode | Enable |    Torque     |       |  CRC  |
 *  +--------+--------+---------------+-------+-------+
 *      0        1        2       3       4       5
 */
void ProcessRx_SetTargetTorque(uint8_t *rx) 
{
  const int16_t motor_target_torque = (rx[2] << 8) | rx[3];
  MC_ProgramTorqueRampMotor1(motor_target_torque, 0);
  SetMotorEnabled(rx[1]);
}

/**
 * Processes the received SPI frame to update the type of response 
 * that will be sent back to the master in subsequent transactions.
 *
 *  +--------+--------+-----------------------+-------+
 *  | Opcode |  Type  |                       |  CRC  |
 *  +--------+--------+-----------------------+-------+
 *      0        1         2      3      4        5
 */
void ProcessRx_SetResponseType(uint8_t *rx) 
{
  response_type = rx[1];
}

/**
 * Processes the received SPI frame to update the Kp and Ki 
 * parameters for the torque (Iq) PID controller.
 *
 *  +--------+----------------+---------------+-------+
 *  | Opcode |       Kp       |       Ki      |  CRC  |
 *  +--------+----------------+---------------+-------+
 *      0        1       2        3       4       5
 */
void ProcessRx_SetPidTorqueKpKi(uint8_t *rx) 
{
  const int16_t kp = (rx[1] << 8) | rx[2];
  const int16_t ki = (rx[3] << 8) | rx[4];

  PID_SetKP(&PIDIqHandle_M1, kp);
  PID_SetKI(&PIDIqHandle_M1, ki);
}

/**
 * Processes the received SPI frame to update the Kp and Ki 
 * parameters for the flux (Id) PID controller.
 *
 *  +--------+----------------+---------------+-------+
 *  | Opcode |       Kp       |       Ki      |  CRC  |
 *  +--------+----------------+---------------+-------+
 *      0        1       2        3       4       5
 */
void ProcessRx_SetPidFluxKpKi(uint8_t *rx) 
{
  const int16_t kp = (rx[1] << 8) | rx[2];
  const int16_t ki = (rx[3] << 8) | rx[4];
  
  PID_SetKP(&PIDIdHandle_M1, kp);
  PID_SetKI(&PIDIdHandle_M1, ki);
}

/**
 * Processes the received SPI frame to update the Kp and Ki 
 * parameters for the speed PID controller.
 *
 *  +--------+----------------+---------------+-------+
 *  | Opcode |       Kp       |       Ki      |  CRC  |
 *  +--------+----------------+---------------+-------+
 *      0        1       2        3       4       5
 */
void ProcessRx_SetPidSpeedKpKi(uint8_t *rx) 
{
  const int16_t kp = (rx[1] << 8) | rx[2];
  const int16_t ki = (rx[3] << 8) | rx[4];
  
  PID_SetKP(&PIDSpeedHandle_M1, kp);
  PID_SetKI(&PIDSpeedHandle_M1, ki);
}

/**
 * Processes the received SPI frame to update the Ka and Kv 
 * parameters for the speed feedforward controller.
 *
 *  +--------+----------------+---------------+-------+
 *  | Opcode |       Ka       |       Kv      |  CRC  |
 *  +--------+----------------+---------------+-------+
 *      0        1       2        3       4       5
 */
void ProcessRx_SetSpeedFeedForwardKaKv(uint8_t *rx) 
{
  const int16_t ka = (rx[1] << 8) | rx[2];
  const int16_t kv = (rx[3] << 8) | rx[4];

  const SpeedFF_TuningStruct_t constants = 
  {
    .wKaGain = (int32_t)ka,
    .wKvGain = (int32_t)kv,
  };

  SpeedFF_SetFFConstants(&SpeedFF_M1, constants);
}

/**
 * Populates the TX buffer with fields corresponding to the
 * current measured speed and motor faults.
 *
 *  +--------+---------------+----------------+-------+
 *  |  Type  |  Speed (RPM)  |     Faults     |  CRC  |
 *  +--------+---------------+----------------+-------+
 *      0        1       2        3      4       5
 */
void PopulateTx_ResponseSpeedAndFaults(uint8_t *tx) 
{
  const int16_t motor_current_speed = MC_GetMecSpeedAverageMotor1();
  const int16_t motor_faults = MC_GetOccurredFaultsMotor1();

  tx[0] = SPEED_AND_FAULTS;
  tx[1] = (motor_current_speed >> 8) & 0xFF;
  tx[2] = motor_current_speed & 0xFF;
  tx[3] = (motor_faults >> 8) & 0xFF;
  tx[4] = motor_faults & 0xFF;
  tx[5] = CrcGenerateChecksum(tx, FRAME_SIZE - 1);
}

/**
 * Populates the TX buffer with fields corresponding to the
 * current Iq and Id values for the motor.
 *
 *  +--------+---------------+----------------+-------+
 *  |  Type  |      Iq       |       Id       |  CRC  |
 *  +--------+---------------+----------------+-------+
 *      0        1       2        3      4       5
 */
void PopulateTx_ResponseIqAndId(uint8_t *tx) 
{
  const qd_t motor_iqd = MC_GetIqdMotor1();

  tx[0] = IQ_AND_ID;
  tx[1] = (motor_iqd.q >> 8) & 0xFF;
  tx[2] = motor_iqd.q & 0xFF;
  tx[3] = (motor_iqd.d >> 8) & 0xFF;
  tx[4] = motor_iqd.d & 0xFF;
  tx[5] = CrcGenerateChecksum(tx, FRAME_SIZE - 1);
}

/**
 * Populates the TX buffer with fields corresponding to the
 * current Vq and Vd values for the motor.
 *
 *  +--------+---------------+----------------+-------+
 *  |  Type  |      Vq       |       Vd       |  CRC  |
 *  +--------+---------------+----------------+-------+
 *      0        1       2        3      4       5
 */
void PopulateTx_ResponseVqAndVd(uint8_t *tx) 
{
  const qd_t motor_vqd = MC_GetVqdMotor1();

  tx[0] = VQ_AND_VD;
  tx[1] = (motor_vqd.q >> 8) & 0xFF;
  tx[2] = motor_vqd.q & 0xFF;
  tx[3] = (motor_vqd.d >> 8) & 0xFF;
  tx[4] = motor_vqd.d & 0xFF;
  tx[5] = CrcGenerateChecksum(tx, FRAME_SIZE - 1);
}

/**
 * Populates the TX buffer with fields corresponding to the
 * phase current and voltage amplitudes for the motor.
 *
 *  +--------+---------------+----------------+-------+
 *  |  Type  | Phase Current |  Phase Voltage |  CRC  |
 *  +--------+---------------+----------------+-------+
 *      0        1       2        3      4       5
 */
void PopulateTx_ResponsePhaseCurrentAndVoltage(uint8_t *tx) 
{
  const int16_t motor_phase_current = MC_GetPhaseCurrentAmplitudeMotor1();
  const int16_t motor_phase_voltage = MC_GetPhaseVoltageAmplitudeMotor1();

  tx[0] = PHASE_CURRENT_AND_VOLTAGE;
  tx[1] = (motor_phase_current >> 8) & 0xFF;
  tx[2] = motor_phase_current & 0xFF;
  tx[3] = (motor_phase_voltage >> 8) & 0xFF;
  tx[4] = motor_phase_voltage & 0xFF;
  tx[5] = CrcGenerateChecksum(tx, FRAME_SIZE - 1);
}

/**
 * Populates the TX buffer with fields corresponding to the
 * current Iq and reference Iq values for the motor.
 *
 *  +--------+---------------+----------------+-------+
 *  |  Type  |      Iq       |     Iq Ref     |  CRC  |
 *  +--------+---------------+----------------+-------+
 *      0        1       2        3      4       5
 */
void PopulateTx_ResponseIqAndIqRef(uint8_t *tx) 
{
  const qd_t motor_iqd = MC_GetIqdMotor1();
  const qd_t motor_iqd_ref = MC_GetIqdrefMotor1();

  tx[0] = IQ_AND_IQ_REF;
  tx[1] = (motor_iqd.q >> 8) & 0xFF;
  tx[2] = motor_iqd.q & 0xFF;
  tx[3] = (motor_iqd_ref.q >> 8) & 0xFF;
  tx[4] = motor_iqd_ref.q & 0xFF;
  tx[5] = CrcGenerateChecksum(tx, FRAME_SIZE - 1);
}

/**
 * Populates the TX buffer with fields corresponding to the
 * current Id and reference Id values for the motor.
 *
 *  +--------+---------------+----------------+-------+
 *  |  Type  |      Id       |     Id Ref     |  CRC  |
 *  +--------+---------------+----------------+-------+
 *      0        1       2        3      4       5
 */
void PopulateTx_ResponseIdAndIdRef(uint8_t *tx) 
{
  const qd_t motor_iqd = MC_GetIqdMotor1();
  const qd_t motor_iqd_ref = MC_GetIqdrefMotor1();

  tx[0] = ID_AND_ID_REF;
  tx[1] = (motor_iqd.d >> 8) & 0xFF;
  tx[2] = motor_iqd.d & 0xFF;
  tx[3] = (motor_iqd_ref.d >> 8) & 0xFF;
  tx[4] = motor_iqd_ref.d & 0xFF;
  tx[5] = CrcGenerateChecksum(tx, FRAME_SIZE - 1);
}

/**
 * Populates the TX buffer with fields corresponding to the
 * current measured speed and reference speed in RPM.
 *
 *  +--------+---------------+----------------+-------+
 *  |  Type  |     Speed     |    Speed Ref   |  CRC  |
 *  +--------+---------------+----------------+-------+
 *      0        1       2        3      4       5
 */
void PopulateTx_ResponseSpeedAndSpeedRef(uint8_t *tx) 
{
  const int16_t motor_speed = MC_GetMecSpeedAverageMotor1();
  const int16_t motor_speed_ref = MC_GetMecSpeedReferenceMotor1();

  tx[0] = SPEED_AND_SPEED_REF;
  tx[1] = (motor_speed >> 8) & 0xFF;
  tx[2] = motor_speed & 0xFF;
  tx[3] = (motor_speed_ref >> 8) & 0xFF;
  tx[4] = motor_speed_ref & 0xFF;
  tx[5] = CrcGenerateChecksum(tx, FRAME_SIZE - 1);
}

/**
 * Enables or disables the motor based on the provided flag.
 *
 * @param enabled Whether to enable or disable the motor.
 */
void SetMotorEnabled(bool enabled) 
{
  if (enabled && !motor_enabled) 
  {
    MC_StartMotor1();
  } 
  else if (!enabled && motor_enabled) 
  {
    MC_StopMotor1();
  }
  motor_enabled = enabled;
}

/* SPI callback handlers -----------------------------------------------------*/

void HAL_SPI_TxRxHalfCpltCallback(SPI_HandleTypeDef* hspi)
{
  if (hspi->Instance == SPI1) 
  {
    txrx_half_completed = true;
  }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef* hspi)
{
  if (hspi->Instance == SPI1) 
  {
    txrx_completed = true;
  }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef* hspi)
{
  if (hspi->Instance == SPI1) 
  {
    // Do nothing for now
  }
}
