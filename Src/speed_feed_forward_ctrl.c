/**
 ******************************************************************************
 * @file    speed_feed_forward_ctrl.c
 * @author  UBC Thunderbots
 * @brief   This module implements a feedforward component for speed control.
 ******************************************************************************
 */

#include "speed_feed_forward_ctrl.h"

#include "mc_type.h"

#include <stddef.h>

/**
 * Initializes the speed feedforward control module.
 */
__weak void SpeedFF_Init(SpeedFF_Handle_t *pHandle)
{
  pHandle->wKaGain = pHandle->wDefKaGain;
  pHandle->wKvGain = pHandle->wDefKvGain;
  pHandle->hSpeedRefPrevious = 0;
  pHandle->hSpeedRefDelta = 0;
  pHandle->hTorqueRefFeedForward = 0;
}

/**
 * Clears the speed feedforward control internal state.
 */
__weak void SpeedFF_Clear(SpeedFF_Handle_t *pHandle)
{
  pHandle->hSpeedRefPrevious = 0;
  pHandle->hSpeedRefDelta = 0;
  pHandle->hTorqueRefFeedForward = 0;
}

/**
 * Computes feedforward torque contribution based on speed reference and its derivative.
 * Torque_ff = Ka * d(SpeedRef)/dt + Kv * SpeedRef
 */
__weak void SpeedFF_ComputeTorqueReference(SpeedFF_Handle_t *pHandle, int16_t *hTorqueRef, int16_t hSpeedRef)
{
  int16_t hSpeedRefDelta;
  int32_t wTorqueFF;
  int32_t wTorqueRefCombined;

  hSpeedRefDelta = hSpeedRef - pHandle->hSpeedRefPrevious;
  pHandle->hSpeedRefDelta = hSpeedRefDelta;
  pHandle->hSpeedRefPrevious = hSpeedRef;

  wTorqueFF = ((int32_t)hSpeedRefDelta * pHandle->wKaGain) / pHandle->wKaDivisor;
  wTorqueFF += ((int32_t)hSpeedRef * pHandle->wKvGain) / pHandle->wKvDivisor;

  wTorqueRefCombined = (int32_t)*hTorqueRef + (int32_t)pHandle->hTorqueRefFeedForward;

  int32_t wTorqueRefUpperLimit = (int32_t)pHandle->hTorqueRefUpperLimit;
  int32_t wTorqueRefLowerLimit = (int32_t)pHandle->hTorqueRefLowerLimit;
  int32_t wDischarge = 0;

  if (wTorqueRefCombined > wTorqueRefUpperLimit)
  {
    wDischarge = wTorqueRefUpperLimit - wTorqueRefCombined;
    wTorqueRefCombined = wTorqueRefUpperLimit;
  }
  else if (wTorqueRefCombined < wTorqueRefLowerLimit)
  {
    wDischarge = wTorqueRefLowerLimit - wTorqueRefCombined;
    wTorqueRefCombined = wTorqueRefLowerLimit;
  }

  pHandle->hTorqueRefFeedForward = (int16_t)(wTorqueFF + wDischarge);

  *hTorqueRef = (int16_t)wTorqueRefCombined;
}

/**
 * Gets the current feedforward torque reference.
 */
__weak int16_t SpeedFF_GetTorqueRefFeedForward(SpeedFF_Handle_t *pHandle)
{
  return pHandle->hTorqueRefFeedForward;
}

/**
 * Sets new feedforward constants (Ka and Kv) at runtime.
 */
__weak void SpeedFF_SetFFConstants(SpeedFF_Handle_t *pHandle, SpeedFF_TuningStruct_t sNewConstants)
{
  pHandle->wKaGain = sNewConstants.wKaGain;
  pHandle->wKvGain = sNewConstants.wKvGain;
}

/**
 * Gets the current feedforward constants (Ka and Kv).
 */
__weak SpeedFF_TuningStruct_t SpeedFF_GetFFConstants(SpeedFF_Handle_t *pHandle)
{
  SpeedFF_TuningStruct_t sLocalConstants;

  sLocalConstants.wKaGain = pHandle->wKaGain;
  sLocalConstants.wKvGain = pHandle->wKvGain;

  return sLocalConstants;
}
