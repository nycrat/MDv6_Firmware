/**
 ******************************************************************************
 * @file    speed_feed_forward_ctrl.h
 * @author  UBC Thunderbots
 * @brief   This module implements a feedforward component for speed control.
 ******************************************************************************
 */

#ifndef SPEED_FEEDFORWARDCTRL_H
#define SPEED_FEEDFORWARDCTRL_H

#include "mc_type.h"

#define SPEED_FEED_FORWARD_KA_DEFAULT 0
#define SPEED_FEED_FORWARD_KV_DEFAULT 0
#define SPEED_FEED_FORWARD_KS_DEFAULT 0
#define SPEED_FEED_FORWARD_KA_DIVISOR 128
#define SPEED_FEED_FORWARD_KV_DIVISOR 128

typedef struct
{
  int16_t hSpeedRefPrevious;      /* Previous speed reference for derivative calculation */
  int16_t hSpeedRefDelta;         /* Speed reference change rate (derivative) */
  int16_t hTorqueRefFeedForward;  /* Torque reference contribution from feedforward */
  int32_t wDefKaGain;             /* Default acceleration feedforward gain */
  int32_t wDefKvGain;             /* Default velocity feedforward gain */
  int32_t wDefKsGain;             /* Default static feedforward gain */
  int32_t wKaGain;                /* Acceleration feedforward gain */
  int32_t wKvGain;                /* Velocity feedforward gain */
  int32_t wKsGain;                /* Static gain for feedforward */
  int32_t wKaDivisor;             /* Divisor for acceleration feedforward gain */
  int32_t wKvDivisor;             /* Divisor for velocity feedforward gain */
  int16_t hTorqueRefUpperLimit;   /* Upper limit for the torque reference */
  int16_t hTorqueRefLowerLimit;   /* Lower limit for the torque reference */
} SpeedFF_Handle_t;

/**
 * Initializes the speed feedforward control module.
 */
void SpeedFF_Init(SpeedFF_Handle_t *pHandle);

/**
 * Clears the speed feedforward control internal state.
 */
void SpeedFF_Clear(SpeedFF_Handle_t *pHandle);

/**
 * Computes feedforward torque contribution based on speed reference and its derivative.
 */
void SpeedFF_ComputeTorqueReference(SpeedFF_Handle_t *pHandle, int16_t *hTorqueRef, int16_t hSpeedRef);

/**
 * Gets the current feedforward torque reference.
 */
int16_t SpeedFF_GetTorqueRefFeedForward(SpeedFF_Handle_t *pHandle);

void SpeedFF_SetKaGain(SpeedFF_Handle_t *pHandle, int32_t wKaGain);
void SpeedFF_SetKvGain(SpeedFF_Handle_t *pHandle, int32_t wKvGain);
void SpeedFF_SetKsGain(SpeedFF_Handle_t *pHandle, int32_t wKsGain);

int32_t SpeedFF_GetKaGain(SpeedFF_Handle_t *pHandle);
int32_t SpeedFF_GetKvGain(SpeedFF_Handle_t *pHandle);
int32_t SpeedFF_GetKsGain(SpeedFF_Handle_t *pHandle);

#endif /* SPEED_FEEDFORWARDCTRL_H */
