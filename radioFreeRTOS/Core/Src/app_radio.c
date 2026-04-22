/*
 * app_radio.c
 *
 *  Created on: Apr 16, 2026
 *      Author: jdapo
 */

#include "app_radio.h"
#include "radio_driver.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "queue.h"


#define RF_FREQUENCY                                915000000 /* Hz */
#define TX_OUTPUT_POWER                             14        /* dBm */
#define LORA_BANDWIDTH                              0         /* Hz */
#define LORA_SPREADING_FACTOR                       7
#define LORA_CODINGRATE                             1
#define LORA_PREAMBLE_LENGTH                        8         /* Same for Tx and Rx */
#define LORA_SYMBOL_TIMEOUT                         5         /* Symbols */

const RadioLoRaBandwidths_t Bandwidths[] = { LORA_BW_125, LORA_BW_250, LORA_BW_500 };

void RadioOnDioIrq(RadioIrqMasks_t radioIrq);

PacketParams_t packetParams;
/* Definitions for radioQueue */
extern osMessageQueueId_t radioQueueHandle;
//uint8_t radioQueueBuffer[ 8 * sizeof( radio_event_t ) ];
//StaticQueue_t radioQueueControlBlock;
//const osMessageQueueAttr_t radioQueue_attributes = {
//  .name = "radioQueue",
//  .cb_mem = &radioQueueControlBlock,
//  .cb_size = sizeof(radioQueueControlBlock),
//  .mq_mem = &radioQueueBuffer,
//  .mq_size = sizeof(radioQueueBuffer)
//};

/**
  * @brief  Initialize the Sub-GHz radio and dependent hardware.
  * @retval None
  */
void radioInit(void)
{
  // Initialize the hardware (SPI bus, TCXO control, RF switch)
  SUBGRF_Init(RadioOnDioIrq);

  // Use DCDC converter if `DCDC_ENABLE` is defined in radio_conf.h
  // "By default, the SMPS clock detection is disabled and must be enabled before enabling the SMPS." (6.1 in RM0453)
  SUBGRF_WriteRegister(SUBGHZ_SMPSC0R, (SUBGRF_ReadRegister(SUBGHZ_SMPSC0R) | SMPS_CLK_DET_ENABLE));
  SUBGRF_SetRegulatorMode();

  // Use the whole 256-byte buffer for both TX and RX
  SUBGRF_SetBufferBaseAddress(0x00, 0x00);

  SUBGRF_SetRfFrequency(RF_FREQUENCY);
  SUBGRF_SetRfTxPower(TX_OUTPUT_POWER);
  SUBGRF_SetStopRxTimerOnPreambleDetect(false);

  SUBGRF_SetPacketType(PACKET_TYPE_LORA);

  SUBGRF_WriteRegister( REG_LR_SYNCWORD, ( LORA_MAC_PRIVATE_SYNCWORD >> 8 ) & 0xFF );
  SUBGRF_WriteRegister( REG_LR_SYNCWORD + 1, LORA_MAC_PRIVATE_SYNCWORD & 0xFF );

  ModulationParams_t modulationParams;
  modulationParams.PacketType = PACKET_TYPE_LORA;
  modulationParams.Params.LoRa.Bandwidth = Bandwidths[LORA_BANDWIDTH];
  modulationParams.Params.LoRa.CodingRate = (RadioLoRaCodingRates_t)LORA_CODINGRATE;
  modulationParams.Params.LoRa.LowDatarateOptimize = 0x00;
  modulationParams.Params.LoRa.SpreadingFactor = (RadioLoRaSpreadingFactors_t)LORA_SPREADING_FACTOR;
  SUBGRF_SetModulationParams(&modulationParams);

  packetParams.PacketType = PACKET_TYPE_LORA;
  packetParams.Params.LoRa.CrcMode = LORA_CRC_ON;
  packetParams.Params.LoRa.HeaderType = LORA_PACKET_VARIABLE_LENGTH;
  packetParams.Params.LoRa.InvertIQ = LORA_IQ_NORMAL;
  packetParams.Params.LoRa.PayloadLength = 0xFF;
  packetParams.Params.LoRa.PreambleLength = LORA_PREAMBLE_LENGTH;
  SUBGRF_SetPacketParams(&packetParams);

  //SUBGRF_SetLoRaSymbNumTimeout(LORA_SYMBOL_TIMEOUT);

  // WORKAROUND - Optimizing the Inverted IQ Operation, see DS_SX1261-2_V1.2 datasheet chapter 15.4
  // RegIqPolaritySetup @address 0x0736
  SUBGRF_WriteRegister( 0x0736, SUBGRF_ReadRegister( 0x0736 ) | ( 1 << 2 ) );
}

/**
  * @brief  Receive data trough SUBGHZSPI peripheral
  * @param  radioIrq  interrupt pending status information
  * @retval None
  */
void RadioOnDioIrq(RadioIrqMasks_t radioIrq)
{
  radio_event_t event;
  /* xHigherPriorityTaskWoken must be set to pdFALSE so it can later be detected if it
  was set to pdTRUE by any of the functions called within the interrupt. */
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  /* Switching on the IRQ given, we only look at the transmission related interrupts,
   * based off the interrupt, we assign an event that drives the behavior of the radio
   * task
  */
  switch (radioIrq)
  {
    case IRQ_TX_DONE:
      event = EVENT_TX_DONE;
      break;
    case IRQ_RX_TX_TIMEOUT:
      event = EVENT_TX_TIMEOUT;
      break;
    case IRQ_CRC_ERROR:
      event = EVENT_TX_ERROR;
      break;
    default:
      break;
  }
 xQueueSendFromISR(radioQueueHandle, &event, &xHigherPriorityTaskWoken);
 portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void radioTx(uint8_t *payload, uint8_t len)
{
	  SUBGRF_SetDioIrqParams( IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT,
							  IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT,
							  IRQ_RADIO_NONE,
							  IRQ_RADIO_NONE );
	  SUBGRF_SetSwitch(RFO_LP, RFSWITCH_TX);
	  // Workaround 5.1 in DS.SX1261-2.W.APP (before each packet transmission)
	  SUBGRF_WriteRegister(0x0889, (SUBGRF_ReadRegister(0x0889) | 0x04));
	  packetParams.Params.LoRa.PayloadLength = 0x4;
	  SUBGRF_SetPacketParams(&packetParams);
	  SUBGRF_SendPayload(payload, len, 0);
}



