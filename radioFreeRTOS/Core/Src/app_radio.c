/*****************************************************************************
 * File Name    : app_radio.c
 *
 * Description  :
 * FreeRTOS LoRa radio transmission module implementation.
 *
 * This module is responsible for:
 *  - SX126x/SubGHz radio initialization
 *  - LoRa modulation configuration
 *  - IRQ event handling
 *  - RTOS radio event synchronization
 *  - Payload transmission
 *
 * Radio Flow:
 *  1. Aggregator task sends payload to radio queue
 *  2. Radio task dequeues payload
 *  3. Payload transmission is initiated
 *  4. IRQ handler captures radio events
 *  5. IRQ events are forwarded to RTOS queue
 *  6. Radio task processes transmission result
 *
 * Created on : April 16, 2026
 * Author     : Jeremiah Apolista
 *****************************************************************************/

#include "app_radio.h"
#include "radio_driver.h"

#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "queue.h"

#include "usart.h"

/*****************************************************************************/
/* Radio Configuration                                                       */
/*****************************************************************************/

/* Operating RF center frequency */
#define RF_FREQUENCY                915000000

/* RF output transmit power in dBm */
#define TX_OUTPUT_POWER             17

/* LoRa bandwidth selection index */
#define LORA_BANDWIDTH              0

/* LoRa spreading factor */
#define LORA_SPREADING_FACTOR       7

/* LoRa coding rate */
#define LORA_CODINGRATE             1

/* LoRa preamble length */
#define LORA_PREAMBLE_LENGTH        8

/* LoRa symbol timeout */
#define LORA_SYMBOL_TIMEOUT         5

/*****************************************************************************/
/* Radio Lookup Tables                                                       */
/*****************************************************************************/

/*
 * Lookup table mapping bandwidth indices
 * to SX126x driver bandwidth values.
 */
const RadioLoRaBandwidths_t Bandwidths[] =
{
    LORA_BW_125,
    LORA_BW_250,
    LORA_BW_500
};

/*****************************************************************************/
/* Function Prototypes                                                       */
/*****************************************************************************/

void RadioOnDioIrq(RadioIrqMasks_t radioIrq);

/*****************************************************************************/
/* FreeRTOS Queue Handles                                                    */
/*****************************************************************************/

/*
 * Queue used to receive payloads from
 * the aggregator task.
 */
QueueHandle_t xRadioQueue;

/*
 * Queue used to transfer radio IRQ events
 * from ISR context to the radio task.
 */
QueueHandle_t radioQueueHandle;

/*****************************************************************************/
/* Radio Driver Variables                                                    */
/*****************************************************************************/

/*
 * SX126x packet configuration structure.
 * Stores LoRa packet configuration parameters.
 */
PacketParams_t packetParams;

/*****************************************************************************/
/* Radio Initialization                                                      */
/*****************************************************************************/

/*
 ******************************************************************************
 * Function Name : radioInit
 *
 * Description:
 * Initializes the SX126x/SubGHz radio peripheral and configures
 * LoRa modulation parameters.
 *
 * Configuration Includes:
 *  - RF frequency
 *  - TX output power
 *  - packet format
 *  - CRC configuration
 *  - spreading factor
 *  - coding rate
 *  - bandwidth
 *
 * Parameters:
 *  None
 *
 * Returns:
 *  None
 *
 * Notes:
 *  This routine must execute before any radio transmissions occur.
 ******************************************************************************
 */
void radioInit(void)
{
    /*
     * Initialize SubGHz radio hardware.
     *
     * Includes:
     *  - SPI interface
     *  - TCXO control
     *  - RF switch configuration
     */
    SUBGRF_Init(RadioOnDioIrq);

    /*
     * Enable SMPS clock detection prior to enabling
     * the DC-DC converter regulator mode.
     */
    SUBGRF_WriteRegister(
            SUBGHZ_SMPSC0R,
            (SUBGRF_ReadRegister(SUBGHZ_SMPSC0R)
            | SMPS_CLK_DET_ENABLE));

    /* Enable DC-DC regulator mode */
    SUBGRF_SetRegulatorMode();

    /*
     * Configure entire 256-byte radio buffer
     * for both TX and RX operations.
     */
    SUBGRF_SetBufferBaseAddress(0x00, 0x00);

    /* Configure RF operating frequency */
    SUBGRF_SetRfFrequency(RF_FREQUENCY);

    /* Configure TX output power */
    SUBGRF_SetRfTxPower(TX_OUTPUT_POWER);

    /*
     * Continue RX timeout timer even after
     * preamble detection.
     */
    SUBGRF_SetStopRxTimerOnPreambleDetect(false);

    /* Configure LoRa packet mode */
    SUBGRF_SetPacketType(PACKET_TYPE_LORA);

    /* Configure LoRa private sync word */
    SUBGRF_WriteRegister(
            REG_LR_SYNCWORD,
            (LORA_MAC_PRIVATE_SYNCWORD >> 8) & 0xFF);

    SUBGRF_WriteRegister(
            REG_LR_SYNCWORD + 1,
            LORA_MAC_PRIVATE_SYNCWORD & 0xFF);

    /*********************************************************************/
    /* LoRa Modulation Parameters                                        */
    /*********************************************************************/

    ModulationParams_t modulationParams;

    modulationParams.PacketType = PACKET_TYPE_LORA;

    modulationParams.Params.LoRa.Bandwidth =
            Bandwidths[LORA_BANDWIDTH];

    modulationParams.Params.LoRa.CodingRate =
            (RadioLoRaCodingRates_t)LORA_CODINGRATE;

    modulationParams.Params.LoRa.LowDatarateOptimize = 0x00;

    modulationParams.Params.LoRa.SpreadingFactor =
            (RadioLoRaSpreadingFactors_t)LORA_SPREADING_FACTOR;

    SUBGRF_SetModulationParams(&modulationParams);

    /*********************************************************************/
    /* LoRa Packet Parameters                                            */
    /*********************************************************************/

    packetParams.PacketType = PACKET_TYPE_LORA;

    packetParams.Params.LoRa.CrcMode = LORA_CRC_ON;

    packetParams.Params.LoRa.HeaderType =
            LORA_PACKET_VARIABLE_LENGTH;

    packetParams.Params.LoRa.InvertIQ = LORA_IQ_NORMAL;

    packetParams.Params.LoRa.PayloadLength = 0xFF;

    packetParams.Params.LoRa.PreambleLength =
            LORA_PREAMBLE_LENGTH;

    SUBGRF_SetPacketParams(&packetParams);

    /*
     * Optional symbol timeout configuration.
     * Currently disabled.
     */
    //SUBGRF_SetLoRaSymbNumTimeout(LORA_SYMBOL_TIMEOUT);

    /*
     * SX126x workaround:
     * Optimize inverted IQ operation.
     *
     * Reference:
     * DS_SX1261-2_V1.2 datasheet section 15.4
     */
    SUBGRF_WriteRegister(
            0x0736,
            SUBGRF_ReadRegister(0x0736) | (1 << 2));
}

/*****************************************************************************/
/* Radio IRQ Handler                                                         */
/*****************************************************************************/

/*
 ******************************************************************************
 * Function Name : RadioOnDioIrq
 *
 * Description:
 * Interrupt callback invoked by the SX126x driver whenever
 * a radio IRQ event occurs.
 *
 * Responsibilities:
 *  - Decode radio IRQ source
 *  - Convert IRQ into RTOS radio event
 *  - Forward event to radio task queue
 *
 * Parameters:
 *  radioIrq : Pending radio interrupt source
 *
 * Returns:
 *  None
 *
 * Notes:
 *  This function executes in ISR context.
 ******************************************************************************
 */
void RadioOnDioIrq(RadioIrqMasks_t radioIrq)
{
    radio_event_t event;

    /*
     * Tracks whether a higher-priority task
     * should be context-switched immediately.
     */
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    /*
     * Translate SX126x IRQ source into
     * application-level radio event.
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
            /*
             * Ignore unsupported radio IRQs.
             */
            return;
    }

    /*
     * Forward radio event from ISR context
     * to radio task event queue.
     */
    xQueueSendFromISR(radioQueueHandle,
                      &event,
                      &xHigherPriorityTaskWoken);

    /*
     * Perform immediate context switch if
     * higher-priority task was awakened.
     */
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/*****************************************************************************/
/* Radio Transmission Function                                               */
/*****************************************************************************/

/*
 ******************************************************************************
 * Function Name : radioTx
 *
 * Description:
 * Configures the SX126x radio for LoRa transmission and
 * initiates payload transmission.
 *
 * Parameters:
 *  payload : Pointer to payload buffer
 *  len     : Payload length in bytes
 *
 * Returns:
 *  None
 *
 * Notes:
 *  Payload length must match the serialized telemetry packet size.
 ******************************************************************************
 */
void radioTx(uint8_t *payload, uint8_t len)
{
    /*
     * Configure radio IRQ sources for
     * transmission-related events.
     */
    SUBGRF_SetDioIrqParams(
            IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT,
            IRQ_TX_DONE | IRQ_RX_TX_TIMEOUT,
            IRQ_RADIO_NONE,
            IRQ_RADIO_NONE);

    /* Configure RF switch for TX operation */
    SUBGRF_SetSwitch(RFO_LP, RFSWITCH_TX);

    /*
     * SX126x workaround:
     * Required before each packet transmission.
     *
     * Reference:
     * DS.SX1261-2.W.APP section 5.1
     */
    SUBGRF_WriteRegister(
            0x0889,
            (SUBGRF_ReadRegister(0x0889) | 0x04));

    /* Update payload length field */
    packetParams.Params.LoRa.PayloadLength = len;

    /* Apply updated packet configuration */
    SUBGRF_SetPacketParams(&packetParams);

    /* Begin payload transmission */
    SUBGRF_SendPayload(payload, len, 0);
}

/*****************************************************************************/
/* RTOS Radio Task                                                           */
/*****************************************************************************/

/*
 ******************************************************************************
 * Function Name : RadioTask
 *
 * Description:
 * FreeRTOS task responsible for LoRa payload transmission
 * and radio event processing.
 *
 * Task Flow:
 *  1. Wait for payload from aggregator queue
 *  2. Initiate radio transmission
 *  3. Wait for transmission completion IRQ
 *  4. Retry transmission on failure
 *
 * Synchronization:
 *  - Receives payloads from aggregator task
 *  - Receives IRQ events from ISR queue
 *
 * Parameters:
 *  argument : Unused
 *
 * Returns:
 *  None
 ******************************************************************************
 */
void RadioTask(void *argument)
{
    /* Binary telemetry payload buffer */
    uint8_t payload[64];

    /* Radio event container */
    radio_event_t event;

    /* UART debug buffer */
    char msg[48];
    int len;

    while (1)
    {
        /*
         * Wait indefinitely for telemetry payload
         * from aggregator task.
         */
        xQueueReceive(xRadioQueue,
                      payload,
                      portMAX_DELAY);

        len = snprintf(msg,
                       sizeof(msg),
                       "Payload Received");

        HAL_UART_Transmit(&huart2,
                          (uint8_t *)msg,
                          len,
                          100);

        len = snprintf(msg,
                       sizeof(msg),
                       "Transmitting Payload");

        HAL_UART_Transmit(&huart2,
                          (uint8_t *)msg,
                          len,
                          100);

        /* Begin LoRa payload transmission */
        radioTx(payload, sizeof(payload));

        /*
         * Wait for radio IRQ event indicating
         * transmission completion or failure.
         */
        if (xQueueReceive(radioQueueHandle,
                          &event,
                          pdMS_TO_TICKS(2000)))
        {
            /*
             * Retry payload transmission if
             * transmission did not complete successfully.
             */
            if (event != EVENT_TX_DONE)
            {
                len = snprintf(msg,
                               sizeof(msg),
                               "Transmission Failed");

                HAL_UART_Transmit(&huart2,
                                  (uint8_t *)msg,
                                  len,
                                  100);

                /* Retry payload transmission */
                radioTx(payload, sizeof(payload));
            }
            else
            {
                len = snprintf(msg,
                               sizeof(msg),
                               "Transmit Complete");

                HAL_UART_Transmit(&huart2,
                                  (uint8_t *)msg,
                                  len,
                                  100);
            }
        }
    }
}


