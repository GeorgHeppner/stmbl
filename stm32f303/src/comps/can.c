#include "commands.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"
#include "stm32f3xx_hal.h"

HAL_COMP(can);

HAL_PIN(pos);
HAL_PIN(vel);
HAL_PIN(enable);

HAL_PIN(pos_in);
HAL_PIN(vel_in);

#define TX_ADDRESS  0x0001
#define RX_ADDRESS  0x0006

uint8_t errors = 0b00000000; // 0: motor disconnected / 1: motor short / 3: position error / 3: overcurrent / 4: undervoltage / 5: overvoltage / 6: CAN timeout / 7: hardfault
uint8_t current = 0;         // motor current in 1/10 A (100mA / LSB)

uint8_t mode = 0; //0 = pos, 1 = vel

float pos = 0, vel = 0, pos_in = 0, vel_in = 0;
uint8_t enable = 0;

typedef struct  {
  unsigned int   id;                    /* 29 bit identifier */
  unsigned char  data[8];               /* Data field */
  unsigned char  len;                   /* Length of data field in bytes */
  unsigned char  format;                /* 0 - STANDARD, 1- EXTENDED IDENTIFIER */
  unsigned char  type;                  /* 0 - DATA FRAME, 1 - REMOTE FRAME */
} CAN_msg;

CAN_msg       CAN_RxMsg;                      /* CAN message for receiving */

uint32_t      CAN_RxRdy = 0;              /* CAN HW received a message */

static uint32_t CAN_filterIdx[2] = {0,0};        /* static variable for the filter index */

uint32_t      CAN_msgId     = 0;

CAN_HandleTypeDef hcan;

uint32_t CAN_ReceiveMessage1 = 0;
uint32_t CAN_ReceiveMessage2 = 0;


#define STANDARD_FORMAT  0
#define EXTENDED_FORMAT  1

#define DATA_FRAME       0
#define REMOTE_FRAME     1


static CanTxMsgTypeDef myTxMessage;
static CanRxMsgTypeDef myRxMessage;
static CAN_FilterConfTypeDef myFilter;

static void MX_CAN_Init(void)
{

  //HAL_CAN_MspInit(&hcan);

  GPIO_InitTypeDef GPIO_InitStruct;
  __HAL_RCC_CAN1_CLK_ENABLE();

  /**CAN GPIO Configuration
  PA11     ------> CAN_RX
  PA12     ------> CAN_TX
  */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_CAN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  hcan.Instance = CAN;
  hcan.Init.Prescaler = 6;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SJW = CAN_SJW_1TQ;
  hcan.Init.BS1 = CAN_BS1_8TQ;
  hcan.Init.BS2 = CAN_BS2_3TQ;
  hcan.Init.TTCM = DISABLE;
  hcan.Init.ABOM = ENABLE;
  hcan.Init.AWUM = DISABLE;
  hcan.Init.NART = DISABLE;
  hcan.Init.RFLM = DISABLE;
  hcan.Init.TXFP = DISABLE;

  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }

  myFilter.FilterNumber = 0;
  myFilter.FilterMode = CAN_FILTERMODE_IDMASK;
  myFilter.FilterScale = CAN_FILTERSCALE_32BIT;
  myFilter.FilterIdHigh = RX_ADDRESS << 5;
  myFilter.FilterIdLow = 0x0000;
  myFilter.FilterMaskIdHigh = 0xFFFF;
  myFilter.FilterMaskIdLow = 0xFFFF;
  myFilter.FilterFIFOAssignment = 0;
  myFilter.FilterActivation = ENABLE;

  HAL_CAN_ConfigFilter(&hcan,&myFilter);

  hcan.pRxMsg= &myRxMessage;
}

void CAN_rdMsg (uint32_t ctrl, CAN_msg *msg)  {
                                              /* Read identifier information  */
  if ((CAN->sFIFOMailBox[0].RIR & CAN_ID_EXT) == 0) {
    msg->format = STANDARD_FORMAT;
    msg->id     = 0x000007FF & (CAN->sFIFOMailBox[0].RIR >> 21);
  } else {
    msg->format = EXTENDED_FORMAT;
    msg->id     = 0x1FFFFFFF & (CAN->sFIFOMailBox[0].RIR >> 3);
  }
                                              /* Read type information        */
  if ((CAN->sFIFOMailBox[0].RIR & CAN_RTR_REMOTE) == 0) {
    msg->type =   DATA_FRAME;
  } else {
    msg->type = REMOTE_FRAME;
  }
                                              /* Read number of rec. bytes    */
  msg->len     = (CAN->sFIFOMailBox[0].RDTR      ) & 0x0F;
                                              /* Read data bytes              */
  msg->data[0] = (CAN->sFIFOMailBox[0].RDLR      ) & 0xFF;
  msg->data[1] = (CAN->sFIFOMailBox[0].RDLR >>  8) & 0xFF;
  msg->data[2] = (CAN->sFIFOMailBox[0].RDLR >> 16) & 0xFF;
  msg->data[3] = (CAN->sFIFOMailBox[0].RDLR >> 24) & 0xFF;

  msg->data[4] = (CAN->sFIFOMailBox[0].RDHR      ) & 0xFF;
  msg->data[5] = (CAN->sFIFOMailBox[0].RDHR >>  8) & 0xFF;
  msg->data[6] = (CAN->sFIFOMailBox[0].RDHR >> 16) & 0xFF;
  msg->data[7] = (CAN->sFIFOMailBox[0].RDHR >> 24) & 0xFF;

  CAN->RF0R |= CAN_RF0R_RFOM0;             /* Release FIFO 0 output mailbox */

  if (msg->len == 8) {
    if ((msg->data[0])  & 0x01) { //set position
      uint8_t b[] = {msg->data[4], msg->data[3], msg->data[2], msg->data[1]};
      memcpy(&pos, &b, sizeof(pos));

      if (mode == 1) {
        mode = 0;
        hal_parse("ypid0.pos_p = 10");

        hal_parse("ypid0.vel_ext_cmd = vel1.vel");
      }
    }

    else if ((msg->data[0] >> 1)  & 0x01) { //set velocity
      uint8_t b[] = {msg->data[4], msg->data[3], msg->data[2], msg->data[1]};
      memcpy(&vel, &b, sizeof(vel));

      if (mode == 0) {
        mode = 1;
        hal_parse("ypid0.pos_p = 0");

        hal_parse("ypid0.vel_ext_cmd = can0.vel");
      }
    }

    else { //polling
      printf("polling...\n");
    }

    /*if ((msg->data[0] >> 2)  & 0x01)) { //home joint
      if ((msg->data[0] >> 3)  & 0x01)) { //set direction

      }
      else {

      }
    }*/

    if ((msg->data[0] >> 4)  & 0x01) { //arm motor
      enable = 1;
    }
    else {
      enable = 0;
    }

    if ((msg->data[0] >> 5)  & 0x01) { //get position
      uint8_t b[] = {0,0,0,0};
      memcpy(&b, &pos_in, sizeof(pos_in));
      hcan.pTxMsg = &myTxMessage;

      myTxMessage.DLC = 8;
      myTxMessage.StdId = TX_ADDRESS;
      myTxMessage.IDE = CAN_ID_STD;
      myTxMessage.Data[0] = msg->data[0];
      myTxMessage.Data[1] = b[3];
      myTxMessage.Data[2] = b[2];
      myTxMessage.Data[3] = b[1];
      myTxMessage.Data[4] = b[0];
      myTxMessage.Data[5] = 0;
      myTxMessage.Data[6] = 0;
      myTxMessage.Data[7] = 0;

      HAL_CAN_Transmit(&hcan, HAL_MAX_DELAY);
    }

    else if ((msg->data[0] >> 6)  & 0x01) { //get velocity
      uint8_t b[] = {0,0,0,0};
      memcpy(&b, &vel_in, sizeof(vel_in));

      myTxMessage.DLC = 8;
      myTxMessage.StdId = TX_ADDRESS;
      myTxMessage.IDE = CAN_ID_STD;
      myTxMessage.Data[0] = msg->data[0];
      myTxMessage.Data[1] = b[3];
      myTxMessage.Data[2] = b[2];
      myTxMessage.Data[3] = b[1];
      myTxMessage.Data[4] = b[0];
      myTxMessage.Data[5] = 0;
      myTxMessage.Data[6] = 0;
      myTxMessage.Data[7] = 0;

      HAL_CAN_Transmit(&hcan, HAL_MAX_DELAY);
    }


  }
}


void read_CAN(void)
{
  CAN_rdMsg (1, &CAN_RxMsg);
}

void testTransmit(char * foo) {
  hcan.pTxMsg = &myTxMessage;

  myTxMessage.DLC = 4;
  myTxMessage.StdId = TX_ADDRESS;
  myTxMessage.IDE = CAN_ID_STD;
  myTxMessage.Data[0] = 0xDE;
  myTxMessage.Data[1] = 0xAD;
  myTxMessage.Data[2] = 0xBE;
  myTxMessage.Data[3] = 0xEF;

  HAL_CAN_Transmit(&hcan, HAL_MAX_DELAY);
}
COMMAND("cantx", testTransmit);

static void nrt_init(volatile void * ctx_ptr, volatile hal_pin_inst_t * pin_ptr){
  // struct enc_ctx_t * ctx = (struct enc_ctx_t *)ctx_ptr;
  struct can_pin_ctx_t * pins = (struct can_pin_ctx_t *)pin_ptr;
  MX_CAN_Init();
}

static void nrt_func(float period, volatile void * ctx_ptr, volatile hal_pin_inst_t * pin_ptr){
  // struct enc_ctx_t * ctx = (struct enc_ctx_t *)ctx_ptr;
  struct can_pin_ctx_t * pins = (struct can_pin_ctx_t *)pin_ptr;


  if (CAN->RF0R & CAN_RF0R_FMP0) {           /* message pending ?*/
    read_CAN();                         /* read the message               */
    CAN_RxRdy = 1;                              /*  set receive flag              */
    CAN_ReceiveMessage1 = CAN->sFIFOMailBox[0].RDLR;  /* read data */
    CAN_ReceiveMessage2 = CAN->sFIFOMailBox[0].RDLR >> 8;  /* read data */
    CAN->RF0R |= CAN_RF0R_RFOM0;            /* release FIFO */
  }

  PIN(pos) = pos;
  PIN(vel) = vel;
  PIN(enable) = enable;

  pos_in = PIN(pos_in);
  vel_in = PIN(vel_in);
}

hal_comp_t can_comp_struct = {
  .name = "can",
  .nrt = nrt_func,
  .rt = 0,//rt_func,
  .frt = 0,
  .nrt_init = nrt_init,
  .rt_start = 0,
  .frt_start = 0,
  .rt_stop = 0,
  .frt_stop = 0,
  .ctx_size = 0,
  .pin_count = sizeof(struct can_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
