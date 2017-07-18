#include "commands.h"
#include "hal.h"
#include "math.h"
#include "defines.h"
#include "angle.h"
#include "stm32f3xx_hal.h"

HAL_COMP(can);

HAL_PIN(pos);
HAL_PIN(vel);

//copied from florolf
/*
static void init_can(void)
{
	// configure ports
	GPIOB->MODER |= GPIO_MODE(8, GPIO_MODE_AF) |
		       GPIO_MODE(9, GPIO_MODE_AF);

	GPIOB->AFRH |= GPIO_AFR(0, GPIO_AF9) |
	              GPIO_AFR(1, GPIO_AF9);

	// configure CAN
	CAN_MCR(CANx) &= ~CAN_MCR_SLEEP;
	CAN_MCR(CANx) |= CAN_MCR_INRQ;

	while (!(CAN_MSR(CANx) & CAN_MSR_INAK))
		;

	// BPR = 4 -> Tq = 500 ns at 8MHz
	CAN_BTR(CANx) = CAN_BTR_SJW_2TQ | CAN_BTR_TS1_9TQ | CAN_BTR_TS2_6TQ | 3;

	generate_id(0);

	CAN_MCR(CANx) &= ~CAN_MCR_INRQ;
	while ((CAN_MSR(CANx) & CAN_MSR_INAK))
		;
}
*/

/* CAN init function */

CAN_HandleTypeDef hcan;

/* USER CODE BEGIN PV */
static CanTxMsgTypeDef myTxMessage;
static CanRxMsgTypeDef myRxMessage;
static CAN_FilterConfTypeDef myFilter;
/* USER CODE END PV */

/* CAN init function */
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
}

void testTransmit(char * foo) {
  printf("otter");
  hcan.pTxMsg = &myTxMessage;

  myTxMessage.DLC = 4;
  myTxMessage.StdId = 0x234;
  myTxMessage.IDE = CAN_ID_STD;
  myTxMessage.Data[0] = 0xDE;
  myTxMessage.Data[1] = 0xAD;
  myTxMessage.Data[2] = 0xBE;
  myTxMessage.Data[3] = 0xEF;

  HAL_CAN_Transmit_IT(&hcan);
  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_7);
}
COMMAND("cantx", testTransmit);

static void nrt_init(volatile void * ctx_ptr, volatile hal_pin_inst_t * pin_ptr){
  // struct enc_ctx_t * ctx = (struct enc_ctx_t *)ctx_ptr;
  struct can_pin_ctx_t * pins = (struct can_pin_ctx_t *)pin_ptr;
  MX_CAN_Init();
}

static void rt_func(float period, volatile void * ctx_ptr, volatile hal_pin_inst_t * pin_ptr){
  // struct enc_ctx_t * ctx = (struct enc_ctx_t *)ctx_ptr;
  struct can_pin_ctx_t * pins = (struct can_pin_ctx_t *)pin_ptr;

  PIN(pos) = 1;
  PIN(vel) = 0;
}

hal_comp_t can_comp_struct = {
  .name = "can",
  .nrt = 0,
  .rt = rt_func,
  .frt = 0,
  .nrt_init = nrt_init,
  .rt_start = 0,
  .frt_start = 0,
  .rt_stop = 0,
  .frt_stop = 0,
  .ctx_size = 0,
  .pin_count = sizeof(struct can_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
