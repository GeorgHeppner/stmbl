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
/* We have no .data section, so these cannot be initialized */
/*

uint32_t h, l;
uint8_t len;
uint8_t *p;

static void do_tx(void)
{
	uint32_t timeout = TX_TIMEOUT;

	CAN_TDT0R(CANx) &= ~CAN_TDTxR_DLC_MASK;
	CAN_TDT0R(CANx) |= len;
	CAN_TDH0R(CANx) = h;
	CAN_TDL0R(CANx) = l;
	CAN_TI0R(CANx) |= CAN_TIxR_TXRQ;

	while ((CAN_TI0R(CANx) & CAN_TIxR_TXRQ) && (timeout-- > 0))
		;
}

static int do_rx()
{
	if (!(CAN_RF0R(CANx) & CAN_RF0R_FMP0_MASK))
		return 0;

	len = CAN_RDT0R(CANx) & CAN_RDTxR_DLC_MASK;
	h = CAN_RDH0R(CANx);
	l = CAN_RDL0R(CANx);

	CAN_RF0R(CANx) |= CAN_RF0R_RFOM0;
	while(CAN_RF0R(CANx) & CAN_RF0R_RFOM0)
		;

	return 1;
}
*/
static void nrt_init(volatile void * ctx_ptr, volatile hal_pin_inst_t * pin_ptr){
  // struct enc_ctx_t * ctx = (struct enc_ctx_t *)ctx_ptr;
  struct can_pin_ctx_t * pins = (struct can_pin_ctx_t *)pin_ptr;

  /**TIM1 GPIO Configuration
  PA8     ------> TIM1_CH1
  PA9     ------> TIM1_CH2
  */
  /*GPIO_InitTypeDef GPIO_InitStruct;
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_TIM1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 2000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  // htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;//TIM_ENCODERMODE_TI1??
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  HAL_TIM_Encoder_Init(&htim1, &sConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig);

  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_1 | TIM_CHANNEL_2);*/

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
