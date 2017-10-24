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

HAL_PIN(saturated);
HAL_PIN(current);

HAL_PIN(tx_pos);
HAL_PIN(rx_pos);
HAL_PIN(tx_current);

HAL_PIN(home_vel_out);

HAL_PIN(home);

HAL_PIN(scale);
HAL_PIN(udc);


#define BOARD_ID  0x003

// ID 1
#if BOARD_ID == 0x001
#define TX_ADDRESS       0x0101     // address that is used for responding
#define RX_ADDRESS       0x0001       // address to listen to
#define MAX_SATURATED    0.2          // max. time in s position PID saturation is allowed
#define MAX_CURRENT      60           // max. motor current in 1/10 A
//#define POSITION_OFFSET  -0.256          // static position offset
#define POSITION_OFFSET  0.0          // use Zero Offset, Ros will handle it
#define SCALE            -2.0 * M_PI                // scaling factor for joint 1,3
#define HOMING_VEL       0.1		// For all else
#define PULLUP           GPIO_PULLUP  // GPIO_PULLDOWN for linear axis, GPIO_PULLUP for all else

// ID 2
#elif BOARD_ID == 0x002
#define TX_ADDRESS       0x0102     // address that is used for responding
#define RX_ADDRESS       0x0002       // address to listen to
#define MAX_SATURATED    0.2          // max. time in s position PID saturation is allowed
#define MAX_CURRENT      60           // max. motor current in 1/10 A
//#define POSITION_OFFSET  0.015          // static position offset
#define POSITION_OFFSET  0.0          // use Zero Offset, Ros will handle it
#define SCALE            -0.0895353   //-89.5353                // scaling factor for linear axis
#define HOMING_VEL       0.05		// For linear axis
#define PULLUP           GPIO_PULLDOWN  // GPIO_PULLDOWN for linear axis, GPIO_PULLUP for all else

// ALSO SET THE PID TO 1

// ID 3
#elif BOARD_ID == 0x003
#define TX_ADDRESS       0x0103     // address that is used for responding
#define RX_ADDRESS       0x0003       // address to listen to
#define MAX_SATURATED    0.2          // max. time in s position PID saturation is allowed
#define MAX_CURRENT      80           // max. motor current in 1/10 A
//#define POSITION_OFFSET  0.0204          // static position offset
#define POSITION_OFFSET  0.0          // use Zero Offset, Ros will handle it
#define SCALE            -2.0 * M_PI    // scaling factor for joint 1,3
// TODO PUT THIS BACK AFTER TESTING
#define HOMING_VEL       0.3		// For all else
//#define HOMING_VEL       0.1		// For all else
#define PULLUP           GPIO_PULLUP  // GPIO_PULLDOWN for linear axis, GPIO_PULLUP for all else

// ID 4
#elif BOARD_ID == 0x004
#define TX_ADDRESS       0x0104     // address that is used for responding
#define RX_ADDRESS       0x0004       // address to listen to
#define MAX_SATURATED    0.2          // max. time in s position PID saturation is allowed
#define MAX_CURRENT      80           // max. motor current in 1/10 A
//#define POSITION_OFFSET  2.5399          // static position offset
#define POSITION_OFFSET  0.0          // use Zero Offset, Ros will handle it
#define SCALE            2.0 * M_PI                // scaling factor for joint 4,5
#define HOMING_VEL       0.1		// For all else
#define PULLUP           GPIO_PULLUP  // GPIO_PULLDOWN for linear axis, GPIO_PULLUP for all else


// ID 5
#elif BOARD_ID == 0x005
#define TX_ADDRESS       0x0105     // address that is used for responding
#define RX_ADDRESS       0x0005       // address to listen to
#define MAX_SATURATED    0.2          // max. time in s position PID saturation is allowed
#define MAX_CURRENT      60           // max. motor current in 1/10 A
//#define POSITION_OFFSET  0.4744          // static position offset
#define POSITION_OFFSET  0.0          // use Zero Offset, Ros will handle it
#define SCALE            2.0 * M_PI                // scaling factor for joint 4,5
#define HOMING_VEL       0.1		// For all else
#define PULLUP           GPIO_PULLUP  // GPIO_PULLDOWN for linear axis, GPIO_PULLUP for all else


// ID 6
#elif BOARD_ID == 0x006
#define TX_ADDRESS       0x0106     // address that is used for responding
#define RX_ADDRESS       0x0006       // address to listen to
#define MAX_SATURATED    0.2          // max. time in s position PID saturation is allowed
#define MAX_CURRENT      90           // max. motor current in 1/10 A
//#define POSITION_OFFSET  -0.2          // static position offset ( Where the Index pin is situated
#define POSITION_OFFSET  0.0          // use Zero Offset, Ros will handle it
#define SCALE             -(2.0 * M_PI) / 24.3495               // scaling factor for joint
#define HOMING_VEL       0.1		// For all else
#define PULLUP           GPIO_PULLUP  // GPIO_PULLDOWN for linear axis, GPIO_PULLUP for all else
#endif


// Undervoltage threshold
#define UVLO             15.0

// Forward Decls
void ledRed(uint8_t state);
void ledBlue(uint8_t state);
void ledGreen(uint8_t state);
void testTransmit(unsigned char * foo);


uint8_t errors = 0b00000000; // 0: motor disconnected / 1: motor short / 2: position error / 3: overcurrent / 4: undervoltage / 5: overvoltage / 6: CAN timeout / 7: hardfault
uint8_t reported_errors = 0b00000000; // 0: motor disconnected / 1: motor short / 2: position error / 3: overcurrent / 4: undervoltage / 5: overvoltage / 6: CAN timeout / 7: hardfault
uint8_t current = 0;         // motor current in 1/10 A (100mA / LSB)

uint8_t running = 0;
volatile uint8_t mode = 0; //0 = pos, 1 = vel , 2 = homing positive , 3 = homin negative
uint8_t indexPin = 0;

uint32_t timeout = 0;

float pos = 0, vel = 0, pos_in = 0, vel_in = 0, txPos = 0;

uint8_t enable = 0;

volatile int8_t homing = 0;

float homingOffset = 0;

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

  GPIO_InitStruct.Pin = GPIO_PIN_2;  //Green
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_7;  //Blue
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_0;  //Red
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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

/*! CDhanges the enable state and sets the LED accordindly*/
void setEnable(int32_t enable_)
{
  if (enable_) // Make Sure that we only writes 1 or 0
  {
    enable = 1;
    ledRed(1);
  }
  else
  {
    enable = 0;
    ledRed(0);
  }
}

inline void setMode(uint32_t desired_mode)
{

  // Sets the mode to either
  // 0 = Position
  // 1 = Velocity
  // 2 = Positive Homing in Velcity mode
  // 3 = Inverse Homing in Velocity mode
  //mode = desired_mode;

  switch (desired_mode)
  {
    // Position
    case 0:
      mode = 0;
//     setPValue();  // Sets P accordingy
//     hal_parse("ypid0.vel_ext_cmd = vel1.vel"); // Set the comand to ... whatever

      if (BOARD_ID == 0x002 || BOARD_ID == 0x006)
      {
        hal_parse("ypid0.pos_p = 7"); //7 for 6 and 2 0.4 for all else
      }
      else
      {
        hal_parse("ypid0.pos_p = 0.4");
      }
      hal_parse("ypid0.vel_ext_cmd = vel1.vel"); // Set the comand to the the output of the velocity cascade i think?

      // Activate Blue LED -> Resulting in Violet when active and Blue when inactive
      ledBlue(1);
      ledGreen(0);
      testTransmit(00);
      break;
    // Velocity
    case 1:
      mode = 1;
      hal_parse("ypid0.pos_p = 0");
      hal_parse("ypid0.vel_ext_cmd = linrev0.cmd_d_out"); // Set the Velcotiy to the scaled input

      // Activate green LED -> Resulting in Yellow when active and Green when inactive
      ledBlue(0);
      ledGreen(1);
      testTransmit(01);
      break;
    // Positive homing (velocity)
    // Currently unused
    case 2:
      //mode = 1;
      hal_parse("ypid0.pos_p = 0");
      hal_parse("ypid0.vel_ext_cmd = linrev0.home_d_out");

      ledBlue(1);
      ledGreen(1);
      testTransmit(02);
      break;
    // Negative homing (veocity)
    // currently unused
    case 3:
      //mode = 1;
      hal_parse("ypid0.pos_p = 0"); //P Was already set
      hal_parse("ypid0.vel_ext_cmd = linrev0.home_neg_d_out");
      // Activate green LED -> Resulting in Yellow when active and Green when inactive
       ledBlue(1);
      ledGreen(1);

      testTransmit(03);
      break;
    // Velocity
    default:
      mode = 1;
      hal_parse("ypid0.pose_p = 0");
      hal_parse("ypid0.vel_ext_cmd = linrev0.cmd_d_out"); // Set the Velcotiy to the scaled input
      // Activate green LED -> Resulting in Yellow when active and Green when inactive
      ledBlue(0);
      ledGreen(0);
      testTransmit(04);
      break;
  }
}

// Resets the control values to initial defaults
void resetControlValues()
{

  // Reset homing to none Homing
  homing = 0;
  // Set the Velocity to 0
  vel = 0;

  // Set the p Values for the mode which we are currently in
  // If the current mode was one that is doing a reset -> Set to standrd velocity mode
  if (mode > 2)
  {
    mode = 0;
  }
  setMode(mode);

  // Disable by default (in case an enable is sent inthe samemessage it willbe evaluated after the error clear and shouldbe applied immediately
  setEnable(0);
}


void CAN_rdMsg (uint32_t ctrl, CAN_msg *msg)  {
  /* Read identifier information  */
  if ((CAN->sFIFOMailBox[0].RIR & CAN_ID_EXT) == 0)
  {
    msg->format = STANDARD_FORMAT;
    msg->id     = 0x000007FF & (CAN->sFIFOMailBox[0].RIR >> 21);
  }
  else
  {
    msg->format = EXTENDED_FORMAT;
    msg->id     = 0x1FFFFFFF & (CAN->sFIFOMailBox[0].RIR >> 3);
  }
  /* Read type information        */
  if ((CAN->sFIFOMailBox[0].RIR & CAN_RTR_REMOTE) == 0)
  {
    msg->type =   DATA_FRAME;
  }
  else
  {
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

  if (msg->len == 8)
  {

    // Bit 7 Clears the errors
    if ((msg->data[0] >> 7)  & 0x01)
    {
      //clear errors if they were present
      if (errors != 0)
      {
        //resetControlValues();
        hal_parse("start");
        errors = 0;
        reported_errors = 0;
        testTransmit(9);
      }
    }

    // Enable Flag
    if (((msg->data[0] >> 4)  & 0x01) && errors == 0)
    {
      setEnable(1);
    }
    else
    {
      setEnable(0);
    }

     // Set Position Flag
    if ((msg->data[0])  & 0x01) { //set position
      uint8_t b[] = {msg->data[4], msg->data[3], msg->data[2], msg->data[1]};
      // The actual position is Copied to the pos variable for later use
      memcpy(&pos, &b, sizeof(pos));
      vel = 0;
      setMode(0);
    }
    // Velocity is set
    else if ((msg->data[0] >> 1)  & 0x01) { //set velocity
      uint8_t b[] = {msg->data[4], msg->data[3], msg->data[2], msg->data[1]};
      memcpy(&vel, &b, sizeof(vel));

      setMode(1);
    }


    if (((msg->data[0] >> 2)  & 0x01)) { //home joint
    // Home is requested
      // Direction is set to a direction 1 = positive idrection -1 = negatice direction
      if ((msg->data[0] >> 3)  & 0x01) { //set direction
          setMode(2);
      }
      else {
        setMode(3);
      }
       homing = 3;
    }



    // Bit 5 requests the position to be sent back
    if ((msg->data[0] >> 5)  & 0x01) { //get position
      uint8_t b[] = {0,0,0,0};
      // It uses the txPos -> Make sure that tx pos contains the converted position
      memcpy(&b, &txPos, sizeof(txPos));
      hcan.pTxMsg = &myTxMessage;

      myTxMessage.DLC = 8;
      myTxMessage.StdId = TX_ADDRESS;
      myTxMessage.IDE = CAN_ID_STD;
      myTxMessage.Data[0] = msg->data[0];
      myTxMessage.Data[1] = b[3];
      myTxMessage.Data[2] = b[2];
      myTxMessage.Data[3] = b[1];
      myTxMessage.Data[4] = b[0];
      myTxMessage.Data[5] = current;
      myTxMessage.Data[6] = errors;
      myTxMessage.Data[7] = 0;

      HAL_CAN_Transmit(&hcan, HAL_MAX_DELAY);
    }

    else if ((msg->data[0] >> 6)  & 0x01) { //get velocity
	// Velocities are relaive, no conversion needed
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
      myTxMessage.Data[5] = current;
      myTxMessage.Data[6] = errors;
      myTxMessage.Data[7] = 0;

      HAL_CAN_Transmit(&hcan, HAL_MAX_DELAY);
    }
  }
}


void read_CAN(void)
{
  CAN_rdMsg (1, &CAN_RxMsg);
}

void testTransmit(unsigned char * foo) {
  hcan.pTxMsg = &myTxMessage;

  myTxMessage.DLC = 4;
  myTxMessage.StdId = TX_ADDRESS;
  myTxMessage.IDE = CAN_ID_STD;
  myTxMessage.Data[0] = 0xFF;
  myTxMessage.Data[1] = 0xFF;
  myTxMessage.Data[2] = 0xFF;
  myTxMessage.Data[3] = foo;

  HAL_CAN_Transmit(&hcan, HAL_MAX_DELAY);
}
COMMAND("cantx", testTransmit);

void sendError() {
//  if (errors != reported_errors)  // Only actively send out the errors if the reported errors are differnt (i.e. we did not send the error before)
//  {

    hcan.pTxMsg = &myTxMessage;

    myTxMessage.DLC = 8;
    myTxMessage.StdId = TX_ADDRESS;
    myTxMessage.IDE = CAN_ID_STD;
    myTxMessage.Data[0] = 0;
    myTxMessage.Data[1] = 0;
    myTxMessage.Data[2] = 0;
    myTxMessage.Data[3] = 0;
    myTxMessage.Data[4] = 0;
    myTxMessage.Data[5] = current;
    myTxMessage.Data[6] = errors;
    myTxMessage.Data[7] = 0;

    HAL_CAN_Transmit(&hcan, HAL_MAX_DELAY);

    reported_errors = errors;
//  }

    // RED!
    ledBlue(0);
    ledGreen(0);
    ledRed(1);
}

// Finished homing, report back to sender
void doneHoming() {
  hcan.pTxMsg = &myTxMessage;

  uint8_t b[] = {0,0,0,0};
  // the tx position is reported back -> Make sure this contains the right position
  memcpy(&b, &txPos, sizeof(txPos));
  hcan.pTxMsg = &myTxMessage;

  myTxMessage.DLC = 8;
  myTxMessage.StdId = TX_ADDRESS;
  myTxMessage.IDE = CAN_ID_STD;
  myTxMessage.Data[0] = 1 << 2;
  myTxMessage.Data[1] = b[3];
  myTxMessage.Data[2] = b[2];
  myTxMessage.Data[3] = b[1];
  myTxMessage.Data[4] = b[0];
  myTxMessage.Data[5] = current;
  myTxMessage.Data[6] = errors;
  myTxMessage.Data[7] = 0;

  HAL_CAN_Transmit(&hcan, HAL_MAX_DELAY);

  HAL_CAN_Transmit(&hcan, HAL_MAX_DELAY);
}

void ledRed(uint8_t state) {
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, !state);
}

void ledBlue(uint8_t state) {
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, !state);
}

void ledGreen(uint8_t state) {
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, !state);
}


static void nrt_init(volatile void * ctx_ptr, volatile hal_pin_inst_t * pin_ptr){
  // struct enc_ctx_t * ctx = (struct enc_ctx_t *)ctx_ptr;
  struct can_pin_ctx_t * pins = (struct can_pin_ctx_t *)pin_ptr;
  MX_CAN_Init();
  PIN(scale) = SCALE;
}

static void nrt_func(float period, volatile void * ctx_ptr, volatile hal_pin_inst_t * pin_ptr){
  // struct enc_ctx_t * ctx = (struct enc_ctx_t *)ctx_ptr;
  struct can_pin_ctx_t * pins = (struct can_pin_ctx_t *)pin_ptr;
  current = abs((int)(5.0 * PIN(current)));
  PIN(tx_current) = current;

  if (CAN->RF0R & CAN_RF0R_FMP0) {           /* message pending ?*/
    read_CAN();                         /* read the message               */
    CAN_RxRdy = 1;                              /*  set receive flag              */
    CAN_ReceiveMessage1 = CAN->sFIFOMailBox[0].RDLR;  /* read data */
    CAN_ReceiveMessage2 = CAN->sFIFOMailBox[0].RDLR >> 8;  /* read data */
    CAN->RF0R |= CAN_RF0R_RFOM0;            /* release FIFO */
  }

  /* In Case of an Error:
     The Error Flag is Set
     The Controller is stopped (hal_parse("stop"))
     The error is reported
     Uppon Clearing the error, the controller is enabled aggain
     Todo: Should we clear the enable Flag?
  */


 //Check for Saturation of controller
  if (PIN(saturated) > MAX_SATURATED && running) {
    //TX
//    hal_parse("stop");
    errors = errors | 1 << 2;
    sendError();
    printf("saturated!\n");
  }

   // Check for overcurrent error
  if (current > MAX_CURRENT && running) {
    //TX
//    hal_parse("stop");
    errors = errors | 1 << 3;
    sendError();
    printf("current!\n");
  }

   //Detecting an undervoltage is a special case as the controller needs to keep running.. (why?)

  if (PIN(udc) < UVLO && running)
  {
    //TX
    //hal_parse("stop");
    errors = errors | 1 << 4;
    setEnable(0);
    sendError();
    printf("UVLO!\n");
  }
  else
  {
    // In Case that the Voltage is fine (no emergency stop)
    // See if there are errors now ( which migt be the case from pressing the stop)
    if (errors)
    {
      hal_parse("stop");
      sendError();
      printf("UVLORESETTED!\n");
    }

  }


  // Actually write things based on the enable status
  PIN(enable) = enable;

  // homing state 2 -> Homing is finished -> Drive to theposition in question
  if (homing == 2) {

// homingOffset = the position we read when the index pin is reached
// pos -> the position we want to move to during reset
// Position offset = the Offset we are using to Offset the index pin aka determine the Zero Positon!
// Position taget during Home = Acutal position + Offset   -> 0.0 + 0ffset of 0.2 -> 0.2 (actual position) but reported would be =0.2-0.2 -> 0
// Position Target = Commanded Position - Offset
// Position Report = Actual position + Offset (-0.2)


    vel = 0;
    //pos = 0;
    PIN(vel) = vel;
    // Set the Output Position of this Module to the ucrrent homing offset (aka the current value that the encoders have)
    PIN(pos) = homingOffset;

    // Restore the previously set mode (to get rid of the homing params)
    setMode(mode);

    // if POS (note that pos is the current value given as position input. So during homing, this actually means it is a homingOffset to move to after the homing)
    if (pos != 0 && mode == 0) {

      if (pos > 0) {
        for (float i = 0; i < pos; i += 0.01) {
          sleep_ms(10);
		//float temp23 = i + homingOffset + POSITION_OFFSET;
	        // printf("MTH: %f, %f, %f, overall: %f , pos_in: %f \n",i, homingOffset , POSITION_OFFSET, temp23, pos_in);
           //PIN(pos) = i + homingOffset + POSITION_OFFSET;  // applying the position offset at this point is pointless as we are moving within the zeroed encoder world
	   PIN(pos) = i + homingOffset + POSITION_OFFSET;

        }
      }
      else {
        for (float i = 0; i > pos; i -= 0.01) {
          //printf("i: %f\n", i);
	  //printf("MTH -: %f, %f, %f\n",i, homingOffset , POSITION_OFFSET);
          sleep_ms(10);
          //PIN(pos) = i + homingOffset + POSITION_OFFSET;  // Same reason as above
	  PIN(pos) = i + homingOffset + POSITION_OFFSET;
        }
      }
    }

    homing = 0;
    doneHoming();
    printf("done homing!\n");
  }
}

void sleep_ms(uint32_t ms) {
  volatile uint32_t i;
  for (i = ms; i != 0; i--) {
    sleep_us(1000);
  }
}
void sleep_us(uint32_t us) {
  volatile uint32_t i;
  for (i = ((SystemCoreClock / 8000000) * us); i != 0; i--) {}
}


static void rt_func(float period, volatile void * ctx_ptr, volatile hal_pin_inst_t * pin_ptr){
  // struct enc_ctx_t * ctx = (struct enc_ctx_t *)ctx_ptr;
  struct can_pin_ctx_t * pins = (struct can_pin_ctx_t *)pin_ptr;
  indexPin = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10);
  PIN(home) = indexPin;



  // Check if any kind of homing is active
  if (homing != 0) {
      // If so, wait for the index pin to be set
      if (indexPin) {
	      // Stop the movement and set homing mode to 2 (position movement state)
        PIN(vel) = 0;
        homing = 2;
        // Save the current encoder position as homing offset -> this needs to be removed
        homingOffset = pos_in;
    }
  }
  else {
    // Normal operation -> Do the input conversion at this point
     // this is supposedly the position than we want to move to.. so the positoon that can is reporting
    // BEWARE! THIS IS THE INTERNAL position we want to go to (aka the encoder position )
   // Position Target = Commanded Position + homing offset (gets rid of the offset during homing) - Position offset (which we set to make it to zero
    PIN(pos) = pos + homingOffset + POSITION_OFFSET;   // Niklas Version.. Seems wrong but .. well
    //PIN(pos) = pos + homingOffset - POSITION_OFFSET;
    PIN(vel) = vel;
  }


  // pos_in is the FRAKING POSITION we have read from the linrev module
  pos_in = PIN(pos_in);
  vel_in = PIN(vel_in);

   // Reported position
// Position Report = Actual position ( internal position - homing offse)  + Offset
  txPos = pos_in - homingOffset - POSITION_OFFSET; // Niklas version i dont think this is right
  //txPos = pos_in - homingOffset + POSITION_OFFSET;

 // Report the tx position to other modules
  PIN(tx_pos) = txPos;
  PIN(rx_pos) = pos;

  PIN(home_vel_out) = HOMING_VEL;

  //timeout++;
}

static void rt_start(float period, volatile void * ctx_ptr, volatile hal_pin_inst_t * pin_ptr){
  // struct enc_ctx_t * ctx = (struct enc_ctx_t *)ctx_ptr;
  struct can_pin_ctx_t * pins = (struct can_pin_ctx_t *)pin_ptr;
  running = 1;
  errors = 0;
  reported_errors = 0;
}

static void rt_stop(float period, volatile void * ctx_ptr, volatile hal_pin_inst_t * pin_ptr){
  // struct enc_ctx_t * ctx = (struct enc_ctx_t *)ctx_ptr;
  struct can_pin_ctx_t * pins = (struct can_pin_ctx_t *)pin_ptr;
  running = 0;
}

hal_comp_t can_comp_struct = {
  .name = "can",
  .nrt = nrt_func,
  .rt = rt_func,
  .frt = 0,
  .nrt_init = nrt_init,
  .rt_start = rt_start,
  .frt_start = 0,
  .rt_stop = rt_stop,
  .frt_stop = 0,
  .ctx_size = 0,
  .pin_count = sizeof(struct can_pin_ctx_t) / sizeof(struct hal_pin_inst_t),
};
