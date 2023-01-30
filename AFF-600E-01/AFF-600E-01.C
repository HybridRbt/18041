/*******************************************************************************
 *               Copyright(c) 2016, Mactronix Inc.
 *               All rights reserved
 *
 *               File:         AFF-600N-V3.c
 *               Description:  main program for AFF-600N-V2 of AT89S51
 *               Author:       Alan Chi
 *               Date:         May 2, 2016
 *               SW:           AFF-600N-03
 *
 *               05/02/2016    rev 1.000      Alan Chi
 *               11/07/2016    rev 2.000      Alan Chi
 ******************************************************************************/
#include <REG52.H>                /* special function register declarations   */
                                  /* for the intended 8051 derivative         */
#include <stdio.h>                /* prototype declarations for I/O functions */

/*  User  */
sbit Flat_Position_Bit_1 = P0^0; //BROWN
sbit Flat_Position_Bit_2 = P0^1; //RED
sbit Flat_Position_Bit_4 = P0^2; //ORENGE
sbit Flat_Position_Bit_8 = P0^3; //GREEN

sbit Flat_Position_Switch_1 = P0^0; //BROWN
sbit Flat_Position_Switch_2 = P0^1; //RED

sbit Boat_Switch = P0^7; //0 = trig

sbit Stepper_Phase_1 = P1^0; //BLACK
sbit Stepper_Phase_2 = P1^1; //GREEN
sbit Stepper_Phase_3 = P1^2; //RED
sbit Stepper_Phase_4 = P1^3; //BLUE

sbit DC_Motor = P1^4;      //1 = move
sbit Led_Indicator = P1^5; //1 = on

sbit Flat_Fine_Tune_1 = P2^0; //DIP SW1
sbit Flat_Fine_Tune_2 = P2^1; //DIP SW2
sbit Flat_Fine_Tune_3 = P2^2; //DIP SW3
sbit Flat_Fine_Tune_4 = P2^3; //DIP SW4
sbit Flat_Align_Twice = P2^4;

sbit Front_Roller_Down_Switch = P2^7; //0 = trig
sbit Front_Roller_Up_Switch = P2^6;   //0 = trig
sbit Flat_Position_Switch = P2^5;     //0 = trig

#define T0_time                  80
#define T1_time                  50000
#define POSITION_COUNT_PITCH     255
#define POSITION_COUNT_06       -5
#define ACCDECSTEPS              20   //0-20
#define ADDSTEPS                 2
#define TUNE_RATIO               20
#define ROLLER_SPEED             4    //4-10: 4 = fastest, 10 = slowest
#define NO_ERROR                 0    //good: no error happened
#define ERROR                   -1

#define CW                       1
#define CCW                      0

unsigned int T0_count = 0;
//unsigned int T1_count = 0;

unsigned int direction = 0;           //Direction: 1 = clockwise, 0 = counter-clockwise
unsigned int flat_position = 0;
unsigned int flat_position_temp = 0;
unsigned int flat_count_fine_tune = 0;

int position_steps = 0;
int position_count_fine_tune = 0;
int relative_position_count = 0;
int absolute_position_count = 0;
int total_count = 0;
int accdec_count = 0;                 //AccDec: 0-AccDecSteps

int Position_Count_Pitch;
int Position_Count_06;
int Tune_Ratio;
int Roller_Speed;
int AccDecSteps;
int AddSteps;

unsigned int i = 0;
unsigned int error = 0;
unsigned int home_error = 0;

unsigned int init_flag = 0;
unsigned int boat_flag = 0;
unsigned int mode_flag = 0; //Mode: 1 = absolute,  0 = relative
unsigned int done_flag = 0; //Done: 1 = move done, 0 = moving
unsigned int loop_flag = 0; //Loop: 1 = over loop, 0 = not over loop

unsigned char P1_TEMP;
unsigned char TH0_TEMP;
unsigned char TL0_TEMP;

//code unsigned char FFW[]={0x01,0x03,0x02,0x06,0x04,0x0c,0x08,0x09}; //clockwise
//code unsigned char REV[]={0x09,0x08,0x0c,0x04,0x06,0x02,0x03,0x01}; //counter-clockwise
code unsigned char FFW[]={0x05,0x01,0x09,0x08,0x0a,0x02,0x06,0x04};   //clockwise
code unsigned char REV[]={0x04,0x06,0x02,0x0a,0x08,0x09,0x01,0x05};   //counter-clockwise

void Timer0_ISR();
int  Delay(unsigned int delay, unsigned int check);
int  RollerHome(void);
void ShutdownMotor(void);
void StopMotor(void);
void StartMotor(void);
int  RelativeMove(int position, int speed, int accdec);
int  AbsoluteMove(int position, int speed, int accdec);

int IsBoatPlaced(void);

/*-----------------------------------------------------------------------------
Function: Timer0_ISR()
          Setup Timer 0 for 100us at 12MHz.
------------------------------------------------------------------------------*/
void Timer0_ISR() interrupt 1
{
  TH0 = TH0_TEMP;
  TL0 = TL0_TEMP;

  T0_count++;

  if(T0_count >= Roller_Speed + AccDecSteps - accdec_count)
  {
    T0_count = 0;

    if(i > 7)
    {
      i = 0;
    }

    P1_TEMP = P1;
    P1_TEMP &= 0xF0;

    if(direction)
    {
      P1_TEMP |= FFW[i];
    }
    else
    {
      P1_TEMP |= REV[i];
    }

    P1 = P1_TEMP;

    i++;
    relative_position_count++;

    if(mode_flag)
    {
      if(direction == CW) //clockwise
      {
        if(relative_position_count > AddSteps)
        {
          absolute_position_count++;

          if(absolute_position_count >= Position_Count_Pitch*16)
          {
            absolute_position_count = 0;
          }
        }
      }
      else //counter-clockwise
      {
        if(relative_position_count > AddSteps)
        {
          absolute_position_count--;

          if(absolute_position_count < 0)
          {
            absolute_position_count = Position_Count_Pitch*16 - 1;
          }
        }
      }
    }

    if(relative_position_count == total_count)
    {
      TR0 = 0;
    }

    if(relative_position_count <= AccDecSteps) //speed changed for accelaration
    {
      accdec_count++;
    }
    else if(total_count - relative_position_count <= AccDecSteps) //speed changed for decelaration
    {
      accdec_count--;
    }
  }
}

/*-----------------------------------------------------------------------------
Function: Timer1_ISR()
          Setup Timer 1 for 50ms at 12MHz.
------------------------------------------------------------------------------*/
/*
void Timer1_ISR() interrupt 3 using 2
{
  TH1 = (65536-T1_time)/256;
  TL1 = (65536-T1_time)%256;

  T1_count++;

  if(T1_count >= 10)
  {
    T1_count = 0;
  }
}
*/

/*-----------------------------------------------------------------------------
Function: Delay(unsigned int delay, unsigned int check)
------------------------------------------------------------------------------*/
int Delay(unsigned int delay, unsigned int check)
{
  unsigned int temp;

  if(check)
  {
    Led_Indicator = 1;
  }

  for(temp=0;temp<delay;temp++)            //Delay
  {
    if(Boat_Switch && check)
    {
      error = 1;
      break;
    }
  }

  if(check)
  {
    Led_Indicator = 0;
  }

  if(error)
  {
    return ERROR;
  }

  return NO_ERROR;
}

/*-----------------------------------------------------------------------------
Function: RollerHome(void)
------------------------------------------------------------------------------*/
int RollerHome(void)
{
  home_error = 0;

  if(!Front_Roller_Up_Switch)
  {
    return NO_ERROR;
  }

  Led_Indicator = 1;
  DC_Motor = 1;                            //Roller up

  while(Front_Roller_Up_Switch)
  {
    if(Boat_Switch)
    {
      home_error = 1;
      break;
    }
  }

  Led_Indicator = 0;
  DC_Motor = 0;                            //Roller stop at up

  if(home_error)
  {
    return ERROR;
  }

  return NO_ERROR;
}

/*-----------------------------------------------------------------------------
Function: RollerDown(void)
------------------------------------------------------------------------------*/
int RollerDown(void)
{
  if(!Front_Roller_Down_Switch)
  {
    return NO_ERROR;
  }

  Led_Indicator = 1;
  DC_Motor = 1;                            //Roller down

  while(Front_Roller_Down_Switch)
  {
    if(Boat_Switch)
    {
      error = 1;
      break;
    }
  }

  Led_Indicator = 0;
  DC_Motor = 0;                            //Roller stop at down

  if(error)
  {
    return ERROR;
  }

  return NO_ERROR;
}

/*-----------------------------------------------------------------------------
Function: ShutdownMotor(void)
------------------------------------------------------------------------------*/
void ShutdownMotor(void)
{
  Led_Indicator = 0;
  TR0 = 0;

  i = 0;
  T0_count = 0;
  TH0 = TH0_TEMP;
  TL0 = TL0_TEMP;

  P1_TEMP = P1;
  P1_TEMP &= 0xF0;
  P1 = P1_TEMP;
}

/*-----------------------------------------------------------------------------
Function: StopMotor(void)
------------------------------------------------------------------------------*/
void StopMotor(void)
{
  Led_Indicator = 0;
  TR0 = 0;

  i = 0;
  T0_count = 0;
  TH0 = TH0_TEMP;
  TL0 = TL0_TEMP;
}

/*-----------------------------------------------------------------------------
Function: StartMotor(void)
------------------------------------------------------------------------------*/
void StartMotor(void)
{
  relative_position_count = 0;
  accdec_count = 0;

  i = 0;
  T0_count = 0;
  TH0 = TH0_TEMP;
  TL0 = TL0_TEMP;

  Led_Indicator = 1;
  TR0 = 1;
}

/*-----------------------------------------------------------------------------
Function: IsBoatPlaced(void)
------------------------------------------------------------------------------*/
int IsBoatPlaced(void)
{
  // only used to start operation
  if (!Boat_Switch) {
    Delay(1000, 0); // delay 100ms 

    if (!Boat_Switch) {
      return 1;
    }
  }

  return 0;
}

/*-----------------------------------------------------------------------------
Function: RelativeMove(int position, int speed, int accdec)
------------------------------------------------------------------------------*/
int RelativeMove(int position, int speed, int accdec)
{
  mode_flag = 0;

  Roller_Speed = speed;
  AccDecSteps = accdec;
  AddSteps = 0;

  total_count = position;

  if(total_count == 0)
  {
    return NO_ERROR;
  }
  else if(total_count < 0)
  {
    total_count *= -1;
    direction = CCW;
  }
  else if(total_count > 0)
  {
    direction = CW;
  }

  total_count = total_count + AddSteps;
  StartMotor();

  while(relative_position_count < total_count)
  {
    if(Boat_Switch)
    {
      error = 1;
      break;
    }
  }

  if(error)
  {
    StopMotor();
    return ERROR;
  }

  return NO_ERROR;
}

/*-----------------------------------------------------------------------------
Function: AbsoluteMove(int position, int speed, int accdec)
------------------------------------------------------------------------------*/
int AbsoluteMove(int position, int speed, int accdec)
{
  mode_flag = 1;
  done_flag = 0;

  Roller_Speed = speed;
  AccDecSteps = accdec;
  AddSteps = ADDSTEPS;

  total_count = position - absolute_position_count;

  if(total_count == 0)
  {
    done_flag = 1;
    return NO_ERROR;
  }
  else if(total_count < 0)
  {
    total_count *= -1;

    if(!loop_flag)
    {
      direction = CCW;
    }
    else
    {
      total_count = Position_Count_Pitch*16 - total_count;
      direction = CW;
    }
  }
  else if(total_count > 0)
  {
    if(!loop_flag)
    {
      direction = CW;
    }
    else
    {
      total_count = Position_Count_Pitch*16 - total_count;
      direction = CCW;
    }
  }

  total_count = total_count + AddSteps;
  StartMotor();

  while(relative_position_count < total_count)
  {
    if(Boat_Switch)
    {
      error = 1;
      break;
    }

/*  flat_position = P0;
    flat_position &= 0x0F;

    if(flat_position_temp != flat_position)
    {
      flat_position_temp = flat_position;
      break;
    } */
  }

  if(error)
  {
    StopMotor();
    return ERROR;
  }

  done_flag = 1;
  return NO_ERROR;
}

/*-----------------------------------------------------------------------------
Function: main(void)
------------------------------------------------------------------------------*/
void main(void)
{
  P0 = 0xFF;                              //as input
  P2 = 0xFF;                              //as input

  P1 = 0x00;                              //as output

  TH0_TEMP = (65536-T0_time)/256;
  TL0_TEMP = (65536-T0_time)%256;

  TMOD = 0x11;
  TH0 = TH0_TEMP;
  TL0 = TL0_TEMP;
  ET0 = 1;
//TH1 = (65536-T1_time)/256;
//TL1 = (65536-T1_time)%256;
//ET1 = 1;
  EA  = 1;
  TR0 = 0;
//TR1 = 0;

  Delay(10000, 0);

/*-----------------------------------------------------------------------------
  Power up, go back to home
------------------------------------------------------------------------------*/
  flat_position = 0;
  flat_position_temp = 0;
  flat_count_fine_tune = 0;

  position_count_fine_tune = 0;
  relative_position_count = 0;
  absolute_position_count = 0;

  i = 0;
  error = 0;
  home_error = 0;

  T0_count = 0;
//T1_count = 0;
  init_flag = 0;
  boat_flag = 0;
  mode_flag = 0;
  done_flag = 0;
  loop_flag = 0;

  Led_Indicator = 0;
  DC_Motor = 0;

  ShutdownMotor();

//RollerHome();

/*-----------------------------------------------------------------------------
  When load a boat, it works one cycle.
------------------------------------------------------------------------------*/
  while(1)
  {
    error = 0;
    Led_Indicator = 0;

    Position_Count_Pitch = POSITION_COUNT_PITCH;
    Position_Count_06 = POSITION_COUNT_06;
    Tune_Ratio = TUNE_RATIO;

    ShutdownMotor();

    flat_count_fine_tune = P2;
    flat_count_fine_tune = ~flat_count_fine_tune;
    flat_count_fine_tune &= 0x0F;
    position_count_fine_tune = (flat_count_fine_tune & 0x07)*Tune_Ratio;

    if(flat_count_fine_tune & 0x08)
    {
      position_count_fine_tune *= -1;
    }

    boat_flag = 0;

    while((IsBoatPlaced() == 1) && !boat_flag)
    {
      boat_flag = 1;

      if(home_error)
      {
        if(RollerHome() != NO_ERROR)
        {
          break;
        }
      }

      if(RollerDown() != NO_ERROR)
      {
        break;
      }

      if(Delay(10000, 1) != NO_ERROR)
      {
        break;
      }

      position_steps = -Position_Count_Pitch;

      if(RelativeMove(position_steps, ROLLER_SPEED, ACCDECSTEPS) != NO_ERROR) //counter-clockwise
      {
        break;
      }

      if(RollerHome() != NO_ERROR)
      {
        break;
      }

      position_steps = Position_Count_Pitch;

      if(RelativeMove(position_steps, ROLLER_SPEED, ACCDECSTEPS) != NO_ERROR) //clockwise
      {
        break;
      }

      if(RollerDown() != NO_ERROR)
      {
        break;
      }

      if(Delay(10000, 1) != NO_ERROR)
      {
        break;
      }

      position_steps = -Position_Count_Pitch*18;

      if(RelativeMove(position_steps, ROLLER_SPEED, ACCDECSTEPS) != NO_ERROR) //counter-clockwise
      {
        break;
      }

      if(Delay(10000, 1) != NO_ERROR)
      {
        break;
      }

      if(RollerHome() != NO_ERROR)
      {
        break;
      }

      //reset absolute count
      absolute_position_count = 0;

      while(1)
      {
        if(!Flat_Position_Switch)
        {
          flat_position = 8;  //at 12 o'clock
        }
        else
        {
          flat_position = 0;  //at 6 o'clock
        }

        flat_position_temp = flat_position;

        if(Delay(10000, 1) != NO_ERROR)
        {
          break;
        }

        if(!Flat_Position_Switch)
        {
          flat_position = 8;  //at 12 o'clock
        }
        else
        {
          flat_position = 0;  //at 6 o'clock
        }

        if(flat_position_temp != flat_position)
        {
          continue;
        }

        //calculate position_steps
        position_steps = position_count_fine_tune + Position_Count_06 + Position_Count_Pitch*(flat_position);

        if(position_steps < 0)
        {
          position_steps += Position_Count_Pitch*16;
          loop_flag = 1;
        }
        else if(position_steps >= Position_Count_Pitch*16)
        {
          position_steps -= Position_Count_Pitch*16;
          loop_flag = 1;
        }
        else
        {
          loop_flag = 0;
        }

        if(AbsoluteMove(position_steps, ROLLER_SPEED, ACCDECSTEPS) != NO_ERROR)
        {
          break;
        }

        if(Delay(10000, 0) != NO_ERROR)
        {
          break;
        }

        if(done_flag)
        {
          Led_Indicator = !Led_Indicator;
        }

        if(Boat_Switch)
        {
          break;
        }
      }
    }
  }
}
