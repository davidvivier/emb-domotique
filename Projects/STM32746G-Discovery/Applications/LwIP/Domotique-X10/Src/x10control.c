
#include "x10control.h"


static char do_send;

static char state;
static signed char next_state[] = {1, -1, 3, -1, 5, -1, 7, -1};
static int state_threshold[] = {16, 8, 1, 0, 1, 1, 1, 3};
static char buf_send[32];


// base for each HOUSE, configured more precisely later by X10_Init()

#if HOUSE == HOUSE_A

extern char buf[] = {0,1,1,0, 0,0,0,0, 1,0,0,1, 1,1,1,1,  0,0,0,0, 0,0,0,0, 1,1,1,1, 1,1,1,1 };

#elif HOUSE == HOUSE_B

extern char buf[] = {0,1,1,1, 0,0,0,0, 1,0,0,0, 1,1,1,1,  0,0,0,0, 0,0,0,0, 1,1,1,1, 1,1,1,1 };

#endif

// A1 ON
//static char buf_on[]  = {0,1,1,0, 0,0,0,0, 1,0,0,1, 1,1,1,1,  0,0,0,0, 0,0,0,0, 1,1,1,1, 1,1,1,1 };
// A1 OFF
//static char buf_off[] = {0,1,1,0, 0,0,0,0, 1,0,0,1, 1,1,1,1,  0,0,1,0, 0,0,0,0, 1,1,0,1, 1,1,1,1 };


static int counter;
static int current_bit_index;


void X10_Init(void) {

  do_send = 0;

  int i = 0;
  for (i = 0; i < 32; i++) {
    //buf_send[i] = 0;
    //buf_send[i] = i%2;
  }

  state = 0;
  counter = 0;
  current_bit_index = -1;

  // config according to settings (HOUSE and UNIT)

  switch (UNIT % 9) {
    case 1:
      buf[17] = 0;
      buf[19] = 0;
      buf[20] = 0;
      break;
    case 2:
      buf[17] = 0;
      buf[19] = 1;
      buf[20] = 0;
      break;
    case 3:
      buf[17] = 0;
      buf[19] = 0;
      buf[20] = 1;
      break;
    case 4:
      buf[17] = 0;
      buf[19] = 1;
      buf[20] = 1;
      break;
    case 5:
      buf[17] = 1;
      buf[19] = 0;
      buf[20] = 0;
      break;
    case 6:
      buf[17] = 1;
      buf[19] = 1;
      buf[20] = 0;
      break;
    case 7:
      buf[17] = 1;
      buf[19] = 0;
      buf[20] = 1;
      break;
    case 8:
      buf[17] = 1;
      buf[19] = 1;
      buf[20] = 1;
      break;
  }

  buf[5] = 0; // UNIT < 9
  if (UNIT > 9) {
    buf[5] = 1;
  }

  // build footer : invert bits

  for (i = 0; i < 8; i++) {

    buf[8+i] = buf[0+i] == 0 ? 1 : 0;
    buf[24+i] = buf[16+i] == 0 ? 1 : 0;
  }

}

void RF_X10_Send_On(void) {
  //LCD_UsrLog ((char *)"Sending ON \n");
  buf[BIT_STATE_INDEX] = 0;
  buf[BIT_STATE_INDEX+8] = 1;
  int i = 0;
  for (i = 0; i < 32; i++) {
    buf_send[i] = buf[i];
  }
  do_send = 1;
}

void RF_X10_Send_Off(void) {
  //LCD_UsrLog ((char *)"Sending OFF \n");
  buf[BIT_STATE_INDEX] = 1;
  buf[BIT_STATE_INDEX+8] = 0;
  int i = 0;
  for (i = 0; i < 32; i++) {
    buf_send[i] = buf[i];
  }
  do_send = 1;
}


void TIM3_triggered(void) {
  
    if (do_send || (state == 2)) {
      counter++;
      if (counter >= state_threshold[state]) {
        // on change d'état
        if (state % 2 == 1) {

          // le changement d'état dépend de où on en est dans l'envoi

          if (state == 3) {
            // on a fini la transmission !

            //do_send = 0;
            state = 0;
            //counter = 0;
            current_bit_index = -1;
          } else if (current_bit_index == 31) {
            // on a fini d'envoyer les bits -> bit de fin
            counter = 0;
            do_send = 0;
            state = 2;

          } else {
            // on a fini d'envoyer le bit précédent
            //  on passe au bit suivant !
            current_bit_index++;

            if (buf_send[current_bit_index] == 0) {
              // send 0
              state = 4;
            } else {
              // send 1
              state = 6;
            }
          }

        } else {

          // on est au milieu d'une phase
          state = next_state[state];

        }

        // on change l'état d'émission selon l'état courant

        if (state%2 == 0) {
          // high 
          HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_SET);
        } else {
          // low
          HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_RESET);
        }
        counter = 0;
        BSP_LED_Toggle(LED1);
        //sprintf((char*)text, "new state = %d\n", state);
        //LCD_UsrLog ( (uint8_t *)text );
      } // chgt etat

    }

}

