
#include "x10control.h"


static char do_send;

static char state;
static signed char next_state[] = {1, -1, 3, -1, 5, -1, 7, -1};
static int state_threshold[] = {16, 8, 1, 0, 1, 1, 1, 3};
static char buf_send[32];

static char buf_on[]  = {0,1,1,0, 0,0,0,0, 1,0,0,1, 1,1,1,1,  0,0,0,0, 0,0,0,0, 1,1,1,1, 1,1,1,1 };

static char buf_off[] = {0,1,1,0, 0,0,0,0, 1,0,0,1, 1,1,1,1,  0,0,1,0, 0,0,0,0, 1,1,0,1, 1,1,1,1 };

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

}

void RF_X10_Send_On(void) {
  //LCD_UsrLog ((char *)"Sending ON \n");
  int i = 0;
  for (i = 0; i < 32; i++) {
    buf_send[i] = buf_on[i];
  }
  do_send = 1;
}

void RF_X10_Send_Off(void) {
  //LCD_UsrLog ((char *)"Sending OFF \n");
  int i = 0;
  for (i = 0; i < 32; i++) {
    buf_send[i] = buf_off[i];
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

