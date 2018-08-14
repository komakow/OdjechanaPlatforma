/* Includes */
#include "main.h"

volatile int BTconnected = 0;                                               //flag used by pending while() checked BT status
uint8_t distance[4];                                                        //matrix contains value of distance read from all sonic sensor


int main(void)
{

  uint8_t flag_send = 0;

  if ((GPIO_InitializeAll() + PeripheryInit()) != 0)                        //config all of GPIO and all of periphery
  {
    gpioPinSetState(PORTD, PIN14, set);                                     //set red diode on discovery for signalizes error
  }
  else
  {
    while (1)
    {

      while (BTconnected == 0)                                                //wait for correct BT connection
      {
        BTDisconnected();                                                     //stop wheel and other periph if BT is not connected with stm
      }

      DistanceReadSend(distance);                                             //Measure and send by UART value of all sonic sensor
      Encoder_SpeedSend();                                                    //Measure and send by UART value of speed

      RunCar();                                                               //Set value of speed depending on read value from UART

    }  //end while
  }  //end else

  return 0;
}


/*
 * @brief Interrupt from BlueTooth status
 */
void EXTI4_IRQHandler(void)
{
  if (EXTI->PR & EXTI_PR_PR4)                                                                 //if interrupt is from PR4
  {
    BTconnected = !BTconnected;                                                               //set variable to reverse status

    if(!(GPIOD->IDR & GPIO_IDR_IDR_12) )                                                          //if BT is connected, set diode
    {
      gpioPinSetState(PORTD,PIN12, set);
    }
    else
    {
      gpioPinSetState(PORTD,PIN12, clear);
    }

    EXTI->PR |= EXTI_PR_PR4;                                                                  //clear pending request
  }
}
