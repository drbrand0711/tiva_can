/*
* This library sets timer A for time synchronization and getting timestamp
* Done by Mouna : ee22b100@smail.iitm.ac.in on Sep 15 2022
*/
#ifndef TIMESTAMP_H
#define TIMESTAMP_H

#include <Arduino.h>
#include <driverlib/sysctl.h>
#include <driverlib/timer.h>
#include <hw_timer.h>
#include <hw_ints.h>
#include <driverlib/interrupt.h>
#include <pin_map.h>
#include <cstdlib>

// CAN Message ID for time synchronization messages
#define CAN_TIME_SYNCHRONIZATION_MESSAGE_ID 500

// Set this to 1 if you want debug messages
#define CAN_TIMER_DEBUG 1 

//Maximum time between two time sync messages
#define MAX_DELAY_TIME_SYNC 40

#define IST_offset 19800000 //Offset to  change from UTC time format to IST  

#if CAN_TIMER_DEBUG
  uint8_t timer_debug_serial_msg[12] = {9, 9, 0, 0, 0, 0, 0, 0, 0, 0, 9, 9}; // 9 is for padding purposes
#endif

#define MSEC_IN_DAY 86400000

void tmr_int();



class time_keeper
{
private:

  //Stores last received timestamp
  long long CAN_timestamp_msg;

  //Stores base of the timer module used
  uint32_t CAN_tmr_base;

  //Stores the load value of timer
  uint32_t CAN_tmr_load_val;

public:

  //Interrupt for when timestamp msg not received for certain duration
  void CAN_tmr_int()
  {
    
    if(TimerIntStatus(this->CAN_tmr_base , false) == TIMER_TIMA_TIMEOUT)
    {
      TimerIntClear(this->CAN_tmr_base , TIMER_TIMA_TIMEOUT);

      //Update the CAN_timestamp_msg with Timer's Load Value
      this->CAN_timestamp_msg += int(this->CAN_tmr_load_val *1000.0 / SysCtlClockGet() );

      #if CAN_TIMER_DEBUG 
        //Sends the current microcontroller timestamp to check for latency in communication

        long long time_now = this->get_time();
        uint8_t *time_buf = (uint8_t *)&time_now;
        
        for (int j = 0; j < 8; j++)
          timer_debug_serial_msg[2 + j] = time_buf[j];
        Serial.write(timer_debug_serial_msg, 12);
      #endif

    }
  }

  //Get the timestamp relative to a single day of 24 hours
  uint32_t get_time_of_day()
  {
    long long complete_ts = this->get_time();

    uint32_t time_of_day = complete_ts % MSEC_IN_DAY;
    return time_of_day;
  }

  /**
   * Starts the timer
  */
  void timer_start()
  {
    TimerEnable(this->CAN_tmr_base, TIMER_BOTH);
  }

  /*
   * Setup the timer and timestamp related parameters
   * 
   * @param tmr_base :base address of the timer module. Default: TIMER 0
   * @param load_val :Loaded value in the timer. Default: 40 seconds 
  */
  time_keeper(uint32_t tmr_base = TIMER0_BASE, uint32_t load_val = MAX_DELAY_TIME_SYNC * (SysCtlClockGet()))
  {
    if (tmr_base == TIMER0_BASE)
    {
      SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
      while (!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0)) ;
    }
    else
    {
      //TODO: ?
    }
    TimerConfigure(tmr_base, TIMER_CFG_PERIODIC);
    this->CAN_tmr_base = tmr_base;
    this->CAN_tmr_load_val = load_val;
    this->CAN_timestamp_msg = 0;
  
    TimerLoadSet(this->CAN_tmr_base, TIMER_A, load_val);
    TimerIntEnable(this->CAN_tmr_base, TIMER_TIMA_TIMEOUT);
    TimerIntRegister(this->CAN_tmr_base, TIMER_A, tmr_int);
  }


  //get current time in milliseconds
  long long get_time(void)
  {
    // variable that holds the time elapsed since receiving the timestamp
    int elapsed = int((this->CAN_tmr_load_val - TimerValueGet(this->CAN_tmr_base, TIMER_A)) * 1000.0 / SysCtlClockGet());
    return this->CAN_timestamp_msg + elapsed;
  }


  //print epoch time in h:m:s:ms format
  #if CAN_TIMER_DEBUG
    void print_IST_Time(uint32_t time)
    {
      uint32_t IST_time = (time + IST_offset)%86400000;
      int msec = IST_time % 1000;
      IST_time /= 1000;
      int s = IST_time % 60;
      IST_time /= 60;
      int m = IST_time % 60;
      IST_time /= 60;
      int h = IST_time; 
      Serial.print(h);
      Serial.print(":");
      Serial.print(m);
      Serial.print(":");
      Serial.print(s);
      Serial.print(".");
      Serial.println(msec);
    }
  #endif
  
  /**
   * Callback for CAN timestamp msg
   * @param id identifier of the CAN msg 
   * @param buf buffer containing the timestamp
  */
  void CAN_timer_callback(int id, uint8_t buf[8])
  {
    #if CAN_TIMER_DEBUG
      long long time_now = this->get_time();
      uint8_t *time_buf = (uint8_t *)&time_now;
    
      for (int j = 0; j < 8; j++)
        timer_debug_serial_msg[2 + j] = time_buf[j];

      Serial.write(timer_debug_serial_msg, 12);
    #endif
    
    this->CAN_timestamp_msg = *(long long *)buf;
    TimerLoadSet(this->CAN_tmr_base, TIMER_A, this->CAN_tmr_load_val);  
  }

};


/**
 * CAN Timer class object for time synchronization purposes
*/
time_keeper CAN_tmr;

/**
 * Cannot register a class member function as an interrupt.
 * Hence calling a dummy function which calls the class member function.
 * TODO: Is there any simpler way to solve this problem? 
*/
void tmr_int()
{
  CAN_tmr.CAN_tmr_int();
}

#endif
