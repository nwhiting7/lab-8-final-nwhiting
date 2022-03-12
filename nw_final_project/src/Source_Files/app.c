/***************************************************************************//**
 * @file
 * @brief Top level application functions
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/
#include "blink.h"
#include "glib.h"
#include "app.h"
#include "os.h"
#include "stdlib.h"
#include <string.h>

/*******************************************************************************
 ***************************  LOCAL VARIABLES   ********************************
 ******************************************************************************/
bool pb0, pb1;
slider_side cap_dir;

#define PB0_TASK_STACK_SIZE      512
#define PB0_TASK_PRIO            20
#define PB1_TASK_STACK_SIZE      512
#define PB1_TASK_PRIO            20
#define CAP_TASK_STACK_SIZE      512
#define CAP_TASK_PRIO            20
#define EM_TASK_STACK_SIZE      512
#define EM_TASK_PRIO            21
#define LED_TASK_STACK_SIZE      512
#define LED_TASK_PRIO            20
#define SPEED_SETPOINT_TASK_SIZE           512
#define SPEED_SETPOINT_TASK_PRIO           20
#define VEHICLE_DIRECTION_TASK_SIZE           2048
#define VEHICLE_DIRECTION_TASK_PRIO           20
#define VEHICLE_MONITOR_TASK_SIZE           512
#define VEHICLE_MONITOR_TASK_PRIO           20
#define LCD_DISPLAY_TASK_SIZE           512
#define LCD_DISPLAY_TASK_PRIO           20

#define BUTTON1_IFC 0x80
#define BUTTON2_IFC 0x40

#define BTN0 0
#define BTN1 1

static OS_TCB cap_tcb;
static OS_TCB em_tcb;
static OS_TCB led_tcb;
static CPU_STK cap_stack[CAP_TASK_STACK_SIZE];
static CPU_STK em_stack[EM_TASK_STACK_SIZE];
static CPU_STK led_stack[EM_TASK_STACK_SIZE];

static OS_TCB speed_setpoint_tcb;
static CPU_STK speed_setpoint_stack[SPEED_SETPOINT_TASK_SIZE];
static OS_TCB vehicle_direction_tcb;
static CPU_STK vehicle_direction_stack[VEHICLE_DIRECTION_TASK_SIZE];
static OS_TCB vehicle_direction_tcb;
static CPU_STK vehicle_direction_stack[VEHICLE_DIRECTION_TASK_SIZE];
static OS_TCB vehicle_monitor_tcb;
static CPU_STK vehicle_monitor_stack[VEHICLE_MONITOR_TASK_SIZE];
static OS_TCB lcd_display_tcb;
static CPU_STK lcd_display_stack[LCD_DISPLAY_TASK_SIZE];

/*******************************************************************************
 *********************   LOCAL FUNCTION PROTOTYPES   ***************************
 ******************************************************************************/
static void cap_task(void *arg);
static void em_task(void *arg);
static void led_task(void *arg);

static void speed_setpoint_task(void *arg);
static void vehicle_direction_task(void *arg);
static void vehicle_monitor_task(void *arg);
static void lcd_display_task(void *arg);

static OS_Q App_MessageQ;
static OS_SEM  App_Semaphore;
static OS_SEM  lcd_semaphore;
static OS_FLAG_GRP flags_vm;
static OS_FLAG_GRP flags_led;
static OS_TMR  vd_timer;
static OS_MUTEX  mutex_vd;
static OS_MUTEX  mutex_ss;

static GLIB_Context_t glibContext;

typedef enum{
  NONE_PRESS,
  speed_update,
  direction_update,
  led_update_0_0 = 4,
  led_update_0_1 = 8,
  led_update_1_0 = 16,
  led_update_1_1 = 32,
} event_flag;

typedef enum{
  no_dir,
  hard_left,
  left,
  right,
  hard_right,
}direction;

typedef struct app_message {
    bool btn0_press,
    btn1_press,
    is_btn;

    int cap_dir;
} APP_MESSAGE;

typedef struct Vehicle_Speed{
  uint32_t speed;
  uint32_t incr_count;
  uint32_t decr_count;
}v_speed;

typedef struct Vehicle_Direction{
  direction curr_dir;
  uint32_t curr_dir_hold;
  uint32_t left_turns;
  uint32_t right_turns;
}v_direction;

typedef struct FIFO_BUFFER{
  bool increment;
  bool decrement;
}fifo;

v_speed global_speed;
v_direction global_direction;
fifo ind_speed;
struct node_t* pb_data;
/***************************************************************************//**
 * @brief Initializes all tasks for Lab3.
 *
 * @details Tasks initialized are pb0 task, pb1 task, capsense task, and idle/energy mode task.
 *
 ******************************************************************************/
void app_init(void)
{
  gpio_open();
  //pb0_init();
  //pb1_init();
  //cap_init();

  speed_setpoint_init();
  vehicle_direction_init();
  vehicle_monitor_init();
  lcd_display_init();
  em_init();
  led_init();

  lab6_init();
}

/***************************************************************************//**
 * @brief Initializes capsense task.
 *
 * @details Task initializes with priority of 20, stack of size 96, its own control block.
 *
 ******************************************************************************/
void cap_init(void)
{
  RTOS_ERR err;

  // Create Blink Task
  OSTaskCreate(&cap_tcb,
               "capsense task",
               cap_task,
               DEF_NULL,
               CAP_TASK_PRIO,
               &cap_stack[0],
               (CAP_TASK_STACK_SIZE / 10u),
               CAP_TASK_STACK_SIZE,
               0u,
               0u,
               DEF_NULL,
               (OS_OPT_TASK_STK_CLR),
               &err);
  EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
}

/***************************************************************************//**
 * @brief Initializes idle/energy mode task.
 *
 * @details Task initializes with priority of 20, stack of size 96, its own control block.
 *
 ******************************************************************************/
void em_init(void)
{
  RTOS_ERR err;

  // Create Blink Task
  OSTaskCreate(&em_tcb,
               "Energy Mode Task",
               em_task,
               DEF_NULL,
               EM_TASK_PRIO,
               &em_stack[0],
               (EM_TASK_STACK_SIZE / 10u),
               EM_TASK_STACK_SIZE,
               0u,
               0u,
               DEF_NULL,
               (OS_OPT_TASK_STK_CLR),
               &err);
  EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
}

/***************************************************************************//**
 * @brief Initializes led task.
 *
 * @details Task initializes with priority of 20, stack of size 96, its own control block.
 *
 ******************************************************************************/
void led_init(void)
{
  RTOS_ERR err;

  OSTaskCreate(&led_tcb,
               "LED Output task",
               led_task,
               DEF_NULL,
               LED_TASK_PRIO,
               &led_stack[0],
               (LED_TASK_STACK_SIZE / 10u),
               LED_TASK_STACK_SIZE,
               0u,
               0u,
               DEF_NULL,
               (OS_OPT_TASK_STK_CLR),
               &err);
  EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
}

/***************************************************************************//**
 * @brief Initializes speed setpoint task.
 *
 * @details Task initializes with priority of 20, stack of size 96, its own control block.
 *
 ******************************************************************************/
void speed_setpoint_init(void)
{
  RTOS_ERR err;

  OSTaskCreate(&speed_setpoint_tcb,
               "Speed Setpoint Task",
               speed_setpoint_task,
               DEF_NULL,
               SPEED_SETPOINT_TASK_PRIO,
               &speed_setpoint_stack[0],
               (SPEED_SETPOINT_TASK_SIZE / 10u),
               SPEED_SETPOINT_TASK_SIZE,
               0u,
               0u,
               DEF_NULL,
               (OS_OPT_TASK_STK_CLR),
               &err);
  EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
}

/***************************************************************************//**
 * @brief Task handler for speed setpoint task.
 *
 * @details What for semaphore to be posted every 100ms.
 *
 ******************************************************************************/
static void speed_setpoint_task(void *arg)
{
    PP_UNUSED_PARAM(arg);
    RTOS_ERR err;


    while (1)
    {


                                           /* Block until another task signals this task.     */
      OSSemPend(&App_Semaphore,        /* Pointer to user-allocated semaphore.    */
      0,                 /* Wait for a maximum of 1000 OS Ticks.    */
      OS_OPT_PEND_BLOCKING, /* Task will block.                        */
      DEF_NULL,             /* Timestamp is not used.                  */
      &err);

      OSMutexPend(&mutex_ss,             /*   Pointer to user-allocated mutex.         */
                   0,                  /*   Wait for a maximum of 1000 OS Ticks.     */
                   OS_OPT_PEND_BLOCKING,  /*   Task will block.                         */
                   DEF_NULL,              /*   Timestamp is not used.                   */
                  &err);

      int received_data = peek(&pb_data);

      if(received_data == 2){
          global_speed.incr_count++;
          global_speed.speed += 5;
          pop(&pb_data);
    }
      else if(received_data == 1){
          pop(&pb_data);

           if(global_speed.speed != 0){
          global_speed.decr_count++;
          global_speed.speed -= 5;
           }
      }

      OSMutexPost(&mutex_ss,         /*   Pointer to user-allocated mutex.         */
                   OS_OPT_POST_1,     /*   Only wake up highest-priority task.      */
                  &err);

        OSFlagPost(&flags_vm,             /*   Pointer to user-allocated event flag.    */
                  speed_update,            /*   Application Flag A bitmask.              */
                  OS_OPT_POST_FLAG_SET,  /*   Set the flag.                            */
                  &err);


    }
}

/***************************************************************************//**
 * @brief Initializes vehicle direction task.
 *
 * @details Task initializes with priority of 20, stack of size 96, its own control block.
 *
 ******************************************************************************/
void vehicle_direction_init(void)
{
  RTOS_ERR err;

  OSTaskCreate(&vehicle_direction_tcb,
               "Vehicle Direction Task",
               vehicle_direction_task,
               DEF_NULL,
               VEHICLE_DIRECTION_TASK_PRIO,
               &vehicle_direction_stack[0],
               (VEHICLE_DIRECTION_TASK_SIZE / 10u),
               VEHICLE_DIRECTION_TASK_SIZE,
               0u,
               0u,
               DEF_NULL,
               (OS_OPT_TASK_STK_CLR),
               &err);
  EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
}

/***************************************************************************//**
 * @brief Task handler for vehicle direction task.
 *
 * @details Wait for semaphore that is poseted every 100 ms.
 *
 ******************************************************************************/
static void vehicle_direction_task(void *arg)
{
    PP_UNUSED_PARAM(arg);
    RTOS_ERR err;

    OSTmrStart(&vd_timer,  /*   Pointer to user-allocated timer.      */
               &err);

    while (1)
    {
        OSSemPend(&lcd_semaphore,        /* Pointer to user-allocated semaphore.    */
              0,                 /* Wait for a maximum of 1000 OS Ticks.    */
              OS_OPT_PEND_BLOCKING, /* Task will block.                        */
              DEF_NULL,             /* Timestamp is not used.                  */
              &err);

        CAPSENSE_Sense();

        direction dir = no_dir;
        if(CAPSENSE_getPressed(CH0)) dir = hard_left;
        if(CAPSENSE_getPressed(CH1)){
            if(dir == no_dir) dir = left;
        }
        if(CAPSENSE_getPressed(CH2)){
            if(dir == no_dir) dir = right;
            else{
                dir = no_dir;
                return;
            }
        }
        if(CAPSENSE_getPressed(CH3)){
            if(dir == no_dir) dir = hard_right;
            else dir = no_dir;
        }

        OSMutexPend(&mutex_vd,             /*   Pointer to user-allocated mutex.         */
                     0,                  /*   Wait for a maximum of 1000 OS Ticks.     */
                     OS_OPT_PEND_BLOCKING,  /*   Task will block.                         */
                     DEF_NULL,              /*   Timestamp is not used.                   */
                    &err);

        global_direction.curr_dir = dir;

        if((dir == left) || (dir == hard_left)){
            global_direction.left_turns++;
        }
        else if((dir == right) || (dir == hard_right)){
            global_direction.right_turns++;
        }



        OSMutexPost(&mutex_vd,         /*   Pointer to user-allocated mutex.         */
                     OS_OPT_POST_1,     /*   Only wake up highest-priority task.      */
                    &err);

        OSFlagPost(&flags_vm,             /*   Pointer to user-allocated event flag.    */
                  direction_update,            /*   Application Flag A bitmask.              */
                  OS_OPT_POST_FLAG_SET,  /*   Set the flag.                            */
                  &err);


    }


}

/***************************************************************************//**
 * @brief Initializes vehicle monitor task.
 *
 * @details Task initializes with priority of 20, stack of size 96, its own control block.
 *
 ******************************************************************************/
void vehicle_monitor_init(void)
{
  RTOS_ERR err;

  OSTaskCreate(&vehicle_monitor_tcb,
               "Vehicle Monitor Task",
               vehicle_monitor_task,
               DEF_NULL,
               VEHICLE_MONITOR_TASK_PRIO,
               &vehicle_monitor_stack[0],
               (VEHICLE_MONITOR_TASK_SIZE / 10u),
               VEHICLE_MONITOR_TASK_SIZE,
               0u,
               0u,
               DEF_NULL,
               (OS_OPT_TASK_STK_CLR),
               &err);
  EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
}

/***************************************************************************//**
 * @brief Task handler for vehicle monitor task.
 *
 * @details Wait for flag to be posted from vehicle speed or direction task.
 *
 ******************************************************************************/
static void vehicle_monitor_task(void *arg)
{
    PP_UNUSED_PARAM(arg);
    RTOS_ERR err;

    uint32_t new_speed;
    direction new_curr_dir = no_dir;
    while (1)
    {

        OSFlagPend(&flags_vm,                /*   Pointer to user-allocated event flag. */
                   ( direction_update | speed_update),             /*   Flag bitmask to match.                */
                           0,                      /*   Wait for 100 OS Ticks maximum.        */
                   (OS_OPT_PEND_FLAG_SET_ANY)|                         /*   Wait until all flags are set and      */
                   (OS_OPT_PEND_BLOCKING     |/*    task will block and                  */
                    OS_OPT_PEND_FLAG_CONSUME), /*    function will clear the flags.       */
                    DEF_NULL,                 /*   Timestamp is not used.                */
                    &err);

        OSMutexPend(&mutex_vd,             /*   Pointer to user-allocated mutex.         */
                     0,                  /*   Wait for a maximum of 1000 OS Ticks.     */
                     OS_OPT_PEND_BLOCKING,  /*   Task will block.                         */
                     DEF_NULL,              /*   Timestamp is not used.                   */
                    &err);

        new_curr_dir = global_direction.curr_dir;

        OSMutexPost(&mutex_vd,         /*   Pointer to user-allocated mutex.         */
                     OS_OPT_POST_1,     /*   Only wake up highest-priority task.      */
                    &err);

        OSMutexPend(&mutex_ss,             /*   Pointer to user-allocated mutex.         */
                     0,                  /*   Wait for a maximum of 1000 OS Ticks.     */
                     OS_OPT_PEND_BLOCKING,  /*   Task will block.                         */
                     DEF_NULL,              /*   Timestamp is not used.                   */
                    &err);

        new_speed = global_speed.speed;

        OSMutexPost(&mutex_ss,         /*   Pointer to user-allocated mutex.         */
                     OS_OPT_POST_1,     /*   Only wake up highest-priority task.      */
                    &err);

        if((new_speed > 75) | ((new_speed > 45) && (new_curr_dir != no_dir))){
           OSFlagPost(&flags_led,             /*   Pointer to user-allocated event flag.    */
           led_update_0_1,            /*   Application Flag A bitmask.              */
           OS_OPT_POST_FLAG_SET,  /*   Set the flag.                            */
           &err);
        }

        else if(!(new_speed > 75) & !((new_speed > 45) && (new_curr_dir != no_dir))){
           OSFlagPost(&flags_led,             /*   Pointer to user-allocated event flag.    */
           led_update_0_0,            /*   Application Flag A bitmask.              */
           OS_OPT_POST_FLAG_SET,  /*   Set the flag.                            */
           &err);
        }

        if(global_direction.curr_dir_hold > 5000){
            OSFlagPost(&flags_led,             /*   Pointer to user-allocated event flag.    */
                       led_update_1_1,            /*   Application Flag A bitmask.              */
                       OS_OPT_POST_FLAG_SET,  /*   Set the flag.                            */
                       &err);
        }
        else{
            OSFlagPost(&flags_led,             /*   Pointer to user-allocated event flag.    */
                       led_update_1_0,            /*   Application Flag A bitmask.              */
                       OS_OPT_POST_FLAG_SET,  /*   Set the flag.                            */
                       &err);
        }



    }


}

/***************************************************************************//**
 * @brief Initializes lcd display task.
 *
 * @details Task initializes with priority of 20, stack of size 96, its own control block.
 *
 ******************************************************************************/
void lcd_display_init(void)
{
    RTOS_ERR err;
    uint32_t status;

    /* Enable the memory lcd */
    status = sl_board_enable_display();
    EFM_ASSERT(status == SL_STATUS_OK);

    /* Initialize the DMD support for memory lcd display */
    status = DMD_init(0);
    EFM_ASSERT(status == DMD_OK);

    /* Initialize the glib context */
    status = GLIB_contextInit(&glibContext);
    EFM_ASSERT(status == GLIB_OK);

    glibContext.backgroundColor = White;
    glibContext.foregroundColor = Black;

    /* Fill lcd with background color */
    GLIB_clear(&glibContext);

    /* Use Narrow font */
    GLIB_setFont(&glibContext, (GLIB_Font_t *) &GLIB_FontNarrow6x8);

  OSTaskCreate(&lcd_display_tcb,
               "LCD Display Task",
               lcd_display_task,
               DEF_NULL,
               LCD_DISPLAY_TASK_PRIO,
               &lcd_display_stack[0],
               (LCD_DISPLAY_TASK_SIZE / 10u),
               LCD_DISPLAY_TASK_SIZE,
               0u,
               0u,
               DEF_NULL,
               (OS_OPT_TASK_STK_CLR),
               &err);
  EFM_ASSERT((RTOS_ERR_CODE_GET(err) == RTOS_ERR_NONE));
}

/***************************************************************************//**
 * @brief Task handler for lcd display task.
 *
 * @details Every 100 ms, semaphore is posted that will call this task.
 *
 ******************************************************************************/
static void lcd_display_task(void *arg)
{
    PP_UNUSED_PARAM(arg);
    RTOS_ERR err;
    int lcd_speed;
    int lcd_dir;
    char snum[5];
    char sdir[20];
    char lcd_no_dir[] = "Forward Direction";
    char lcd_hardl_dir[] = "Hard Left";
    char lcd_l_dir[] = "Left";
    char lcd_r_dir[] = "Right";
    char lcd_hardr_dir[] = "Hard Right";

    while (1)
    {
        OSSemPend(&lcd_semaphore,        /* Pointer to user-allocated semaphore.    */
              0,                 /* Wait for a maximum of 1000 OS Ticks.    */
              OS_OPT_PEND_BLOCKING, /* Task will block.                        */
              DEF_NULL,             /* Timestamp is not used.                  */
              &err);

        OSMutexPend(&mutex_vd,             /*   Pointer to user-allocated mutex.         */
                     0,                  /*   Wait for a maximum of 1000 OS Ticks.     */
                     OS_OPT_PEND_BLOCKING,  /*   Task will block.                         */
                     DEF_NULL,              /*   Timestamp is not used.                   */
                    &err);

        lcd_dir = global_direction.curr_dir;

        OSMutexPost(&mutex_vd,         /*   Pointer to user-allocated mutex.         */
                     OS_OPT_POST_1,     /*   Only wake up highest-priority task.      */
                    &err);

        OSMutexPend(&mutex_ss,             /*   Pointer to user-allocated mutex.         */
                     0,                  /*   Wait for a maximum of 1000 OS Ticks.     */
                     OS_OPT_PEND_BLOCKING,  /*   Task will block.                         */
                     DEF_NULL,              /*   Timestamp is not used.                   */
                    &err);

        lcd_speed = global_speed.speed;

        OSMutexPost(&mutex_ss,         /*   Pointer to user-allocated mutex.         */
                     OS_OPT_POST_1,     /*   Only wake up highest-priority task.      */
                    &err);


        itoa(lcd_speed,snum,10);

        switch(lcd_dir){
          case hard_left:
            strcpy(sdir, lcd_hardl_dir);
            break;

          case left:
            strcpy(sdir, lcd_l_dir);
            break;

          case right:
            strcpy(sdir, lcd_r_dir);
            break;
          case hard_right:
            strcpy(sdir, lcd_hardr_dir);
            break;
          default:
            strcpy(sdir, lcd_no_dir);
            break;
        }

        GLIB_clear(&glibContext);

        GLIB_drawStringOnLine(&glibContext,
                              snum,
                               1,
                               GLIB_ALIGN_LEFT,
                               5,
                               5,
                               true);

        GLIB_drawStringOnLine(&glibContext,
                              sdir,
                               0,
                               GLIB_ALIGN_LEFT,
                               5,
                               5,
                               true);
        DMD_updateDisplay();

    }


}





/***************************************************************************//**
 * @brief Task handler for capsense
 *
 * @details Every 100 ms, call function that updates the state of cap_pressed and drives LEDs
 * accordingly.
 *
 ******************************************************************************/
static void cap_task(void *arg)
{
    PP_UNUSED_PARAM(arg);
    RTOS_ERR err;
    APP_MESSAGE  msg;

    while (1)
    {
        OS_SEM_CTR  ctr;

                                           /* Block until another task signals this task.     */
        ctr = OSSemPend(&App_Semaphore,        /* Pointer to user-allocated semaphore.    */
                         0,                 /* Wait for a maximum of 1000 OS Ticks.    */
                         OS_OPT_PEND_BLOCKING, /* Task will block.                        */
                         DEF_NULL,             /* Timestamp is not used.                  */
                        &err);
        //EFM_ASSERT(err.Code == RTOS_ERR_NONE);
        EFM_ASSERT(!ctr);

        cap_pressed();

        if((pb0 && !pb1) || (cap_dir == LEFT)) msg.btn0_press = true;
        else msg.btn0_press = false;

        if((pb1 && !pb0) || (cap_dir == RIGHT)) msg.btn1_press = true;
        else msg.btn1_press = false;

        OSQPost(&App_MessageQ,                /*   Pointer to user-allocated message queue.       */
                    (void *)&msg,                 /*   The message is a pointer to the APP_MESSAGE.   */
                    (OS_MSG_SIZE)sizeof(void *),  /*   Size of the message is the size of a pointer.  */
                     OS_OPT_POST_FIFO,            /*   Add message at the end of the queue.           */
                    &err);

    }


}

/***************************************************************************//**
 * @brief Task handler for idle/energy mode state.
 *
 * @details When no other task is running, enter EM1 to conserve energy.
 ******************************************************************************/
static void em_task(void *arg)
{
    PP_UNUSED_PARAM(arg);

    while (1)
    {
        EMU_EnterEM1();
    }


}

/***************************************************************************//**
 * @brief Task handler for pb1.
 *
 * @details Every 100 ms, call function that updates the state of pb1 and drives LEDs
 * accordingly.
 *
 ******************************************************************************/
static void led_task(void *arg)
{
    PP_UNUSED_PARAM(arg);
    RTOS_ERR err;
    OS_FLAGS flags;

    while (1)
    {
       flags = OSFlagPend(&flags_led,                /*   Pointer to user-allocated event flag. */
                           (led_update_0_0 | led_update_0_1 | led_update_1_0 | led_update_1_1),             /*   Flag bitmask to match.                */
                                   0,                      /*   Wait for 100 OS Ticks maximum.        */
                           (OS_OPT_PEND_FLAG_SET_ANY)|                         /*   Wait until all flags are set and      */
                           (OS_OPT_PEND_BLOCKING     |/*    task will block and                  */
                            OS_OPT_PEND_FLAG_CONSUME), /*    function will clear the flags.       */
                            DEF_NULL,                 /*   Timestamp is not used.                */
                            &err);

       if(flags == led_update_0_1){
           GPIO_PinOutSet(LED0_port, LED0_pin);
       }
       else if(flags == led_update_0_0) GPIO_PinOutClear(LED0_port, LED0_pin);

       if(flags == led_update_1_1) GPIO_PinOutSet(LED1_port, LED1_pin);
       else if(flags == led_update_1_0) GPIO_PinOutClear(LED1_port, LED1_pin);

    }


}

/***************************************************************************//**
 * @brief Updates global variable pb1 based on whether push button  is pressed.
 *
 * @details pb0 will be set to 1 if button0 is pressed otherwise will be set to 0.
 *
 ******************************************************************************/
void pb0_pressed(void){
  if(!GPIO_PinInGet(BUTTON0_port, BUTTON0_pin)){
      pb0 = PRESSED;
  }
  else pb0 = NOT_PRESSED;
}

/***************************************************************************//**
 * @brief Updates global variable pb1 based on whether push button 1 is pressed.
 *
 * @details pb1 will be set to 1 if button1 is pressed otherwise will be set to 1.
 *
 ******************************************************************************/
void pb1_pressed(void){
  if(!GPIO_PinInGet(BUTTON1_port, BUTTON1_pin)){
        pb1 = PRESSED;
  }
  else pb1 = NOT_PRESSED;
}

/***************************************************************************//**
 * @brief Updates global variable cap_dir based on location of finger on touch pad.
 *
 * @details Uses Enum to represent positions (Left, Right, and not pressed) as well
 * as Channel on touchpad(CH0-CH3). First calls function CAPSENSE_Sense() to update
 * CH0-CH3. Finger can only be on either right or left side, cannot be on both sides
 * at one time.
 *
 ******************************************************************************/
void cap_pressed(void){
  CAPSENSE_Sense();
  cap_dir = NONE;
  if(CAPSENSE_getPressed(CH0)) cap_dir = LEFT;
  if(CAPSENSE_getPressed(CH1)){
      if(cap_dir == NONE) cap_dir = LEFT;
  }
  if(CAPSENSE_getPressed(CH2)){
      if(cap_dir == NONE) cap_dir = RIGHT;
      else{
          cap_dir = NONE;
          return;
      }
  }
  if(CAPSENSE_getPressed(CH3)){
      if((cap_dir == NONE) || (cap_dir == RIGHT)) cap_dir = RIGHT;
      else cap_dir = NONE;
  }
}

/***************************************************************************//**
 * @brief Initalizes all OS Resources used.
 *
 * @details Uses Timer, semaphore, flag.
 *
 ******************************************************************************/
void lab6_init(void){
  RTOS_ERR err;
  global_speed.decr_count = 0;
  global_speed.incr_count = 0;
  global_speed.speed = 0;

  global_direction.curr_dir = no_dir;
  global_direction.curr_dir_hold = no_dir;
  global_direction.left_turns = 0;
  global_direction.right_turns = 0;

  ind_speed.decrement = false;
  ind_speed.increment = false;

  NVIC_EnableIRQ(GPIO_EVEN_IRQn);
  NVIC_EnableIRQ(GPIO_ODD_IRQn);
  GPIO_ExtIntConfig(gpioPortF, BUTTON0_pin, BUTTON0_pin, true, true, true);
  GPIO_ExtIntConfig(gpioPortF, BUTTON1_pin, BUTTON1_pin, true, true, true);
  GPIO_IntClear(0x0000FFFF);

//////////////////////////////////////////////Timer////////////////////////////////////////////////////////////////
  OSTmrCreate(&vd_timer,            /*   Pointer to user-allocated timer.     */
                  "Vehicle Direction Timer",           /*   Name used for debugging.             */
                     0,                  /*     0 initial delay.                   */
                    3,                  /*   100 Timer Ticks period.              */
                   OS_OPT_TMR_PERIODIC,  /*   Timer is periodic.                   */
                   App_TimerCallback,    /*   Function Called when timer expires.           */
                   DEF_NULL,             /*   No arguments to callback.            */
                  &err);
      EFM_ASSERT(err.Code == RTOS_ERR_NONE);
      //////////////////////////////////////////////Timer////////////////////////////////////////////////////////////////
//        OSTmrCreate(&vm_timer,            /*   Pointer to user-allocated timer.     */
//                        "Vehicle Monitor Timer",           /*   Name used for debugging.             */
//                           0,                  /*     0 initial delay.                   */
//                         10,                  /*   100 Timer Ticks period.              */
//                         OS_OPT_TMR_PERIODIC,  /*   Timer is periodic.                   */
//                         vehicle_monitor_task,    /*   Function Called when timer expires.           */
//                         DEF_NULL,             /*   No arguments to callback.            */
//                        &err);
//            EFM_ASSERT(err.Code == RTOS_ERR_NONE);
//////////////////////////////////////////////Semaphore/////////////////////////////////////////////////////////////
      OSSemCreate(&App_Semaphore,    /*   Pointer to user-allocated semaphore.          */
                  "App Semaphore",   /*   Name used for debugging.                      */
                   0,                /*   Initial count: available in this case.        */
                  &err);
      EFM_ASSERT(err.Code == RTOS_ERR_NONE);

      OSSemCreate(&lcd_semaphore,    /*   Pointer to user-allocated semaphore.          */
                  "LCD Semaphore",   /*   Name used for debugging.                      */
                   0,                /*   Initial count: available in this case.        */
                  &err);
      EFM_ASSERT(err.Code == RTOS_ERR_NONE);
/////////////////////////////////////////////Flags/////////////////////////////////////////////////////////////////
      OSFlagCreate(&flags_vm,              /*   Pointer to user-allocated event flag.         */
                       "Vehicle Monitor Flags",             /*   Name used for debugging.                      */
                        0,                      /*   Initial flags, all cleared.                   */
                       &err);
      EFM_ASSERT(err.Code == RTOS_ERR_NONE);

      OSFlagCreate(&flags_led,              /*   Pointer to user-allocated event flag.         */
                       "LED Output Flags",             /*   Name used for debugging.                      */
                        0,                      /*   Initial flags, all cleared.                   */
                       &err);
      EFM_ASSERT(err.Code == RTOS_ERR_NONE);
      ///////////////////////////////////////Message Queue/////////////////////////////////////////////////////////
//      OSQCreate(&App_MessageQ,           /*   Pointer to user-allocated message queue.          */
//                    "App MessageQ",          /*   Name used for debugging.                          */
//                     10,                     /*   Queue will have 10 messages maximum.              */
//                    &err);
//      EFM_ASSERT(err.Code == RTOS_ERR_NONE);
      /////////////////////////////////////////MUTEX///////////////////////////////////////////////////////////////
      OSMutexCreate(&mutex_vd,   /*   Pointer to user-allocated mutex.                 */
                    "Vehicle Direction Mutex",  /*   Name used for debugging.                         */
                    &err);
      OSMutexCreate(&mutex_ss,   /*   Pointer to user-allocated mutex.                 */
                    "Speed Setpoint Mutex",  /*   Name used for debugging.                         */
                    &err);

}

/***************************************************************************//**
 * @brief Function that is called when OS Timer completes.
 *
 * @details Posts semaphore.
 *
 ******************************************************************************/
void  App_TimerCallback (void)
{
    /* Called when timer expires:                            */

  RTOS_ERR    err;
                                 /* Post the pending App_SomeTask task.             */
  OSSemPost(&lcd_semaphore,    /* Pointer to user-allocated semaphore.    */
                   OS_OPT_POST_1,    /* Only wake up highest-priority task.     */
                  &err);


}

/***************************************************************************//**
 * @brief Called when PB0 is pressed.
 *
 * @details Indicated PB0 is pressed.
 *
 ******************************************************************************/
void GPIO_EVEN_IRQHandler(void)
{
  RTOS_ERR    err;

  GPIO->IFC |= BUTTON2_IFC;
  pb0_pressed();

  if((pb1 != PRESSED) && (pb0 == PRESSED)){
      push(&pb_data,2);
      OSSemPost(&App_Semaphore,    /* Pointer to user-allocated semaphore.    */
                       OS_OPT_POST_1,    /* Only wake up highest-priority task.     */
                      &err);
  }

}

/***************************************************************************//**
 * @brief Called when PB1 is pressed.
 *
 ******************************************************************************/
void GPIO_ODD_IRQHandler(void)
{
  RTOS_ERR    err;

  GPIO->IFC |= BUTTON1_IFC;
  pb1_pressed();

  if((pb0 != PRESSED) && (pb1 == PRESSED)){

          push(&pb_data,1);

          OSSemPost(&App_Semaphore,    /* Pointer to user-allocated semaphore.    */
                           OS_OPT_POST_1,    /* Only wake up highest-priority task.     */
                          &err);
      }
  }

/***************************************************************************//**
 * @brief Used to calculate how long current direction is pressed.
 ******************************************************************************/
void SysTick_Handler(void)
{
  if(global_direction.curr_dir != no_dir) global_direction.curr_dir_hold++;
  else global_direction.curr_dir_hold = 0;

}





