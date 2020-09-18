/**
 * @file rtos.c
 * @author ITESO
 * @date Feb 2018
 * @brief Implementation of rtos API
 *
 * This is the implementation of the rtos module for the
 * embedded systems II course at ITESO
 */

#include "rtos.h"
#include "rtos_config.h"
#include "clock_config.h"

#ifdef RTOS_ENABLE_IS_ALIVE
#include "fsl_gpio.h"
#include "fsl_port.h"
#endif
/**********************************************************************************/
// Module defines
/**********************************************************************************/

#define FORCE_INLINE 	__attribute__((always_inline)) inline

#define STACK_FRAME_SIZE			8
#define STACK_LR_OFFSET				2
#define STACK_PSR_OFFSET			1
#define STACK_PSR_DEFAULT			0x01000000

/**********************************************************************************/
// IS ALIVE definitions
/**********************************************************************************/

#ifdef RTOS_ENABLE_IS_ALIVE
#define CAT_STRING(x,y)  		x##y
#define alive_GPIO(x)			CAT_STRING(GPIO,x)
#define alive_PORT(x)			CAT_STRING(PORT,x)
#define alive_CLOCK(x)			CAT_STRING(kCLOCK_Port,x)
static void init_is_alive(void);
static void refresh_is_alive(void);
#endif

/**********************************************************************************/
// Type definitions
/**********************************************************************************/

typedef enum
{
	S_READY = 0, S_RUNNING, S_WAITING, S_SUSPENDED
} task_state_e;
typedef enum
{
	kFromISR = 0, kFromNormalExec
} task_switch_type_e;

typedef struct
{
	uint8_t priority;
	task_state_e state;
	uint32_t *sp;
	void (*task_body)();
	rtos_tick_t local_tick;
	uint32_t reserved[10];
	uint32_t stack[RTOS_STACK_SIZE];
} rtos_tcb_t;

/**********************************************************************************/
// Global (static) task list
/**********************************************************************************/

struct
{
	uint8_t nTasks;
	rtos_task_handle_t current_task;
	rtos_task_handle_t next_task;
	rtos_tcb_t tasks[RTOS_MAX_NUMBER_OF_TASKS + 1];
	rtos_tick_t global_tick;
} task_list =
{ 0 };

/**********************************************************************************/
// Local methods prototypes
/**********************************************************************************/

static void reload_systick(void);
static void dispatcher(task_switch_type_e type);
static void activate_waiting_tasks();
FORCE_INLINE static void context_switch(task_switch_type_e type);
static void idle_task(void);

/**********************************************************************************/
// API implementation
/**********************************************************************************/

void rtos_start_scheduler(void)
{
#ifdef RTOS_ENABLE_IS_ALIVE
	init_is_alive();
#endif
	task_list.global_tick = 0;

	SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk
	        | SysTick_CTRL_ENABLE_Msk;
	reload_systick();

	rtos_create_task(idle_task, 0, kAutoStart);

	NVIC_SetPriority(PendSV_IRQn, 0xFF);
	for (;;)
		;
}

rtos_task_handle_t rtos_create_task(void (*task_body)(), uint8_t priority,
		rtos_autostart_e autostart)
{
	if(RTOS_MAX_NUMBER_OF_TASKS > task_list.nTasks)
	{
		if(kAutoStart == autostart)
		{
			task_list.tasks[task_list.nTasks].state = S_READY;
		}
		else
		{
			task_list.tasks[task_list.nTasks].state = S_SUSPENDED;
		}
		task_list.tasks[task_list.nTasks].task_body = task_body;
		task_list.tasks[task_list.nTasks].sp = &(task_list.tasks[task_list.nTasks].stack[RTOS_STACK_SIZE - 1]) - STACK_FRAME_SIZE;
		task_list.tasks[task_list.nTasks].stack[RTOS_STACK_SIZE - STACK_LR_OFFSET] = (uint32_t) task_body;
		task_list.tasks[task_list.nTasks].stack[RTOS_STACK_SIZE - STACK_PSR_OFFSET] = (STACK_PSR_DEFAULT);

		task_list.tasks[task_list.nTasks].priority = priority;

		task_list.tasks[task_list.nTasks].local_tick = 0;

		task_list.nTasks ++;

		return (task_list.nTasks - 1);
	}
	return -1;
}

rtos_tick_t rtos_get_clock(void)
{

	return task_list.global_tick;
}

void rtos_delay(rtos_tick_t ticks)
{
	task_list.tasks[task_list.current_task].state = S_WAITING;
	task_list.tasks[task_list.current_task].local_tick = ticks;
	dispatcher(kFromNormalExec);
}

void rtos_suspend_task(void)
{
	task_list.tasks[task_list.current_task].state = S_SUSPENDED;
	dispatcher(kFromNormalExec);
}

void rtos_activate_task(rtos_task_handle_t task)
{
	task_list.tasks[task].state = S_READY;
	dispatcher(kFromNormalExec);
}

/**********************************************************************************/
// Local methods implementation
/**********************************************************************************/

static void reload_systick(void)
{
	SysTick->LOAD = USEC_TO_COUNT(RTOS_TIC_PERIOD_IN_US,
			CLOCK_GetCoreSysClkFreq());
	SysTick->VAL = 0;
}

static void dispatcher(task_switch_type_e type)
{
	rtos_task_handle_t next_task = task_list.nTasks - 1;
	uint8_t maxPrior = 0;
	uint8_t idx = 0;

	for(idx = 0 ; task_list.nTasks > idx; idx++)
	{
		if(maxPrior > task_list.tasks[idx].priority)
		{
			maxPrior = task_list.tasks[idx].priority;
		}
	}

	for (idx = 0 ; task_list.nTasks > idx; idx++)
	{
		if(maxPrior < task_list.tasks[idx].priority
				&& (S_RUNNING == task_list.tasks[idx].state || S_READY == task_list.tasks[idx].state))
		{
			maxPrior = task_list.tasks[idx].priority;
			next_task = idx;
		}
	}
	task_list.next_task = next_task;

	if(task_list.current_task != task_list.next_task)
	{
		context_switch(type);
	}
}

FORCE_INLINE static void context_switch(task_switch_type_e type)
{
	static uint8_t first = 1;
	register uint32_t r0 asm("sp");

	(void) r0;

	if(!first)
	{
		asm("mov r0, r7");
		task_list.tasks[task_list.current_task].sp = (uint32_t*) r0;
		if (kFromNormalExec == type)
		  {
			task_list.tasks[task_list.current_task].sp -= (7);
			//task_list.tasks[task_list.current_task].state = S_READY;
		  }
		  else
		  {
			task_list.tasks[task_list.current_task].sp -= (-9);
		  }
	}
	else
	{
		first = 0;
	}
	task_list.current_task = task_list.next_task;
	task_list.tasks[task_list.current_task].state = S_RUNNING;
	SCB -> ICSR |= SCB_ICSR_PENDSVSET_Msk;
}

static void activate_waiting_tasks()
{
	uint8_t idx = 0;
	for (idx = 0 ; task_list.nTasks > idx; idx++)
	{
		if(S_WAITING == task_list.tasks[idx].state)
		{
			task_list.tasks[idx].local_tick--;
			if(0 == task_list.tasks[idx].local_tick)
			{
				task_list.tasks[idx].state = S_READY;
			}
		}
	}
}

/**********************************************************************************/
// IDLE TASK
/**********************************************************************************/

static void idle_task(void)
{
	for (;;)
		;
}

/****************************************************/
// ISR implementation
/****************************************************/

void SysTick_Handler(void)
{
#ifdef RTOS_ENABLE_IS_ALIVE
	refresh_is_alive();
#endif
	task_list.global_tick ++;
	activate_waiting_tasks();
	reload_systick();
	dispatcher(kFromISR);
}

void PendSV_Handler(void)
{
	register int32_t r0 asm("r0");
	(void) r0;
	SCB->ICSR |= SCB_ICSR_PENDSVCLR_Msk;
	r0 = (int32_t) task_list.tasks[task_list.current_task].sp;
	asm("mov r7,r0");
}

/**********************************************************************************/
// IS ALIVE SIGNAL IMPLEMENTATION
/**********************************************************************************/

#ifdef RTOS_ENABLE_IS_ALIVE
static void init_is_alive(void)
{
	gpio_pin_config_t gpio_config =
	{ kGPIO_DigitalOutput, 1, };

	port_pin_config_t port_config =
	{ kPORT_PullDisable, kPORT_FastSlewRate, kPORT_PassiveFilterDisable,
	        kPORT_OpenDrainDisable, kPORT_LowDriveStrength, kPORT_MuxAsGpio,
	        kPORT_UnlockRegister, };
	CLOCK_EnableClock(alive_CLOCK(RTOS_IS_ALIVE_PORT));
	PORT_SetPinConfig(alive_PORT(RTOS_IS_ALIVE_PORT), RTOS_IS_ALIVE_PIN,
	        &port_config);
	GPIO_PinInit(alive_GPIO(RTOS_IS_ALIVE_PORT), RTOS_IS_ALIVE_PIN,
	        &gpio_config);
}

static void refresh_is_alive(void)
{
	static uint8_t state = 0;
	static uint32_t count = 0;
	SysTick->LOAD = USEC_TO_COUNT(RTOS_TIC_PERIOD_IN_US,
	        CLOCK_GetCoreSysClkFreq());
	SysTick->VAL = 0;
	if (RTOS_IS_ALIVE_PERIOD_IN_US / RTOS_TIC_PERIOD_IN_US - 1 == count)
	{
		GPIO_PinWrite(alive_GPIO(RTOS_IS_ALIVE_PORT), RTOS_IS_ALIVE_PIN,
		        state);
		state = state == 0 ? 1 : 0;
		count = 0;
	} else //
	{
		count++;
	}
}
#endif
