/*
 * HardwareTimer.c
 *
 *  Created on: 3 сент. 2016 г.
 *      Author: bigmac
 */


#include "HardwareTimer.h"

/* Static methods */
static void timer_tick(HardwareTimerResources *timer);


/*
 * Initialization doesn't provide any options now
 */
void HardwareTimer_Init(HardwareTimerResources *timer, uint16_t prescaler)
{
	timer->io_tim->PSC = prescaler;								//clock prescaler
	timer->io_tim->DIER |= TIM_DIER_UIE;						//update interrupt enable
	timer->io_tim->CR1 &= (~TIM_CR1_OPM | TIM_CR1_ARPE);		//periodic mode
}

/*
 * Enable timer
 */
void HardwareTimer_Start(HardwareTimerResources *timer)
{
	timer->io_tim->CR1 |= TIM_CR1_CEN;
}

/*
 * Disable timer
 */
void HardwareTimer_Stop(HardwareTimerResources *timer)
{
	timer->io_tim->CR1 &= ~TIM_CR1_CEN;
}

/*
 * Set auto-reload register
 */
void HardwareTimer_SetDuration(HardwareTimerResources *timer, uint16_t duration)
{
	timer->io_tim->ARR = duration;
}


/*
 * ***********************************************
 * Start timer. Using given list for controlling
 * Call SoftwareTimer_tick with the same list
 * to update timer
 * ***********************************************
 */
void HardwareTimer_AddTimer(HardwareTimerResources *h_timer, SoftwareTimer *timer)
{
	HardwareTimerList *list = h_timer->timers;

	timer->state = Timer_Active;
	uint32_t timer_length = timer->length;
	LinkedListNode *new_node = (LinkedListNode*)malloc(sizeof(LinkedListNode));
	new_node->data = (void*)timer;
	new_node->next = NULL;

	if(list->head == NULL)
	{
		list->head = new_node;
		return;
	}
	else
	{
		LinkedListNode *cur_node = list->head;
		LinkedListNode *prev_node = list->head;
		while(cur_node != NULL)
		{
			SoftwareTimer *tim = (SoftwareTimer*)cur_node->data;
			if(timer_length < tim->ticks)
			{
				new_node->next = cur_node;
				if(cur_node == list->head)
				{
					list->head = new_node;
				}
				else
				{
					prev_node->next = new_node;
				}
				return;
			}
			prev_node = cur_node;
			cur_node = cur_node->next;
		}

		prev_node->next = new_node;
	}
}

/*
 * Remove software timer from timers list
 */
void HardwareTimer_RemoveTimer(HardwareTimerResources *h_timer, SoftwareTimer *timer)
{
	HardwareTimerList *list = h_timer->timers;

	LinkedListNode *node = list->head;
	LinkedListNode *prev_node = list->head;
	while(node != NULL)
	{
		if((SoftwareTimer*)node->data == timer)
		{
			if(node == list->head)
			{
				list->head = node->next;
			}
			else
			{
				prev_node->next = node->next;
			}
			break;
		}
		else
		{
			prev_node = node;
		}
	}
	timer->state = Timer_Idle;
}


/*
 * Simple delay in MS
 */
void HardwareTimer_DelayMs(HardwareTimerResources *timer, uint32_t ms)
{
	SoftwareTimer tim;
	SoftwareTimer_Init(&tim);
	SoftwareTimer_Arm(&tim, Timer_OnePulse, ms);
	HardwareTimer_AddTimer(timer, &tim);
	SoftwareTimer_WaitFor(&tim);
}


/* Check only for update interrupt flag */
void HardwareTimer_IRQHandler(HardwareTimerResources *timer)
{
	if(timer->io_tim->SR & TIM_SR_UIF)
	{
		timer_tick(timer);
		timer->io_tim->SR &= ~TIM_SR_UIF;
	}
}


/*
 * ********************************************
 * Call it on hardware timer compare interrupt
 *
 * ********************************************
 */
static void timer_tick(HardwareTimerResources *timer)
{
	HardwareTimerList *list = timer->timers;

    LinkedList done_timers;
    LinkedList_allocate(&done_timers);
	LinkedListNode *node = list->head;
	LinkedListNode *prev_node = list->head;
	while(node != NULL)
	{
		SoftwareTimer *tim = (SoftwareTimer*)(node->data);
		if(tim != NULL)
		{
			if(tim->ticks > 1)
			{
				tim->ticks -= 1;
				prev_node = node;
				node = node->next;
			}
			else
			{
				if(tim->cb != NULL)
				{
					tim->cb();
				}

				tim->state = Timer_Done;
				LinkedList_add(&done_timers, (void*)tim);
				if(node == list->head)
				{
					list->head = node->next;
					free(node);
					node = list->head;
				}
				else
				{
					prev_node->next = node->next;
					free(node);
					node = prev_node->next;
				}
			}
		}
	}

	node = done_timers.head;
	prev_node = node;
	while(node != NULL)
	{
        SoftwareTimer *tim = (SoftwareTimer*)(node->data);
        if(tim->type == Timer_Repeat)
        {
            SoftwareTimer_Arm(tim, tim->type, tim->length);
            HardwareTimer_AddTimer(timer, tim);
        }
        prev_node = node;
        node = node->next;
        free(prev_node);
	}
}

/*
 * Timer 2 driver definition
 */

static HardwareTimerList		tim2_list;
static HardwareTimerResources	tim2_resources;

static void HardwareTimer2_init(uint16_t prescaler)
{
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	NVIC_EnableIRQ(TIM2_IRQn);

	tim2_resources.timers = &tim2_list;
	tim2_resources.io_tim = TIM2;

	HardwareTimer_Init(&tim2_resources, prescaler);
}

static void HardwareTimer2_start(void) { HardwareTimer_Start(&tim2_resources); }
static void HardwareTimer2_stop(void) { NVIC_DisableIRQ(TIM2_IRQn); HardwareTimer_Stop(&tim2_resources); }
static void HardwareTimer2_set_duration(uint16_t duration) { HardwareTimer_SetDuration(&tim2_resources, duration); }
static void HardwareTimer2_add_timer(SoftwareTimer *tim) { HardwareTimer_AddTimer(&tim2_resources, tim); }
static void HardwareTimer2_remove_timer(SoftwareTimer *tim) { HardwareTimer_RemoveTimer(&tim2_resources, tim); }
static void HardwareTimer2_delay(uint16_t ms) { HardwareTimer_DelayMs(&tim2_resources, ms); }


HardwareTimerDriver		TIMER2_Driver = {
		.Init = (init_func_t)HardwareTimer2_init,
		.Start = (start_func_t)HardwareTimer2_start,
		.Stop = (stop_func_t)HardwareTimer2_stop,
		.SetDuration = (set_duration_func_t)HardwareTimer2_set_duration,
		.AddTimer = (add_timer_func_t)HardwareTimer2_add_timer,
		.RemoveTimer = (remove_timer_func_t)HardwareTimer2_remove_timer,
		.Delay = (delay_func_t)HardwareTimer2_delay
};

volatile uint32_t enter_addr = 0;

/* Interrupts */
void TIM2_IRQHandler(void)
{
	asm(
			"push {r8}\n\t"
			"ldr r7, =(enter_addr)\n\t"
			"add r8, sp, #0x04\n\t"				// one word higher (r8)  // three words higher (r7, lr, r8)
			"str r8, [r7]\n\t"					// store address into enter_addr
			"pop {r8}\n\t"
	);
	HardwareTimer_IRQHandler(&tim2_resources);
}


