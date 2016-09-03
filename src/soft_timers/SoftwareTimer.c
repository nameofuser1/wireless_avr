/*
 * SoftwareTimer.c
 *
 *  Created on: 02 дек. 2015 г.
 *      Author: kripton
 */

#include "SoftwareTimer.h"

/*
 * **********************************
 * Init new timer
 * **********************************
 */
void SoftwareTimer_init(SoftwareTimer_Resources *tim)
{
	tim->cb = NULL;
	tim->state = Idle;
	tim->ticks = 0;
	tim->type = Timer_OnePulse;
	tim->length = 0;
}


/*
 * *****************************************
 * Set callback. Callback is called every
 * time timer tick event occurs
 * *****************************************
 */
void SoftwareTimer_add_cb(SoftwareTimer_Resources *tim, SoftTimerCallback cb)
{
	tim->cb = cb;
}


/*
 * *****************************************
 * Preparing timer for start
 * *****************************************
 */
void SoftwareTimer_arm(SoftwareTimer_Resources *tim, SoftTimerType type, uint32_t length)
{
	tim->state = Idle;
    tim->type = type;
	tim->length = length;
	tim->ticks = length;
}


/*
 * ***********************************************
 * Start timer. Using given list for controlling
 * Call SoftwareTimer_tick with the same list
 * to update timer
 * ***********************************************
 */
void SoftwareTimer_add_timer(HardwareTimerList *list, SoftwareTimer_Resources *timer)
{
	timer->state = Active;
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
			SoftwareTimer_Resources *tim = (SoftwareTimer_Resources*)cur_node->data;
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


void SoftwareTimer_remove_timer(HardwareTimerList *list, SoftwareTimer_Resources *timer)
{
	LinkedListNode *node = list->head;
	LinkedListNode *prev_node = list->head;
	while(node != NULL)
	{
		if((SoftwareTimer_Resources*)node->data == timer)
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
	timer->state = Idle;
}


/*
 * ********************************************
 * Call it on hardware timer compare interrupt
 *
 * ********************************************
 */
void SoftwareTimer_tick(HardwareTimerList *list)
{
    LinkedList done_timers;
    LinkedList_allocate(&done_timers);
	LinkedListNode *node = list->head;
	LinkedListNode *prev_node = list->head;
	while(node != NULL)
	{
		SoftwareTimer_Resources *tim = (SoftwareTimer_Resources*)(node->data);
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

				tim->state = Done;
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
        SoftwareTimer_Resources *tim = (SoftwareTimer_Resources*)(node->data);
        if(tim->type == Timer_Repeat)
        {
            SoftwareTimer_arm(tim, tim->type, tim->length);
            SoftwareTimer_add_timer(list, tim);
        }
        prev_node = node;
        node = node->next;
        free(prev_node);
	}
}


/*
 * *******************************************
 * !!! IMPORTANT !!!
 * Can wait only for one pulse timers
 * For repeating timers please use callback
 * *******************************************
 */
void SoftwareTimer_wait_for(SoftwareTimer_Resources *tim)
{
	if(tim->state != Active)
	{
		return;
	}

	if(tim->type == Timer_OnePulse)
	{
		while(tim->state != Done);
	}
}


void SoftwareTimer_delay_ms(HardwareTimerList *list, uint32_t ms)
{
	SoftwareTimer_Resources tim;
	SoftwareTimer_init(&tim);
	SoftwareTimer_arm(&tim, Timer_OnePulse, ms);
	SoftwareTimer_add_timer(list, &tim);
	SoftwareTimer_wait_for(&tim);
}
