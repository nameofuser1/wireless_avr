#include <common/LinkedList.h>


void LinkedList_allocate(LinkedList *linked_list)
{
    linked_list->head = NULL;
    linked_list->tail = NULL;
}


void LinkedList_free(LinkedList *linked_list)
{
    LinkedListNode *node = (linked_list)->head;
    while(node != NULL)
    {
        LinkedListNode *next_node = node->next;
        free(node);
        node = next_node;
    }
}


bool LinkedList_add(LinkedList *list, void *data)
{
    LinkedListNode *tail_node = malloc(sizeof(LinkedListNode));
    if(tail_node == NULL)
    {
        return false;
    }

    if(list->head == NULL)
    {
        list->head = tail_node;
        list->head->data = data;
        list->head->next = NULL;
        list->tail = list->head;
    }
    else if(list->head == list->tail)
    {
        tail_node->data = data;
        tail_node->next = NULL;
        list->tail = tail_node;
        list->head->next = tail_node;
    }
    else if(list->tail->next == NULL)
    {
        tail_node->data = data;
        tail_node->next = NULL;
        list->tail->next = tail_node;
        list->tail = tail_node;
    }
    else
    {
        free(tail_node);
        return false;
    }

    return true;
}


void LinkedList_remove(LinkedList *list, LinkedListNode *remove_node)
{
    if(list->head != NULL)
    {
        LinkedListNode *node = list->head;
        LinkedListNode *prev_node = list->head;
        while(node != NULL)
        {
            if(node == remove_node)
            {
                if(prev_node == NULL)
                {
                    list->head = node->next;
                    free(node);
                    break;
                }
                else
                {
                    prev_node->next = node->next;
                    free(node);
                    break;
                }
            }
            else
            {
                prev_node = node;
            }

            node = node->next;

        }
    }
}


