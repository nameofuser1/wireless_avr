#ifndef LINKEDLIST_H_INCLUDED
#define LINKEDLIST_H_INCLUDED

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>


typedef struct LinkedListNode {

    void                    (*data);
    struct LinkedListNode   (*next);

}   LinkedListNode;


typedef struct {

    LinkedListNode      (*head);
    LinkedListNode      (*tail);

}   LinkedList;



void                LinkedList_allocate(LinkedList *linked_list);
void                LinkedList_free(LinkedList *LinkedList);
bool                LinkedList_add(LinkedList *list, void *data);
void                LinkedList_remove(LinkedList *list, LinkedListNode *node);



#endif // LINKEDLIST_H_INCLUDED
