/*
 * fifo.c
 *
 *  Created on: Mar 1, 2022
 *      Author: nickd
 */

#include "fifo.h"

struct node_t* create_queue(int button_value, int size) {

    struct node_t* head = create_new_node(button_value);

    for(int i = 0; i < size; i++){
        push(&head, 0);
    }
    pop(&head);
    return head;
}

struct node_t* create_new_node(int button_value) {
    struct node_t* new_node = 0;
    new_node = (struct node_t*)malloc(sizeof(struct node_t));
    new_node->push_button = button_value;
    new_node->next = 0;
    return new_node;
}

int peek(struct node_t** head) {
    if(is_empty(head)) return 0;
    else return (*head)->push_button;
}

void pop(struct node_t** head) {
        if(*head == 0){
            return;
        }
        struct node_t* next_node = (*head)->next;
        free(*head);
        (*head) = next_node;
}

void push(struct node_t** head, int button_value) {
    struct node_t* new_node= create_new_node(button_value);
    struct node_t* last_node = *head;

    new_node->next = 0;

    if(*head == 0){
        *head = new_node;
        return;
    }

    while(last_node->next != 0) last_node = last_node->next;

    last_node->next = new_node;
}

int is_empty(struct node_t** head) {
    if(*head == 0) return 1;
    else return 0;
}

void empty_queue(struct node_t** head) {
   while(!is_empty(head)) pop(head);
}


