#include <stdio.h>
#include <stdlib.h>
#include "queue.h"

struct node_t* create_queue(struct task_t* task, int size) {

    struct node_t* head = create_new_node(task);

    for(int i = 0; i < size; i++){
        push(&head, &task[i]);
    }
    pop(&head);
    return head;
}

struct node_t* create_new_node(struct task_t* task) {
    struct node_t* new_node = NULL;
    new_node = (struct node_t*)malloc(sizeof(struct node_t));
    new_node->task = task;      
    new_node->next = NULL;
    return new_node;
}

struct task_t* peek(struct node_t** head) {
    if(is_empty(head)) return NULL;
    else return (*head)->task;
}

void pop(struct node_t** head) {
        if(*head == NULL){
            return;
        }
        struct node_t* next_node = (*head)->next;
        free(*head);
        (*head) = next_node;
}

void push(struct node_t** head, struct task_t* task) {
    struct node_t* new_node= create_new_node(task);
    struct node_t* last_node = *head;

    new_node->next = NULL;

    if(*head == NULL){
        *head = new_node;
        return;
    }

    while(last_node->next != NULL) last_node = last_node->next;

    last_node->next = new_node;
}

int is_empty(struct node_t** head) {
    if(*head == NULL) return 1;     
    else return 0;
}

void empty_queue(struct node_t** head) {
   while(!is_empty(head)) pop(head);
}
