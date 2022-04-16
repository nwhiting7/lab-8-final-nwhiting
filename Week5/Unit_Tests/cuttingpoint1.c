#include "cuttingpoint1.h"
#include "queue.h"
#include <stdio.h>

void init(struct task_t *task, int *execution, int size) {
    for(int i = 0; i < size; i++){
        task[i].execution_time = execution[i];
        task[i].process_id = i;
    }
}

void first_come_first_served(struct task_t *task, int size) {
    
    int wait_time = 0;

    struct node_t* head = create_queue(task, size);

    while(!is_empty(&head)){
        head->task->waiting_time = wait_time;
        head->task->turnaround_time = task->execution_time + wait_time;
        wait_time += head->task->execution_time;
        pop(&head);
    }

}

float calculate_average_wait_time(struct task_t *task, int size) {
    float wait_time = 0.0;
    for(int i = 0; i < size; i++){
        wait_time += task[i].waiting_time;
    }
    
    return (wait_time / size);
}

float calculate_average_turn_around_time(struct task_t *task, int size) {
    float turn_time = 0.0;
    for(int i = 0; i < size; i++){
        turn_time += task[i].turnaround_time;
    }
    
    return (turn_time / size);

struct game_platform slider_force_task(float max_force, float velocity, int direction){
        struct game_platform test_plat;
        if(direction == right) test_plat.max_force = max_force;
        else if(direction == left) test_plat.max_force = max_force * -1;
        else{
            test_plat.max_force = 0;
            test_plat.velocity -= (test_plat.velocity * 0.1);
        }

        return test_plat;
}

}