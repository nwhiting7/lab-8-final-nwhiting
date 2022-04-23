#include "fcfs.h"
#include "queue.h"
#include <stdio.h>

void init(struct slider_info *slider_info, int button_pressed, int ind_armed, int ind_charging, int use_laser, int num_activations){
        slider_info->button_pressed = button_pressed;
        slider_info->ind_armed = ind_armed;
        slider_info->ind_charging = ind_charging;
        slider_info->num_activations = num_activations;
        slider_info->use_laser = use_laser;
}

void init_dir(struct direction_info *direction_info, int direction, int max_force){
    direction_info->direction = direction;
    direction_info->max_force = max_force;

}

void init_plat(struct platform_info *platform_info, int velocity, int position, int max_force, int mass){
    platform_info->velocity = velocity;
    platform_info->position = position;
    platform_info->max_force = max_force;
    platform_info->mass = mass;
}

void canyon_init(struct canyon_info *canyon_info, int x_start, int x_end, int position_left, int position_right, int bounce_enable){
    canyon_info->x_start = x_start;
    canyon_info->x_end = x_end;
    canyon_info->position_left = position_left;
    canyon_info->position_right = position_right;
    canyon_info->bounce_enable = bounce_enable;
}

// void first_come_first_served(struct task_t *task, int size) {
    
//     int wait_time = 0;

//     struct node_t* head = create_queue(task, size);

//     while(!is_empty(&head)){
//         head->task->waiting_time = wait_time;
//         head->task->turnaround_time = task->execution_time + wait_time;
//         wait_time += head->task->execution_time;
//         pop(&head);
//     }

// }

// float calculate_average_wait_time(struct task_t *task, int size) {
//     float wait_time = 0.0;
//     for(int i = 0; i < size; i++){
//         wait_time += task[i].waiting_time;
//     }
    
//     return (wait_time / size);
// }

// float calculate_average_turn_around_time(struct task_t *task, int size) {
//     float turn_time = 0.0;
//     for(int i = 0; i < size; i++){
//         turn_time += task[i].turnaround_time;
//     }
    
//     return (turn_time / size);
// }