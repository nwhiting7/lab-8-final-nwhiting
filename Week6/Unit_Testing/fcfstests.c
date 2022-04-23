#include <stdlib.h>
#include "ctest.h"
#include <stdio.h>
#include "fcfs.h"

//Note: the name in the first parameter slot must match all tests in that group
CTEST_DATA(firstcomefirstserved) {
    struct slider_info slider[1];
    struct direction_info direction[1];
    struct platform_info platform[1];
    struct canyon_info canyon[1];
};

CTEST_SETUP(firstcomefirstserved) {
    int button_pressed = 2, ind_armed = 1, ind_charging = 0, use_laser = 0, num_activations = 1,
     dir = 0, max_force = 6, velocity = 0, position = 0, mass = 2, x_start = 0, x_end = 6, position_left = 1,
     position_right = 1, bounce_enable = 0;
    init(data->slider, button_pressed, ind_armed, ind_charging, use_laser, num_activations);
    init_dir(data->direction, dir, max_force);
    init_plat(data->platform, velocity, position, max_force, mass);
    canyon_init(data->canyon, x_start, x_end, position_left, position_right, bounce_enable);
}

CTEST2(firstcomefirstserved, test_process) {

    if((data->slider->button_pressed == 2) && (data->slider->use_laser == 0) &&
    data->slider->num_activations != 0){
        data->slider->use_laser = 1;
        data->slider->num_activations--;
    }
    
    ASSERT_EQUAL(1, (int)data->slider->use_laser);
    ASSERT_EQUAL(0, (int)data->slider->num_activations);
    
}

CTEST2(firstcomefirstserved, cut1_test1) {
    data->slider->use_laser = 0;
    if((data->slider->button_pressed == 2) && (data->slider->use_laser == 0) &&
    data->slider->num_activations != 0){
        data->slider->use_laser = 1;
        data->slider->num_activations--;
    }

    ASSERT_EQUAL(1, (int)data->slider->use_laser);
    ASSERT_EQUAL(0, (int)data->slider->num_activations);

}

CTEST2(firstcomefirstserved, cut1_test3){
    data->slider->button_pressed = 2;
    data->slider->ind_armed = 0;
    data->slider->ind_charging = 0;

    if((data->slider->button_pressed == 2) && (data->slider->ind_armed != 1) && (data->slider->ind_charging != 1)){
        data->slider->ind_armed = 1;
    }

    ASSERT_EQUAL(1, (int)data->slider->ind_armed);    
}

CTEST2(firstcomefirstserved, cut1_test4){
    if((data->slider->button_pressed == 2) && (data->slider->ind_armed != 1) && (data->slider->ind_charging != 1)){
        data->slider->ind_armed = 1;
    }
    else{
        data->slider->ind_armed = 0;
    }

    ASSERT_EQUAL(0, (int)data->slider->ind_armed);    
}

CTEST2(firstcomefirstserved, cut2_test1){
    int cap_channel = 3;
    int max_force_const = 5;
    
    data->direction->direction = 4;
    if(cap_channel == 0){
        data->direction->direction = 0;
    }
    else if(cap_channel == 1){
        if(data->direction->direction == 4) data->direction->direction = 1;
        else data->direction->direction = 4;
    }
    else if(cap_channel == 2){
        if(data->direction->direction == 4) data->direction->direction = 2;
        else data->direction->direction = 4;
    }
    else if(cap_channel == 3){
        if(data->direction->direction == 4) data->direction->direction = 3;
        else data->direction->direction = 4;
    }

    if(data->direction->direction == 3) data->direction->max_force = max_force_const;
    else if(data->direction->direction == 2) data->direction->max_force = max_force_const * 0.5;
    else if(data->direction->direction == 1) data->direction->max_force = max_force_const * -1;
    else if(data->direction->direction == 0) data->direction->max_force = max_force_const * 0.5 * -1;
    else data->direction->max_force = 0;

    ASSERT_EQUAL(3, data->direction->direction); 
    ASSERT_EQUAL(max_force_const, data->direction->max_force);   

}

CTEST2(firstcomefirstserved, test_process_6){
    int cap_channel = 1;
    int max_force_const = 5;
    
    data->direction->direction = 3;
    if(cap_channel == 0){
        data->direction->direction = 0;
    }
    else if(cap_channel == 1){
        if(data->direction->direction == 4) data->direction->direction = 1;
        else data->direction->direction = 4;
    }
    else if(cap_channel == 2){
        if(data->direction->direction == 4) data->direction->direction = 2;
        else data->direction->direction = 4;
    }
    else if(cap_channel == 3){
        if(data->direction->direction == 4) data->direction->direction = 3;
        else data->direction->direction = 4;
    }

    if(data->direction->direction == 3) data->direction->max_force = max_force_const;
    else if(data->direction->direction == 2) data->direction->max_force = max_force_const * 0.5;
    else if(data->direction->direction == 1) data->direction->max_force = max_force_const * -1;
    else if(data->direction->direction == 0) data->direction->max_force = max_force_const * 0.5 * -1;
    else data->direction->max_force = 0;

    ASSERT_EQUAL(4, data->direction->direction);   
    ASSERT_EQUAL(0, data->direction->max_force);  
    
}

CTEST2(firstcomefirstserved, test_process_7){
    int cap_channel = 0;
    int max_force_const = 5;
    
    data->direction->direction = 4;
    if(cap_channel == 0){
        data->direction->direction = 0;
    }
    else if(cap_channel == 1){
        if(data->direction->direction == 4) data->direction->direction = 1;
        else data->direction->direction = 4;
    }
    else if(cap_channel == 2){
        if(data->direction->direction == 4) data->direction->direction = 2;
        else data->direction->direction = 4;
    }
    else if(cap_channel == 3){
        if(data->direction->direction == 4) data->direction->direction = 3;
        else data->direction->direction = 4;
    }

    if(data->direction->direction == 3) data->direction->max_force = max_force_const;
    else if(data->direction->direction == 2) data->direction->max_force = max_force_const * 0.5;
    else if(data->direction->direction == 1) data->direction->max_force = max_force_const * -1;
    else if(data->direction->direction == 0) data->direction->max_force = max_force_const * 0.5 * -1;
    else data->direction->max_force = 0;

    ASSERT_EQUAL(0, data->direction->direction);   
    ASSERT_EQUAL(max_force_const * -0.5, data->direction->max_force);  
    
}

CTEST2(firstcomefirstserved, test_process_8){
    int timer_update = 2;

    data->platform->velocity = data->platform->velocity + (data->platform->max_force / data->platform->mass)*timer_update; //6
    data->platform->position = data->platform->position + (data->platform->velocity * timer_update);

    ASSERT_EQUAL(6, data->platform->velocity);
    ASSERT_EQUAL(12, data->platform->position);

}

CTEST2(firstcomefirstserved, test_process_9){
    int timer_update = 2;

    data->platform->max_force *= -1;

    data->platform->velocity = data->platform->velocity + (data->platform->max_force / data->platform->mass)*timer_update; //6
    data->platform->position = data->platform->position + (data->platform->velocity * timer_update);

    ASSERT_EQUAL(-6, data->platform->velocity);
    ASSERT_EQUAL(-12, data->platform->position);

}

CTEST2(firstcomefirstserved, test_process_10){

    if((data->platform->position - data->canyon->position_left) < data->canyon->x_start){
        data->platform->position = data->canyon->x_start + data->canyon->position_left;
        if(data->canyon->bounce_enable) data->platform->velocity *= -1;
    }
    else if((data->platform->position + data->canyon->position_right) > data->canyon->x_end){
        data->platform->position = data->canyon->x_end - data->canyon->position_right;
        if(data->canyon->bounce_enable) data->platform->velocity *= -1;
    }

    ASSERT_EQUAL(1, data->canyon->position_left);
    ASSERT_EQUAL(1, data->platform->position);

}