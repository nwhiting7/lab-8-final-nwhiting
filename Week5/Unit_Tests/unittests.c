#include <stdlib.h>
#include "ctest.h"
#include <stdio.h>
#include "cuttingpoint1.h"

// Note: the name in the first parameter slot must match all tests in that group
CTEST_DATA(cutpointtests) {
    struct task_t task[3];
    int size;
};

CTEST_SETUP(cutpointtests) {
    int execution[] = {1, 2, 3};
    data->size = sizeof(execution) / sizeof(execution[0]);
    init(data->task, execution, data->size);
    first_come_first_served(data->task, data->size);
}
//Ensure right side of capsense slider initiaties correct velocity direction and max force.
CTEST2(cutpointtests, test_process_1){
    struct game_platform test;
    test = slider_force_task(4.0, 3.0, 0);
    ASSERT_EQUAL(4.0, test.max_force);
    ASSERT_EQUAL(3.0, test.velocity);
}
//Ensure friction decreases velocity when platform is moving to the right of the screen.
CTEST2(cutpointtests, test_process_2){
    struct game_platform test;
    test = slider_force_task(0.0, 4.0, 2);
    ASSERT_EQUAL(3.6, test.velocity);
    ASSERT_EQUAL(0, test.max_force);
}
//Ensure left side of capsense slider initiaties correct velocity direction and max force.
CTEST2(cutpointtests, test_process_3){
    struct game_platform test;
    test = slider_force_task(5.0, 2.0, 1);
    ASSERT_EQUAL(-5.0, test.velocity);
    ASSERT_EQUAL(2.0, test.max_force);
}
//Ensure friction decreases velocity even when no direction and force are present.
CTEST2(cutpointtests, test_process_4){
    struct game_platform test;
    test = slider_force_task(0.0, 10, 0);
    ASSERT_EQUAL(8, test.velocity);
}
//ensure kinematic equations are correct.
CTEST2(cutpointtests, test_process_5){
    struct game_platform test;
    test.mass = 3.0;
    test.max_force = 2.0;
    test.timer_update = 0.1;
    test.velocity = 4;
    test = physics_task(test);
    ASSERT_EQUAL(4.06, test.velocity);
    ASSERT_EQUAL(0.406, test.position);
}
//ensure platoform stops at right cayon wall when bounce is not enabled.
CTEST2(cutpointtests, test_process_6){
    struct game_platform test;
    test.mass = 3.0;
    test.max_force = 2.0;
    test.timer_update = 0.1;
    test.velocity = 4;
    test.position = 0;
    test.least_position = 8;
    test.most_position = 16;
    test = physics_task(test);
    ASSERT_EQUAL(4.06, test.velocity);
    ASSERT_EQUAL(8, test.position);
}
//ensure platoform stops at left canyon wall when bounce is not enabled.
CTEST2(cutpointtests, test_process_7){
    struct game_platform test;
    test.mass = 3.0;
    test.max_force = 2.0;
    test.timer_update = 0.1;
    test.velocity = 4;
    test.position = 36;
    test.least_position = 8;
    test.most_position = 16;
    test = physics_task(test);
    ASSERT_EQUAL(4.06, test.velocity);
    ASSERT_EQUAL(16, test.position);
}
//ensure platoform bounces off left canyon wall when bounce is enabled
CTEST2(cutpointtests, test_process_8){
    struct game_platform test;
    test.mass = 3.0;
    test.max_force = 2.0;
    test.timer_update = 0.1;
    test.velocity = 4;
    test.position = 0;
    test.least_position = 8;
    test.most_position = 16;
    test = physics_task(test);
    ASSERT_EQUAL(4.06, test.velocity);
    ASSERT_EQUAL(8.4, test.position);
}
//ensure platform bounces off right canyon wall
CTEST2(cutpointtests, test_process_9){
    struct game_platform test;
    test.mass = 3.0;
    test.max_force = 2.0;
    test.timer_update = 0.1;
    test.velocity = 4;
    test.position = 36;
    test.least_position = 8;
    test.most_position = 16;
    test = physics_task(test);
    ASSERT_EQUAL(-4.06, test.velocity);
    ASSERT_EQUAL(15.6, test.position);
}
//ensure platoform is destroyed when above max velocity.
CTEST2(cutpointtests, test_process_10){
    struct game_platform test;
    test.mass = 3.0;
    test.max_force = 2.0;
    test.timer_update = 0.1;
    test.velocity = 4;
    test.position = 36;
    test.least_position = 8;
    test.most_position = 16;
    test.max_speed = 4;
    test.destroyed = false;
    test = physics_task(test);
    ASSERT_EQUAL(-4.06, test.velocity);
    ASSERT_EQUAL(16, test.position);
    ASSERT_EQUAL(true, test.destroyed);

    struct game_platform test;
    test.mass = 3.0;
    test.max_force = 2.0;
    test.timer_update = 0.1;
    test.velocity = 4;
    test.position = 0;
    test.least_position = 8;
    test.most_position = 16;
    test.max_speed = 4;
    test.destroyed = false;
    test = physics_task(test);
    ASSERT_EQUAL(4.06, test.velocity);
    ASSERT_EQUAL(8.4, test.position);
    ASSERT_EQUAL(4, test.max_speed);
    ASSERT_EQUAL(true, test.destroyed);
}



