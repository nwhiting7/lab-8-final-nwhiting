
#ifndef __FIRST_COME_FIRST_SERVED__
#define __FIRST_COME_FIRST_SERVED__

//----------------------------------------------------------------------------------------------------------------------------------
/// @brief Structure which holds the Task information
//----------------------------------------------------------------------------------------------------------------------------------
struct slider_info {

    /// Process number for the task
    int button_pressed;

    // Amount of time the task takes to execute
    int ind_armed;

    // Amount of time the task spends waiting to be executed
    int ind_charging;

    // Amount of time the task spends in the queue
    int use_laser;

    int num_activations;
};

struct direction_info{
    int direction;
    int max_force;
};

struct platform_info{
    int velocity;
    int position;
    int max_force;
    int mass;
};

struct canyon_info{
    int x_start;
    int x_end;
    int position_left;
    int position_right;
    int bounce_enable;
};

//----------------------------------------------------------------------------------------------------------------------------------
/// @brief Intialize the task array
///
/// @param[in] task The buffer containing task data
/// @param[in] execution The execution time for each task
/// @param[in] size The size of the buffer
//----------------------------------------------------------------------------------------------------------------------------------
void init(struct slider_info *task, int button_pressed, int ind_armed, int ind_charging, int use_laser, int num_activations);

void init_dir(struct direction_info *task, int direction, int max_force);

void init_plat(struct platform_info *task, int velocity, int position, int max_force, int mass);

void canyon_init(struct canyon_info *task, int x_start, int x_end, int position_left, int position_right, int bounce_enable); 

// //----------------------------------------------------------------------------------------------------------------------------------
// /// @brief Run the first come first served algorithm and
// /// calculate the wait and turn around time for each task
// ///
// /// @param[in] task The buffer containing task data
// /// @param[in] size The size of the buffer
// //----------------------------------------------------------------------------------------------------------------------------------
// void first_come_first_served(struct task_t *task, int size);

// //----------------------------------------------------------------------------------------------------------------------------------
// /// @brief Calculates the average wait time.
// ///
// /// @param[in] task The buffer containing task data
// /// @param[in] size The size of the buffer
// ///
// /// @return The average wait time.
// //----------------------------------------------------------------------------------------------------------------------------------
// float calculate_average_wait_time(struct task_t *task, int size);

// //----------------------------------------------------------------------------------------------------------------------------------
// /// @brief Calculates the average turn around time.
// ///
// /// @param[in] task The buffer containing task data
// /// @param[in] size The size of the buffer
// ///
// /// @return The average turn around time.
// //----------------------------------------------------------------------------------------------------------------------------------
// float calculate_average_turn_around_time(struct task_t *task, int size);

#endif // __FIRST_COME_FIRST_SERVED__