#include "command_controller.h"

std::unordered_map<std::string, float> arguments;
enum Command current_command = NONE;
std::mutex robot_state_mutex;

void prepare_move_forward(float distance, float speed)
{
    while (!robot_state_mutex.try_lock())
    {
        vTaskDelay(1);
    }
    arguments["distance"] = distance;
    arguments["speed"] = speed;
    current_command = MOVIN_FORWARD;
}
void prepare_move_backward(float distance, float speed)
{
    while (!robot_state_mutex.try_lock())
    {
        vTaskDelay(1);
    }
    arguments["distance"] = distance;
    arguments["speed"] = speed;
    current_command = MOVIN_BACKWARD;
}
void prepare_rotate(float angle, int8_t is_radian)
{
    while (!robot_state_mutex.try_lock())
    {
        vTaskDelay(1);
    }
    arguments["angle"] = angle;
    arguments["is_radian"] = is_radian;
    current_command = ROTATIN;
}
void prepare_orientate(float orientation, int8_t is_radian)
{
    while (!robot_state_mutex.try_lock())
    {
        vTaskDelay(1);
    }
    arguments["orientation"] = orientation;
    arguments["is_radian"] = is_radian;
    current_command = ORIENTATIN;
}
void prepare_go_to(float x, float y, float speed)
{
    while (!robot_state_mutex.try_lock())
    {
        vTaskDelay(1);
    }
    arguments["x"] = x;
    arguments["y"] = y;
    arguments["speed"] = speed;
    current_command = GOIN_TO;
}
void prepare_move_forward_with_deviation(float distance, float deviation, float speed)
{
    while (!robot_state_mutex.try_lock())
    {
        vTaskDelay(1);
    }
    arguments["distance"] = distance;
    arguments["deviation"] = deviation;
    arguments["speed"] = speed;
    current_command = MOVIN_FORWARD_WITH_DEVIATION;
}
void prepare_circle_curve(float target_x, float target_y, float target_phi, float speed)
{
    while (!robot_state_mutex.try_lock())
    {
        vTaskDelay(1);
    }
    arguments["target_x"] = target_x;
    arguments["target_y"] = target_y;
    arguments["target_phi"] = target_phi;
    arguments["speed"] = speed;
    current_command = CIRCLIN_CURVE_CMD;
}

void execute_command()
{
    switch (current_command)
    {
    case MOVIN_FORWARD:
        move_forward(arguments["distance"], arguments["speed"]);
        robot_state_mutex.unlock();
        break;
    case MOVIN_BACKWARD:
        move_backward(arguments["distance"], arguments["speed"]);
        robot_state_mutex.unlock();
        break;
    case ROTATIN:
        rotate(arguments["angle"], (int8_t)arguments["is_radian"]);
        robot_state_mutex.unlock();
        break;
    case ORIENTATIN:
        orientate(arguments["orientation"], (int8_t)arguments["is_radian"]);
        robot_state_mutex.unlock();
        break;
    case GOIN_TO:
        go_to(arguments["x"], arguments["y"], arguments["speed"]);
        robot_state_mutex.unlock();
        break;
    case MOVIN_FORWARD_WITH_DEVIATION:
        move_forward_with_deviation(arguments["distance"], arguments["deviation"], arguments["speed"]);
        robot_state_mutex.unlock();
        break;
    case CIRCLIN_CURVE_CMD:
        circle_curve(arguments["target_x"], arguments["target_y"], arguments["target_phi"], arguments["speed"]);
        robot_state_mutex.unlock();
        break;
    default:
        vTaskDelay(1);
        break;
    }
    current_command = NONE;
}