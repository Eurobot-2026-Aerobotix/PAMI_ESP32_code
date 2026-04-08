#include "command_controller.h"
#include "navigation.h"

std::unordered_map<std::string, float> arguments;
enum Command current_command = NONE;
std::mutex robot_state_mutex;

void prepare_rotate(float angle, int8_t is_radian)
{
    /*
    while (!robot_state_mutex.try_lock())
    {
        vTaskDelay(1);
    }
    */
    arguments["angle"] = angle;
    arguments["is_radian"] = is_radian;
    current_command = ROTATIN;
}
void prepare_orientate(float orientation, int8_t is_radian)
{
    /*
    while (!robot_state_mutex.try_lock())
    {
        vTaskDelay(1);
    }
    */
    arguments["orientation"] = orientation;
    arguments["is_radian"] = is_radian;
    current_command = ORIENTATIN;
}
void prepare_go_to(float x, float y, float speed)
{
    /*
    while (!robot_state_mutex.try_lock())
    {
        vTaskDelay(1);
    }
    */
    target_x = x;
    target_y = y;
    //has_target = true;
    arguments["x"] = x;
    arguments["y"] = y;
    arguments["speed"] = speed;
    current_command = GOIN_TO;
}
void execute_command()
{
    switch (current_command)
    {
    case ROTATIN:
        rotate(arguments["angle"], (int8_t)arguments["is_radian"]);
        break;
    case ORIENTATIN:
        orientate(arguments["orientation"], (int8_t)arguments["is_radian"]);
        break;
    case GOIN_TO:
        go_to(arguments["x"], arguments["y"], arguments["speed"]);
        break;
    default:
        vTaskDelay(10);
        break;
    }
    current_command = NONE;
}