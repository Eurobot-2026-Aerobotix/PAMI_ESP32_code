#pragma once

#include <Arduino.h>
#include "navigation.h"
#include <string>
#include <unordered_map>
#include <mutex>

enum Command
{
    NONE,
    MOVIN_FORWARD,
    MOVIN_BACKWARD,
    ROTATIN,
    ORIENTATIN,
    GOIN_TO,
    MOVIN_FORWARD_WITH_DEVIATION,
    CIRCLIN_CURVE_CMD
};

extern std::unordered_map<std::string, float> arguments;
extern enum Command current_command;
extern std::mutex robot_state_mutex;

void prepare_move_forward(float distance, float speed);
void prepare_move_backward(float distance, float speed);
void prepare_rotate(float angle, int8_t is_radian);
void prepare_orientate(float orientation, int8_t is_radian);
void prepare_go_to(float x, float y, float speed);
void prepare_move_forward_with_deviation(float distance, float deviation, float speed);
void prepare_circle_curve(float target_x, float target_y, float target_phi, float speed);

void execute_command();