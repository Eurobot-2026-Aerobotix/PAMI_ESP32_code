#pragma once

#include "odometry.h"

float current_x = 63;
float current_y = 675;
float current_phi = deg_to_rad(90);    
float absolute_phi = deg_to_rad(90);
float phi_offset = 0;  // Offset to align encoder frame with world frame
int32_t previous_encoder_count_R = 0;
int32_t previous_encoder_count_L = 0;

void update_speed(){
	float dt = (micros() - speedCalculationPreviousTime);
    current_speed_L = current_speed_L*0.95 + 0.05*(encoder_count_L - previous_encoder_count_L)*(wheel_diameter_L * PI / CPR * directionL)*1000000 / dt;
	current_speed_R = current_speed_R*0.95 + 0.05*(encoder_count_R - previous_encoder_count_R)*(wheel_diameter_R * PI / CPR * directionR)*1000000 / dt;
	speedCalculationPreviousTime = micros();
}

void update_position(){
	
	encoder_count_L = encoder_get_L();
	encoder_count_R = encoder_get_R();

	update_speed();
	float dR = ticks_to_distance(encoder_count_R - previous_encoder_count_R, wheel_diameter_R);
	float dL = ticks_to_distance(encoder_count_L - previous_encoder_count_L, wheel_diameter_L);
	float dC = (dR+dL)/2;
	previous_encoder_count_R = encoder_count_R;
	previous_encoder_count_L = encoder_count_L;

	float previous_absolute_phi = absolute_phi;
	float encoder_phi = ( ticks_to_distance(encoder_count_R, wheel_diameter_R) - ticks_to_distance(encoder_count_L, wheel_diameter_L) )/wheels_spacing;
	absolute_phi = encoder_phi + phi_offset;

	current_x += dC*cos( ( previous_absolute_phi + absolute_phi ) / 2 );
	current_y += dC*sin( ( previous_absolute_phi + absolute_phi ) / 2 );

	current_phi = absolute_phi;
	while (current_phi > PI){
		current_phi -= 2*PI;
	}
	while (current_phi <= -PI){
		current_phi += 2*PI;
	}
}
void reset_position(){
	current_x = 0;
	current_y = 0;
	current_phi = 0;
	absolute_phi = 0;
	phi_offset = 0;
}
void set_position(float current__x, float current__y, float current__phi, float absolute__phi){
	current_x = current__x;
	current_y = current__y;
	current_phi = current__phi;
	absolute_phi = absolute__phi;
	// Calculate offset between desired angle and current encoder angle
	float encoder_phi = ( ticks_to_distance(encoder_get_R(), wheel_diameter_R) - ticks_to_distance(encoder_get_L(), wheel_diameter_L) )/wheels_spacing;
	phi_offset = absolute__phi - encoder_phi;
}
float get_distance_to(float x, float y){
	return sqrt(SQR(current_x - x) + SQR(current_y - y));
}
float get_arc_distance_to(float target_point__x, float target_point__y, float center_point__x, float center_point__y, float R){
	float projection_angle = atan2(current_y - center_point__y , current_x - center_point__x);
	float target_angle = atan2(target_point__y - center_point__y , target_point__x - center_point__x);
	return fabs(R * (projection_angle - target_angle));
}
float get_angle_to(float x, float y){
	return atan2(y - current_y, x - current_x);
}
float ticks_to_distance(int32_t ticks, float wheel_diameter){
	return (ticks*wheel_diameter*PI/CPR);
}
float distance_to_ticks(int32_t distance, float wheel_diameter){
	return (distance*CPR)/(wheel_diameter*PI);
}
float rad_to_deg(float radian){
	return ((radian*180)/PI);
}
float deg_to_rad(float degrees){
	return ((degrees*PI)/180);
}
float normalize_angle(float angle){
	float a = angle;
	while (a > PI){
		a -= 2*PI;
	}
	while (a <= -PI){
		a += 2*PI;
	}
	return a;
}
