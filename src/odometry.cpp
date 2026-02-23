#include "odometry.h"

float current_x = 0;
float current_y = 0;
double current_phi = 0;
double absolute_phi = 0;
float phi_offset = 0;  // Offset to align encoder frame with world frame
int32_t previous_encoder_count_R = 0;
int32_t previous_encoder_count_L = 0;
int32_t previous_encoder_timer_CNT_R = 0;
int32_t previous_encoder_timer_CNT_L = 0;

void update_speed(float dt){
    current_speed_L = current_speed_L*0.95 + 0.05*(encoder_count_L - previous_encoder_count_L)*(wheel_diameter_L * PI / CPR * directionL) / dt;
	current_speed_R = current_speed_R*0.95 + 0.05*(encoder_count_R - previous_encoder_count_R)*(wheel_diameter_R * PI / CPR * directionR) / dt;
	speedCalculationPreviousTime = micros();
}
void update_encoder_counts(){

	int32_t delta = encoder_get_R() - previous_encoder_timer_CNT_R;
	delta = (delta>15000)? delta-32768 : (delta<-15000)? delta+32768 : delta; // handling overflow
	encoder_count_R += delta; // it was + not - but the encoder phase pins were inverted

	delta = encoder_get_L() - previous_encoder_timer_CNT_L;
	delta = (delta>15000)? delta-32768 : (delta<-15000)? delta+32768 : delta;
	encoder_count_L += delta;

	previous_encoder_timer_CNT_R = encoder_get_R();
	previous_encoder_timer_CNT_L = encoder_get_L();
}
void update_position(float dt){

	update_encoder_counts();

	update_speed(dt);
	double dR = ticks_to_distance(encoder_count_R - previous_encoder_count_R, wheel_diameter_R);
	double dL = ticks_to_distance(encoder_count_L - previous_encoder_count_L, wheel_diameter_L);
	double dC = (dR+dL)/2;
	previous_encoder_count_R = encoder_count_R;
	previous_encoder_count_L = encoder_count_L;

	double previous_absolute_phi = absolute_phi;
	double encoder_phi = ( ticks_to_distance(encoder_count_R, wheel_diameter_R) - ticks_to_distance(encoder_count_L, wheel_diameter_L) )/wheels_spacing;
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
double ticks_to_distance(int32_t ticks, double wheel_diameter){
	return (ticks*wheel_diameter*PI/CPR);
}
double distance_to_ticks(int32_t distance, double wheel_diameter){
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
