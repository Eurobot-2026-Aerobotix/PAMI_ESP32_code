#include "navigation.h"

enum Control_loop control_loop_state = SPEED;
enum Wheel_state wheel_state_R = DEAD_STOP;
enum Wheel_state wheel_state_L = DEAD_STOP;
enum Trajectory trajectory = STRAIGHT;
float ramping_acceleration = 4;
float breaking_acceleration = -5;
float emergency_breaking_acceleration = -10;
float rotating_speed = 200;
float acc_linearization_coef;

float cruise_speed_R;
float cruise_speed_L;
float starting_phi;
float turning_radius;
//float deviation_distance;
int min_distance = 120;

int32_t previous_speed_control_time = 0;

int move_forward(float distance, float speed, float target_point_x, float target_point_y){
	wait_for_idle();
	reset_speed_control_variables();
	direction_R = 1;
	direction_L = 1;

	// all of these formulas are derived from a ramp graph: speed = f(distance)
	float x1 = speed / ramping_acceleration;
	float x2 = speed / breaking_acceleration + distance;

	if (x2 <= x1){ // correcting the cruising speed if it is too high to reach using the current ramp and break
		float intersection_x = (breaking_acceleration * distance) / (breaking_acceleration - ramping_acceleration);
		float intersection_speed = ramping_acceleration * intersection_x;
		speed = 0.9 * intersection_speed;
		x1 = speed / ramping_acceleration;
		x2 = speed / breaking_acceleration + distance;
	}
	acc_linearization_coef = sqrt(x1);

	cruise_speed_R = speed;
	cruise_speed_L = speed;
	
	float distance_to_target = distance;
	float previous_distance_to_target = distance;
	float x = 0;

	wheel_state_R = ACCELERATING;
	wheel_state_L = ACCELERATING;
	trajectory = STRAIGHT;
	control_loop_state = SPEED;
	starting_phi = atan2(target_point_y - current_y, target_point_x - current_x);
	float starting_x = current_x;
	float starting_y = current_y;
	float lane_correction_term;
	float phase_two_distance = 30;
	while (distance_to_target > phase_two_distance){
		x = distance - distance_to_target;
		if (wheel_state_R == ACCELERATING && x > x1){
			wheel_state_R = CRUISING;
			wheel_state_L = CRUISING;
		}else if (wheel_state_R == CRUISING && x > x2){
			wheel_state_R = CONTROLLED_BREAK;
			wheel_state_L = CONTROLLED_BREAK;
		}else if (wheel_state_R == EMERGENCY_BREAK){
			emergency_break();
			return 1;
		}

		float phi_error = normalize_angle(calculate_instant_target_phi(x, distance) - current_phi);
		float deviation_angle = normalize_angle(atan2(current_y - starting_y, current_x - starting_x) - atan2(target_point_y - starting_y, target_point_x - starting_x) );
		float deviation_distance = (-1) * sqrt(SQR(current_y - starting_y) + SQR(current_x - starting_x)) * sin(deviation_angle); // positive to the left and negative to the right
		float phi_correction_term = (phi_error) * ( 2 / ( MAX( ( abs(target_speed_R) + abs(target_speed_L) )/2, 100.0) /500.0) ); // the first coefficient is tunable
		lane_correction_term = (deviation_distance) * ( 0.1 / ( MAX( ( abs(target_speed_R) + abs(target_speed_L) )/2, 100.0) /500.0)) + phi_correction_term; // the first coefficient is tunable
		lane_correction_term = MIN(MAX(lane_correction_term, -0.1), 0.1); // the coefficient is tunable

		set_target_speeds(fabs(calculate_instant_target_speed_R(x, distance)) * (1 + lane_correction_term), fabs(calculate_instant_target_speed_L(x, distance)) * (1 - lane_correction_term));
		custom_delay(4);
		previous_distance_to_target = distance_to_target;
		distance_to_target = distance - get_distance_to(starting_x, starting_y);
	}
	// completing the last part of distance separately, so the break happens on point
	// calculating the distance to move that  results in the robot being the closest to target point, i can't explain the math in code unfortunately
	speed = (target_speed_R + target_speed_L) / 2;
	speed = MAX(fabs(speed), 75);
	int32_t phase_two_start_time = micros();
	while (distance_to_target > 1){
		float phi_error = normalize_angle(calculate_instant_target_phi(x, distance) - current_phi);
		float deviation_angle = normalize_angle(atan2(current_y - starting_y, current_x - starting_x) - atan2(target_point_y - starting_y, target_point_x - starting_x) );
		float deviation_distance = (-1) * sqrt(SQR(current_y - starting_y) + SQR(current_x - starting_x)) * sin(deviation_angle); // positive to the left and negative to the right
		float phi_correction_term = (phi_error) * ( 2 / ( MAX( ( abs(target_speed_R) + abs(target_speed_L) )/2, 100.0) /500.0) ); // the first coefficient is tunable
		lane_correction_term = (deviation_distance) * ( 0.1 / ( MAX( ( abs(target_speed_R) + abs(target_speed_L) )/2, 100.0) /500.0)) + phi_correction_term; // the first coefficient is tunable
		lane_correction_term = MIN(MAX(lane_correction_term, -0.1), 0.1); // the coefficient is tunable

		set_target_speeds(speed * (1 + lane_correction_term), speed * (1 - lane_correction_term));
		speed = MAX(fabs(speed) * 0.995, 75);
		previous_distance_to_target = distance_to_target;
		custom_delay(2);
		distance_to_target = distance - get_distance_to(starting_x, starting_y);

		if (micros() - phase_two_start_time > 4000000)
		{
			break;
		}
		if (wheel_state_R == EMERGENCY_BREAK){
			emergency_break();
			return 1;
		}
	}
	stop_motors();
	float phi_error = normalize_angle(calculate_instant_target_phi(x, distance) - current_phi);
	set_target_encoder_counts(encoder_count_R, encoder_count_L - (phi_error)*wheels_spacing*CPR / (wheel_diameter_L*PI) ); // the second part is there to fix any last minute fuck ups, concerning the phi
	wheel_state_R = DEAD_STOP;
	wheel_state_L = DEAD_STOP;
	return 0;
}

int move_backwards(float distance, float speed){
	// asta3
	wait_for_idle();
	reset_speed_control_variables();
	direction_R = -1;
	direction_L = -1;

	float target_point_x = current_x - distance * cos(current_phi);
	float target_point_y = current_y - distance * sin(current_phi);

	// all of these formulas are derived from a ramp graph: speed = f(distance)
	float x1 = speed / ramping_acceleration;
	float x2 = speed / breaking_acceleration + distance;

	if (x2 <= x1){ // correcting the cruising speed if it is too high to reach using the current ramp and break
		float intersection_x = (breaking_acceleration * distance) / (breaking_acceleration - ramping_acceleration);
		float intersection_speed = ramping_acceleration * intersection_x;
		speed = 0.9 * intersection_speed;
		x1 = speed / ramping_acceleration;
		x2 = speed / breaking_acceleration + distance;
	}
	acc_linearization_coef = sqrt(x1);

	cruise_speed_R = -speed;
	cruise_speed_L = -speed;
	
	float distance_to_target = distance;
	float previous_distance_to_target = distance;
	float x = 0;

	wheel_state_R = ACCELERATING;
	wheel_state_L = ACCELERATING;
	trajectory = STRAIGHT;
	control_loop_state = SPEED;
	starting_phi = atan2(target_point_y - current_y, target_point_x - current_x) + PI;
	float starting_x = current_x;
	float starting_y = current_y;
	float lane_correction_term;
	float phase_two_distance = 30;

	bool right_stopped = false;
	bool left_stopped = false;
	while (distance_to_target > phase_two_distance){
		x = distance - distance_to_target;
		if (wheel_state_R == ACCELERATING && x > x1){
			wheel_state_R = CRUISING;
			wheel_state_L = CRUISING;
		}else if (wheel_state_R == CRUISING && x > x2){
			wheel_state_R = CONTROLLED_BREAK;
			wheel_state_L = CONTROLLED_BREAK;
		}else if (wheel_state_R == EMERGENCY_BREAK){
			emergency_break();
			return 1;
		}

		set_target_speeds(calculate_instant_target_speed_R(x, distance), calculate_instant_target_speed_R(x, distance));
		custom_delay(4);
		previous_distance_to_target = distance_to_target;
		distance_to_target = distance - get_distance_to(starting_x, starting_y);
	}
	debugger = 100;
	wheel_state_R = EMERGENCY_BREAK;
	wheel_state_L = EMERGENCY_BREAK;
	stop_motors();
	int start_time = millis();
	while(fabs(current_speed_L) > 5 || fabs(current_speed_R) > 5){
		custom_delay(10);
		if (millis() - start_time > 500){
			break;
		}
	}
	debugger = 200;
	return 0;
}
int bezier_curve(float x1, float y1, float x2, float y2, float x3, float y3, float speed, int direction){
	float x0 = current_x;
	float y0 = current_y;

	float distance =  get_bezier_distance(0, x0, y0, x1, y1, x2, y2, x3, y3);

	float d_1 = speed/ramping_acceleration;
	float d_2 = speed/breaking_acceleration + distance ;

	if (d_2 <= d_1){ // correcting the cruising speed if it is too high to reach using the current ramp and break
		float intersection_x = (breaking_acceleration*distance )/(breaking_acceleration - ramping_acceleration);
		float intersection_speed = ramping_acceleration * intersection_x;
		speed = 0.9 * intersection_speed;
		d_1 = speed/ramping_acceleration;
		d_2 = speed/breaking_acceleration + distance;
	}

	orientate(atan2(y1-y0, x1-x0) + PI*(-direction+1)/2, 1);

	acc_linearization_coef = sqrt(d_1);

	wait_for_idle();
	reset_speed_control_variables();

	direction_R = 1;
	direction_L = 1;
	cruise_speed_R = speed;
	cruise_speed_L = speed;
	float distance_to_target = distance;
	float d = 0;
	wheel_state_R = ACCELERATING;
	wheel_state_L = ACCELERATING;
	control_loop_state = SPEED;
	trajectory = BEZIER_CURVE;

	float lane_correction_term;
	float t = 0;
	float speed_R , speed_L;


	while (distance_to_target >= 1.0f && t < 1.0000f){
		d = distance - distance_to_target;

		if (wheel_state_R == ACCELERATING && d > d_1 ){
			wheel_state_R = CRUISING;
			wheel_state_L = CRUISING;
		}else if (wheel_state_R == CRUISING && d > d_2){
			wheel_state_R = CONTROLLED_BREAK;
			wheel_state_L = CONTROLLED_BREAK;
		}else if (wheel_state_R == EMERGENCY_BREAK){
			emergency_break();
			return 1;
		}

		t = project_bezier(current_x, current_y, x0, y0, x1, y1, x2, y2, x3, y3);

		float radius = get_bezier_radius(t, x0, y0, x1, y1, x2, y2, x3, y3);
	    speed = direction * calculate_instant_target_speed_R(d, distance);

		float signed_distance = direction * signed_distance_to_bezier(current_x, current_y, x0, y0, x1, y1, x2, y2, x3, y3, t);
		float phi_error = get_target_phi(t, x0, y0, x1, y1, x2, y2, x3, y3) + (PI*(1-direction) )/2 - current_phi ;
		if(phi_error > PI)
			phi_error -= 2*PI ;
		else if (phi_error < -PI)
			phi_error += 2*PI ;
		float phi_correction_term = (phi_error) * ( 5 / ( MAX( ( abs(target_speed_R) + abs(target_speed_L) )/2, 100.0) /500.0) ); // 3 the first coefficient is tunable

		lane_correction_term = MIN(fabs(SQR(signed_distance) * (0.4f)), 0.5)*SGN(signed_distance) + phi_correction_term;// this coefficient is tunable
		if (d < 0.1*d_1){ // 0.002f;
			lane_correction_term = 0;
		}

		if (fabs(radius) < 100000){
			speed_R = ( speed * (radius + direction * (wheels_spacing / 2.0f)) / radius );
			speed_L = ( speed * (radius - direction * (wheels_spacing / 2.0f)) / radius );
		}
		else {
			speed_R = speed ;
			speed_L = speed ;
		}
		if ((fabs(speed_R)>700) || (fabs(speed_L) > 700)){
			speed = direction * 700.0/ MAX((radius + (wheels_spacing / 2.0f)) / (radius), (radius - (wheels_spacing / 2.0f)) / (radius));
			speed_R = ( speed * (radius + direction * (wheels_spacing / 2.0f)) / (radius));
			speed_L = ( speed * (radius - direction * (wheels_spacing / 2.0f)) / (radius));
		}
  
		distance_to_target = get_bezier_distance(t, x0, y0, x1, y1, x2, y2,x3 ,y3);

		set_target_speeds(speed_R*(1 + lane_correction_term) ,speed_L* (1 - lane_correction_term));
		//send_values_to_usart(current_speed_R, current_speed_L, target_speed_R, target_speed_L);
		//send_values_to_usart(signed_distance, 0, 0, 0);
		custom_delay(4);
	}
	stop_motors();
	float target_angle = atan2(y3-y2 , x3-x2) + PI*(-direction+1)/2;
	float diff = absolute_phi - target_angle;

	while (diff >  PI) { target_angle += 2*PI; diff -= 2*PI; }
	while (diff < -PI) { target_angle -= 2*PI; diff += 2*PI; }

	float position_correction = ( ( (target_angle - absolute_phi) * wheels_spacing*CPR / ( ((wheel_diameter_L + wheel_diameter_R)/2)*PI) ) ) / 2.0f;
	set_target_encoder_counts(encoder_count_R + position_correction, encoder_count_L - position_correction); // the second part is there to fix any last minute fuck ups, concerning the phi
	wheel_state_R = DEAD_STOP;
	wheel_state_L = DEAD_STOP;
	return 0;
}
float calculate_instant_target_speed_R_full(float x, float distance, float ramping__acceleration, float breaking__acceleration)
{
	if (wheel_state_R == ACCELERATING){
		if (direction_R == 1)
		{
			return MAX(MIN(ramping__acceleration * sqrt(x)*acc_linearization_coef, cruise_speed_R), 50);
		}
		else
		{
			return MIN(MAX(-ramping__acceleration * sqrt(x)*acc_linearization_coef, cruise_speed_R), -50);
		}
	}
	else if (wheel_state_R == CRUISING)
	{
		return cruise_speed_R;
	}
	else if (wheel_state_R == CONTROLLED_BREAK)
	{
		if (direction_R == 1)
		{
			return MAX(breaking__acceleration * (x - distance), 50);
		}
		else
		{
			return MIN(-breaking__acceleration * (x - distance), -50);
		}
	}
	else if (wheel_state_R == EMERGENCY_BREAK)
	{
		if (direction_R == 1)
		{
			return MAX(target_speed_R + emergency_breaking_acceleration * x, 0);
		}
		else
		{
			return MIN(target_speed_R - emergency_breaking_acceleration * x, 0);
		}
	}
	return 0; // just to prevent debugger error, will be erased once a definitive else is added
}
float calculate_instant_target_speed_L_full(float x, float distance, float ramping__acceleration, float breaking__acceleration)
{
	if (wheel_state_L == ACCELERATING){
		if (direction_L == 1)
		{
			return MAX(MIN(ramping__acceleration * sqrt(x)*acc_linearization_coef, cruise_speed_L), 50);
		}
		else
		{
			return MIN(MAX(-ramping__acceleration * sqrt(x)*acc_linearization_coef, cruise_speed_L), -50);
		}
	}
	else if (wheel_state_L == CRUISING)
	{
		return cruise_speed_L;
	}
	else if (wheel_state_L == CONTROLLED_BREAK)
	{
		if (direction_L == 1)
		{
			return MAX(breaking__acceleration * (x - distance), 0);
		}
		else
		{
			return MIN(-breaking__acceleration * (x - distance), 0);
		}
	}
	else if (wheel_state_L == EMERGENCY_BREAK)
	{
		if (direction_L == 1)
		{
			return MAX(target_speed_L + emergency_breaking_acceleration * x, 0);
		}
		else
		{
			return MIN(target_speed_L - emergency_breaking_acceleration * x, 0);
		}
	}
	return 0; // just to prevent debugger error, will be erased once a definitive else is added
}
float calculate_instant_target_speed_R(float x, float distance)
{
	return calculate_instant_target_speed_R_full(x, distance, ramping_acceleration, breaking_acceleration);
}
float calculate_instant_target_speed_L(float x, float distance)
{
	return calculate_instant_target_speed_L_full(x, distance, ramping_acceleration, breaking_acceleration);
}
float calculate_instant_target_phi(float x, float distance)
{
	if (trajectory == STRAIGHT)
	{
		return starting_phi;
	}
	else if (trajectory == CIRCLE_CURVE)
	{
		return starting_phi - x / turning_radius;
	}
	return 0;
}
int rotate(float angle, int8_t is_radian){
	wait_for_idle();
	reset_speed_control_variables();
	float rotating_breaking_acceleration = breaking_acceleration;
	float rotating_ramping_acceleration = 4;
	angle = is_radian ? angle : deg_to_rad(angle);
	float arc_length = (wheels_spacing / 2) * angle;

	direction_R = SGN(angle);
	direction_L = -direction_R;

	// all of these formulas are derived from a ramp graph: speed = f(distance)
	float speed = rotating_speed;
	float distance = fabs(arc_length);
	float x1 = speed / rotating_ramping_acceleration;
	float x2 = speed / rotating_breaking_acceleration + distance;

	if (x2 <= x1)
	{ // correcting the cruising speed if it is too high to reach using the current ramp and break
		float intersection_x = (rotating_breaking_acceleration * distance) / (rotating_breaking_acceleration - rotating_ramping_acceleration);
		float intersection_speed = rotating_ramping_acceleration * intersection_x;
		speed = 0.9 * intersection_speed;
		x1 = speed / rotating_ramping_acceleration;
		x2 = speed / rotating_breaking_acceleration + distance;
	}

	cruise_speed_R = speed * direction_R;
	cruise_speed_L = speed * direction_L;

	// I will track only the right wheel cause both wheels need to be in sync
	float distance_to_target = distance;
	float previous_distance_to_target = distance;
	float x;
	float target_phi = absolute_phi + angle;

	// Yaw centering, preventing that a wheel goes faster than the other and moves the rotation axis away from center
	float distance_traveled_R;
	float distance_traveled_L;
	const int32_t starting_encoder_count_R = encoder_count_R;
	const int32_t starting_encoder_count_L = encoder_count_L;
	float yaw_centering_term;

	float phase_two_angle = 15;

	wheel_state_R = ACCELERATING;
	wheel_state_L = ACCELERATING;
	trajectory = ROTATING;
	control_loop_state = SPEED;
	while (fabs(target_phi - absolute_phi) > deg_to_rad(phase_two_angle))
	{
		x = distance - distance_to_target;
		if (wheel_state_R == ACCELERATING && x > x1){
			wheel_state_R = CRUISING;
			wheel_state_L = CRUISING;
		}
		else if (wheel_state_R == CRUISING && x > x2){
			wheel_state_R = CONTROLLED_BREAK;
			wheel_state_L = CONTROLLED_BREAK;
		}
		if (millis() - strategy_start_time >= max_running_time){
			emergency_break();
			return 1;
		}

		distance_traveled_R = fabs(encoder_count_R - starting_encoder_count_R) * wheel_diameter_R * PI / CPR;
		distance_traveled_L = fabs(encoder_count_L - starting_encoder_count_L) * wheel_diameter_L * PI / CPR;

		yaw_centering_term = MIN((distance_traveled_R - distance_traveled_L) * 0.05, 1); // the coefficient is tunable
		yaw_centering_term = MAX(yaw_centering_term, -1);
		if (x < 0.1 * x1){
			yaw_centering_term = 0;
		}
		speed = calculate_instant_target_speed_R_full(x, distance, rotating_ramping_acceleration, rotating_breaking_acceleration);
		speed = MAX(fabs(speed), 80);
		speed = speed * direction_R;
		set_target_speeds(speed * (1 - yaw_centering_term), -speed * (1 + yaw_centering_term));
		custom_delay(4);
		previous_distance_to_target = distance_to_target;
		distance_to_target = (wheels_spacing / 2) * fabs(target_phi - absolute_phi);
		distance_to_target = MIN(distance_to_target, distance);
	}
	// completing the last part of distance separately, so the break happens on point

	speed = fabs(speed)*direction_R;
	distance_to_target = (wheels_spacing/2) * fabs(target_phi - absolute_phi);
	previous_distance_to_target = distance_to_target;

	uint32_t phase_two_start_time = micros();
	while (fabs(target_phi - absolute_phi) > deg_to_rad(5) && distance_to_target <= previous_distance_to_target + 10){
		distance_traveled_R = (encoder_count_R - starting_encoder_count_R) * wheel_diameter_R * PI / CPR;
		distance_traveled_L = (encoder_count_L - starting_encoder_count_L) * wheel_diameter_L * PI / CPR;
		yaw_centering_term = MIN((fabs(distance_traveled_R) - fabs(distance_traveled_L)) * 0.1, 1);
		yaw_centering_term = MAX(yaw_centering_term, -1);
		yaw_centering_term = 0;

		set_target_speeds(speed * (1 - yaw_centering_term), -speed * (1 + yaw_centering_term));
		speed = MAX(fabs(speed) * 0.985, 80) * direction_R;
		previous_distance_to_target = distance_to_target;
		custom_delay(2);
		distance_to_target = (wheels_spacing/2) * fabs(target_phi - absolute_phi);
		distance_to_target = MIN(distance_to_target, distance);
		if (millis() - strategy_start_time >= max_running_time){
			emergency_break();
			return 1;
		}
		
		if (micros() - phase_two_start_time > 3000000)
		{
			break;
		}
	}
	stop_motors();
	set_target_encoder_counts(encoder_count_R, encoder_count_L - (target_phi - absolute_phi)*wheels_spacing*CPR / (wheel_diameter_L*PI) ); // the second part is there to fix any last minute fuck ups, concerning phi
	wheel_state_R = DEAD_STOP;
	wheel_state_L = DEAD_STOP;
	return 0;
}
int orientate(float orientation, int8_t is_radian)
{
	orientation = (is_radian) ? orientation : deg_to_rad(orientation);
	int result = 0;
	wait_for_idle();
	if (abs(orientation - current_phi) > deg_to_rad(0.5)){
		float angle = orientation - current_phi;
		if (angle > PI){
			angle -= 2 * PI;
		}
		else if (angle < -PI){
			angle += 2 * PI;
		}
		result = rotate(angle, 1);
	}
	return result;
}
int go_to(float x, float y, float speed)
{
	wait_for_idle();
	float target_angle = get_angle_to(x, y);

	if (orientate(target_angle, 1)){
		return 1;
	}
	wait_for_idle();
	int result = move_forward(get_distance_to(x, y), speed, x, y) * 2; // why *2 ? cause fuck you
	return result;
}
void emergency_break()
{
	wheel_state_R = EMERGENCY_BREAK;
	wheel_state_L = EMERGENCY_BREAK;
}
void reset_speed_control_variables(){
	previous_speed_control_time = micros()-999;
	speed_integral_L = 0;
	speed_integral_R = 0;
	speed_derivative_L = 0;
	speed_derivative_R = 0;
	PWM_L = 0;
	PWM_R = 0;
	previous_speed_error_L = 0;
	previous_speed_error_R = 0;
}
bool is_idle(){
	int encoder_tolerance = 3;
	int pwm_tolerance = 500;
	bool right_wheel_idle = (fabs(target_encoder_count_R - encoder_count_R) < encoder_tolerance) && (fabs(PWM_R) < pwm_tolerance) && (fabs(previous_position_error_R) < encoder_tolerance);
	bool left_wheel_idle = (fabs(target_encoder_count_L - encoder_count_L) < encoder_tolerance) && (fabs(PWM_L) < pwm_tolerance) && (fabs(previous_position_error_L) < encoder_tolerance);
	return right_wheel_idle && left_wheel_idle;
}
void set_target_encoder_counts(int32_t encoder__count_R, int32_t encoder__count_L){
	reset_position_control_variables();
	target_encoder_count_R = encoder__count_R;
	target_encoder_count_L = encoder__count_L;
	position_integral_R = 0;
	position_derivative_R = 0;
	position_integral_L = 0;
	position_derivative_L = 0;
	control_loop_state = POSITION;
}
void wait_for_obstacle_clearance() {
	for(int i=0; i<2; i++){
		while(1){
			if (get_ultrasonic_distance() >= min_distance + 20){ // adding a hysteresis of 20 mm to prevent the robot from stopping and starting repeatedly if the obstacle is on the edge of the ultrasonic sensor range
				custom_delay(10);
				if (get_ultrasonic_distance() >= min_distance + 20){ // adding a hysteresis of 20 mm to prevent the robot from stopping and starting repeatedly if the obstacle is on the edge of the ultrasonic sensor range
					break;
				}
			}
			custom_delay(100); // wait for 100 ms before checking again
		}
	}
	while(millis() - strategy_start_time >= max_running_time){
		control_loop_state = DISACTIVATED;
		stop_motors();
	}
}
void wait_for_idle(){
	int start_time = millis();
	while(!is_idle() && !(current_speed_L == 0 && current_speed_R == 0) && (wheel_state_R != EMERGENCY_BREAK) && (wheel_state_L != EMERGENCY_BREAK) ){
		custom_delay(10);
		if (millis() - start_time > 700){
			break;
		}
	}
}
void prepare_emergency_break(){
	float d = get_ultrasonic_distance();
	float delta_mm = max(0.0f, d - min_distance);
	int32_t delta_ticks = (int32_t) (delta_mm * CPR / (wheel_diameter_R * PI));
	set_target_encoder_counts(encoder_count_R + delta_ticks, encoder_count_L + delta_ticks);
	wheel_state_R = EMERGENCY_BREAK;
	wheel_state_L = EMERGENCY_BREAK;
}