#include "navigation.h"

enum Control_loop control_loop_state = SPEED;
enum Wheel_state wheel_state_R = DEAD_STOP;
enum Wheel_state wheel_state_L = DEAD_STOP;
enum Trajectory trajectory = STRAIGHT;
float ramping_acceleration = 10;// 5;
float breaking_acceleration = -2;//-5.5;
float emergency_breaking_acceleration = -10;
float rotating_speed = 300;

float cruise_speed_R;
float cruise_speed_L;
float starting_phi;
float turning_radius;
int min_distance = 70;

int32_t previous_speed_control_time = 0;
static inline float d_acc_for_v(float v, float amax, float jerk) {
    return 0.5f * v * (amax * amax + jerk * v) / (amax * jerk);
}
int move_distance_2(float distance, float vmax, float amax, float jerk) {
	reset_speed_control_variables();
	control_loop_state = SPEED;
    const float eps = 1e-8f;
    if (fabsf(distance) < eps) return 0;

    float dir = (distance >= 0.0f) ? 1.0f : -1.0f;
    float D = fabsf(distance);

    if (amax <= eps || jerk <= eps) {
        set_target_speeds(dir * vmax, dir * vmax);
        return 0;
    }
    float v_thresh = (amax * amax) / jerk;
    float dacc_vmax = d_acc_for_v(vmax, amax, jerk);
    float dmin = 2.0f * dacc_vmax;

    float v_peak = vmax;
    float t_cruise = 0.0f;

    if (D >= dmin) {
        v_peak = vmax;
        t_cruise = (D - dmin) / vmax;
    } else {
        float a_q = jerk;
        float b_q = amax * amax;
        float c_q = -amax * jerk * D;
        float disc = b_q * b_q - 4.0f * a_q * c_q;
        if (disc < 0.0f) disc = 0.0f;
        float v_cand = (-b_q + sqrtf(disc)) / (2.0f * a_q);
        if (v_cand >= v_thresh) {
            v_peak = v_cand;
            t_cruise = 0.0f;
        } else {
            v_peak = powf((D * sqrtf(jerk) / 2.0f), 2.0f / 3.0f);
            t_cruise = 0.0f;
        }
    }
    int branchA = (v_peak >= v_thresh) ? 1 : 0;

    float dt = 0.00165f;
    float T_total = 0.0f;
    float ta = amax / jerk;
    float t_const = 0.0f;
    float t1=0,t2=0,t3=0,t4=0,t5=0,t6=0,t7=0;

    if (branchA) {
        t_const = v_peak / amax - ta;
        if (t_const < 0.0f) t_const = 0.0f;
        t1 = ta;
        t2 = t1 + t_const;
        t3 = t2 + ta;
        t4 = t3 + t_cruise;
        t5 = t4 + ta;
        t6 = t5 + t_const;
        t7 = t6 + ta;
        T_total = t7;
    } else {
        ta = powf(v_peak / jerk, 0.5f);
        t1 = ta;
        t2 = t1;
        t3 = 2*ta;
        t4 = t3 + t_cruise;
        t5 = t4 + ta;
        t6 = t5;
        t7 = t5 + ta;
        T_total = t7;
    }
    float x = 0.0f;
    float v = 0.0f;
    float acc = 0.0f;
    float Jerk = 0.0f;
    int steps = (int)ceil(T_total / dt);

    float target_phi = current_phi;

    for (int i = 0; i < steps; ++i) {
        float t = i * dt;
        if (t < t1) {Jerk = jerk;
        	acc = Jerk * t;}
        else if (t < t2){Jerk = 0.0f;
        	acc = amax; }
        else if (t < t3){Jerk = - jerk ;
        	acc = amax + Jerk * (t - t2);}
        else if (t < t4) {Jerk = 0.0f;
        	acc = 0.0f;}
        else if (t < t5) {Jerk = - jerk;
        	acc = Jerk * (t - t4);}
        else if (t < t6) {Jerk = 0.0f;
        	acc = -amax;}
        else if (t < t7) {Jerk = jerk;
        	acc = -amax + Jerk * (t - t6);}
        else acc = 0.0f;

        v += acc * dt;
        if (v > v_peak) v = v_peak;
        if (v < 0) v = 0 ;
        x += v * dt;
        if (x >= D) {
            x = D;
            v = 0.0f;
            acc = 0.0f;
            set_target_speeds(0.0f, 0.0f);
            break;
        }
        //float corr = lane_PID(calculate_instant_target_phi(x, distance) - absolute_phi);
        float vR = dir * v *(1);
        float vL = dir * v *(1);
        set_target_speeds(vR, vL);
		send_values_to_usart(current_speed_L, current_speed_R, target_speed_L, target_speed_R);

    }
    stop_motors();
    delay(4);
    return 0;

}
int move_forward(float distance, float speed){
	while(!is_idle()){
		custom_delay(10);
		Serial.println("Waiting for idle before moving forward");
	};
	custom_delay(200);
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

	cruise_speed_R = speed;
	cruise_speed_L = speed;
	float target_point_x = current_x + distance * cos(current_phi);
	float target_point_y = current_y + distance * sin(current_phi);
	float distance_to_target = distance;
	float previous_distance_to_target = distance;
	float x = 0;

	wheel_state_R = ACCELERATING;
	wheel_state_L = ACCELERATING;
	trajectory = STRAIGHT;
	control_loop_state = SPEED;
	starting_phi = absolute_phi;
	float lane_correction_term;
	float phase_two_distance = 20;
	while (distance_to_target > phase_two_distance){
		x = distance - distance_to_target;
		if (wheel_state_R == ACCELERATING && x > x1){
			wheel_state_R = CRUISING;
			wheel_state_L = CRUISING;
		}else if (wheel_state_R == CRUISING && x > x2){
			wheel_state_R = CONTROLLED_BREAK;
			wheel_state_L = CONTROLLED_BREAK;
		}else if (wheel_state_R == EMERGENCY_BREAK){
			return 1;
		}

		lane_correction_term = MAX(MIN((calculate_instant_target_phi(x, distance) - absolute_phi) * 0, 0), 0); // the coefficient is tunable// 10 1 -1

		set_target_speeds(fabs(calculate_instant_target_speed_R(x, distance)) * (1 + lane_correction_term), fabs(calculate_instant_target_speed_R(x, distance)) * (1 - lane_correction_term));
		custom_delay(4);
		previous_distance_to_target = distance_to_target;
		distance_to_target = get_distance_to(target_point_x, target_point_y);
		distance_to_target = MIN(distance_to_target, distance);

		send_values_to_usart(current_speed_L, current_speed_R, target_speed_L, target_speed_R);
	}
	// completing the last part of distance separately, so the break happens on point
	// calculating the distance to move that  results in the robot being the closest to target point, i can't explain the math in code unfortunately
	float x_closest = (target_point_y - current_y + tan(current_phi) * current_x - tan(current_phi - PI / 2) * target_point_x) / (tan(current_phi) - tan(current_phi - PI / 2));
	float y_closest = tan(current_phi) * (x_closest - current_x) + current_y;
	speed = (target_speed_R + target_speed_L) / 2;
	//speed = MAX(speed, 100);
	distance_to_target = get_distance_to(x_closest, y_closest);
	previous_distance_to_target = distance_to_target;
	int32_t phase_two_start_time = micros();
	while (distance_to_target > 0.1 && distance_to_target <= previous_distance_to_target){
		lane_correction_term = MAX(MIN((calculate_instant_target_phi(x, distance) - absolute_phi) * 0, 0), 0); // the coefficient is tunable
		set_target_speeds(speed * (1 + lane_correction_term), speed * (1 - lane_correction_term));
		//speed = MAX(speed * 0.995, 100);
		previous_distance_to_target = distance_to_target;
		custom_delay(2);
		distance_to_target = get_distance_to(x_closest, y_closest);

		if (micros() - phase_two_start_time > 4000000)
		{
			break;
		}
		if (wheel_state_R == EMERGENCY_BREAK){
			return 1;
		}
		send_values_to_usart(current_speed_L, current_speed_R, target_speed_L, target_speed_R);
	}
	stop_motors();
	//set_target_encoder_counts(encoder_count_R, encoder_count_L - (calculate_instant_target_phi(distance, distance) - absolute_phi)*wheels_spacing*CPR / (wheel_diameter_L*PI) ); // the second part is there to fix any last minute fuck ups, concerning the phi
	wheel_state_R = DEAD_STOP;
	wheel_state_L = DEAD_STOP;
	return 0;
}
int stop(float a ,float b){
	stop_motors();
	return 0;
}
int move_backward(float distance, float speed)
{
	while(!is_idle()){
		custom_delay(10);
	};
	custom_delay(200);
	reset_speed_control_variables();
	speed = (speed < 0) ? -speed : speed;
	distance = (distance < 0) ? -distance : distance;
	direction_R = -1;
	direction_L = -1;

	// all of these formulas are derived from a ramp graph: speed = f(distance)
	float x1 = speed / ramping_acceleration;
	float x2 = speed / breaking_acceleration + distance;

	if (x2 <= x1)
	{ // correcting the cruising speed if it is too high to reach using the current ramp and break
		float intersection_x = (breaking_acceleration * distance) / (breaking_acceleration - ramping_acceleration);
		float intersection_speed = ramping_acceleration * intersection_x;
		speed = 0.9 * intersection_speed;
		x1 = speed / ramping_acceleration;
		x2 = speed / breaking_acceleration + distance;
	}

	cruise_speed_R = -speed;
	cruise_speed_L = -speed;
	float target_point_x = current_x - distance * cos(current_phi);
	float target_point_y = current_y - distance * sin(current_phi);
	float distance_to_target = distance;
	float previous_distance_to_target = distance;
	float x = 0;

	wheel_state_R = ACCELERATING;
	wheel_state_L = ACCELERATING;
	trajectory = STRAIGHT;
	control_loop_state = SPEED;
	starting_phi = absolute_phi;
	float lane_correction_term;
	float phase_two_distance = 20;
	while (distance_to_target > phase_two_distance)
	{
		x = distance - distance_to_target;
		if (wheel_state_R == ACCELERATING && x > x1)
		{
			wheel_state_R = CRUISING;
			wheel_state_L = CRUISING;
		}
		else if (wheel_state_R == CRUISING && x > x2)
		{
			wheel_state_R = CONTROLLED_BREAK;
			wheel_state_L = CONTROLLED_BREAK;
		}
		else if (wheel_state_R == EMERGENCY_BREAK)
		{
			return 1;
		}

		lane_correction_term = (calculate_instant_target_phi(x, distance) - absolute_phi) * 10; // the coefficient is tunable
		if (x < 0.1 * x1)
		{
			lane_correction_term = 0;
		}
		set_target_speeds(calculate_instant_target_speed_R(x, distance) * (1 - lane_correction_term), calculate_instant_target_speed_L(x, distance) * (1 + lane_correction_term));
		custom_delay(4);
		previous_distance_to_target = distance_to_target;
		distance_to_target = get_distance_to(target_point_x, target_point_y);
	}
	// completing the last part of distance separately, so the break happens on point
	// calculating the distance to move that results in the robot being the closest to target point, i can't explain the math in code unfortunately
	float x_closest = (target_point_y - current_y + tan(current_phi) * current_x - tan(current_phi - PI / 2) * target_point_x) / (tan(current_phi) - tan(current_phi - PI / 2));
	float y_closest = tan(current_phi) * (x_closest - current_x) + current_y;
	speed = (target_speed_R + target_speed_L) / 2;
	if (fabs(speed) < phase_two_distance * (-breaking_acceleration))
	{ // handling the case where the robot skips the first while loop, being too close to target_distance
		speed = -phase_two_distance * breaking_acceleration;
	}
	distance_to_target = get_distance_to(x_closest, y_closest);
	previous_distance_to_target = distance_to_target;

	int32_t phase_two_start_time = micros();
	while (distance_to_target > 0.1 && distance_to_target <= previous_distance_to_target)
	{
		lane_correction_term = (calculate_instant_target_phi(x, distance) - absolute_phi) * 20; // the coefficient is tunable
		set_target_speeds(speed * (1 - lane_correction_term), speed * (1 + lane_correction_term));
		speed = MIN(speed * 0.995, 0);
		previous_distance_to_target = distance_to_target;
		custom_delay(2);
		distance_to_target = get_distance_to(x_closest, y_closest);

		if (micros() - phase_two_start_time > 4000000)
		{
			break;
		}
		if (wheel_state_R == EMERGENCY_BREAK){
			return 1;
		}
	}
	stop_motors();
	int32_t phi_error = (int32_t)( (calculate_instant_target_phi(distance, distance) - absolute_phi)*wheels_spacing*CPR / (wheel_diameter_L*PI) ); // this part is there to fix any last minute fuck ups, concerning the phi
	set_target_encoder_counts(encoder_count_R + phi_error - phi_error/2, encoder_count_L - phi_error/2 ); 
	wheel_state_R = DEAD_STOP;
	wheel_state_L = DEAD_STOP;
	return 0;
}
float calculate_instant_target_speed_R_full(float x, float distance, float ramping__acceleration, float breaking__acceleration)
{
	if (wheel_state_R == ACCELERATING){
		if (direction_R == 1)
		{
			return MAX(MIN(ramping__acceleration * x, cruise_speed_R), 50);
		}
		else
		{
			return MIN(MAX(-ramping__acceleration * x, cruise_speed_R), -50);
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
			return MAX(breaking__acceleration * (x - distance), 0);
		}
		else
		{
			return MIN(-breaking__acceleration * (x - distance), 0);
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
			return MAX(MIN(ramping__acceleration * x, cruise_speed_L), 50);
		}
		else
		{
			return MIN(MAX(-ramping__acceleration * x, cruise_speed_L), -50);
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
	return calculate_instant_target_speed_R_full(x, distance, ramping_acceleration, breaking_acceleration);
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
	while(!is_idle()){
		custom_delay(10);
		Serial.println("Waiting for idle before rotating");
	};
	custom_delay(200);
	reset_speed_control_variables();
	float rotating_breaking_acceleration = -20;
	angle = is_radian ? angle : deg_to_rad(angle);
	float arc_length = (wheels_spacing / 2) * angle;

	direction_R = SGN(angle);
	direction_L = -direction_R;

	// all of these formulas are derived from a ramp graph: speed = f(distance)
	float speed = rotating_speed;
	float distance = fabs(arc_length);
	float x1 = speed / ramping_acceleration;
	float x2 = speed / rotating_breaking_acceleration + distance;

	if (x2 <= x1)
	{ // correcting the cruising speed if it is too high to reach using the current ramp and break
		float intersection_x = (rotating_breaking_acceleration * distance) / (rotating_breaking_acceleration - ramping_acceleration);
		float intersection_speed = ramping_acceleration * intersection_x;
		speed = 0.9 * intersection_speed;
		x1 = speed / ramping_acceleration;
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

		distance_traveled_R = fabs(encoder_count_R - starting_encoder_count_R) * wheel_diameter_R * PI / CPR;
		distance_traveled_L = fabs(encoder_count_L - starting_encoder_count_L) * wheel_diameter_L * PI / CPR;

		yaw_centering_term = MIN((distance_traveled_R - distance_traveled_L) * 0.05, 1); // the coefficient is tunable
		yaw_centering_term = MAX(yaw_centering_term, -1);
		if (x < 0.1 * x1){
			yaw_centering_term = 0;
		}
		speed = calculate_instant_target_speed_R_full(x, distance, ramping_acceleration, rotating_breaking_acceleration);
		speed = fabs(speed) * direction_R;
		set_target_speeds(speed * (1 - yaw_centering_term), -speed * (1 + yaw_centering_term));
		custom_delay(4);
		previous_distance_to_target = distance_to_target;
		distance_to_target = (wheels_spacing / 2) * fabs(target_phi - absolute_phi);
		distance_to_target = MIN(distance_to_target, distance);

		//send_values_to_usart(current_speed_L, current_speed_R, target_speed_L, target_speed_R);
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

		set_target_speeds(speed * (1 - yaw_centering_term), -speed * (1 + yaw_centering_term));
		speed = MAX(fabs(speed) * 0.985, 100) * direction_R;
		previous_distance_to_target = distance_to_target;
		custom_delay(2);
		distance_to_target = (wheels_spacing/2) * fabs(target_phi - absolute_phi);
		distance_to_target = MIN(distance_to_target, distance);

		if (micros() - phase_two_start_time > 3000000)
		{
			break;
		}
		//send_values_to_usart(current_speed_L, current_speed_R, target_speed_L, target_speed_R);
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
    send_values_to_usart(target_speed_R, target_speed_L, current_speed_R, current_speed_R );
	return result;
}
int go_to(float x, float y, float speed)
{
	if (orientate(get_angle_to(x, y), 1)){
		return 1;
	}
	int result = move_forward(get_distance_to(x, y), speed) * 2; // why *2 ? cause fuck you
	return result;
}
void emergency_break()
{
	wheel_state_R = EMERGENCY_BREAK;
	wheel_state_L = EMERGENCY_BREAK;
	stop_motors();
	custom_delay(500);
	set_target_encoder_counts(encoder_count_R, encoder_count_L);
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
	int encoder_tolerance = 18;
	int pwm_tolerance = 200;
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
	while (get_ultrasonic_distance() < min_distance) {
		custom_delay(100); // wait for 100 ms before checking again
	}
}