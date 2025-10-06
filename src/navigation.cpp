#include "navigation.h"



enum Wheel_state wheel_state_R = DEAD_STOP;
enum Wheel_state wheel_state_L = DEAD_STOP;
enum Trajectory trajectory = STRAIGHT;
float ramping_acceleration = 5;
float breaking_acceleration = -2;
float emergency_breaking_acceleration = -10;
float rotating_speed = 200;
int8_t direction_R;
int8_t direction_L;
float cruise_speed_R;
float cruise_speed_L;
float starting_phi;
float turning_radius;

int move_forward(float distance, float speed){
	direction_R = 1;
	direction_L = 1;

	// all of these formulas are derived from a ramp graph: speed = f(distance)
	float x1 = speed/ramping_acceleration;
	float x2 = speed/breaking_acceleration + distance;

	if (x2 <= x1){ // correcting the cruising speed if it is too high to reach using the current ramp and break
		float intersection_x = (breaking_acceleration*distance)/(breaking_acceleration - ramping_acceleration);
		float intersection_speed = ramping_acceleration * intersection_x;
		speed = 0.9 * intersection_speed;
		x1 = speed/ramping_acceleration;
		x2 = speed/breaking_acceleration + distance;
	}

	cruise_speed_R = speed;
	cruise_speed_L = speed;
	float target_point_x = current_x + distance*cos(current_phi);
	float target_point_y = current_y + distance*sin(current_phi);
	float distance_to_target = distance;
	float previous_distance_to_target = distance;
	float x = 0;

	wheel_state_R = ACCELERATING;
	wheel_state_L = ACCELERATING;
	trajectory = STRAIGHT;
	starting_phi = absolute_phi;
	float lane_correction_term;
	float phase_two_distance = 30;
	while (distance_to_target > phase_two_distance){
		x = distance - distance_to_target;
		if (wheel_state_R == ACCELERATING && x > x1 ){
			wheel_state_R = CRUISING;
			wheel_state_L = CRUISING;
		}else if (wheel_state_R == CRUISING && x > x2){
			wheel_state_R = CONTROLLED_BREAK;
			wheel_state_L = CONTROLLED_BREAK;
		}else if (wheel_state_R == EMERGENCY_BREAK){
			return 0;
		}

		lane_correction_term = (calculate_instant_target_phi(x, distance) - absolute_phi) * 10; // the coefficient is tunable
		if (x < 0.1*x1){
			lane_correction_term = 0;
		}
		set_target_speeds(calculate_instant_target_speed_R(x, distance)*(1 + lane_correction_term) ,calculate_instant_target_speed_L(x, distance)* (1 - lane_correction_term));
		vTaskDelay(4);
		previous_distance_to_target = distance_to_target;
		distance_to_target = get_distance_to(target_point_x, target_point_y);

//		if (distance_to_target >= previous_distance_to_target + 2){
//			stop_motors();
//			return 1;
//		}

		send_values_to_usart(current_speed_L, current_speed_R, target_speed_L, target_speed_R);
	}
	// completing the last part of distance separately, so the break happens on point
		// calculating the distance to move that  results in the robot being the closest to target point, i can't explain the math in code unfortunately
	float x_closest = ( target_point_y - current_y + tan(current_phi)*current_x - tan(current_phi - PI/2)*target_point_x ) / ( tan(current_phi) - tan(current_phi - PI/2) );
	float y_closest = tan(current_phi)*(x_closest - current_x) + current_y;
	speed = (target_speed_R + target_speed_L)/ 2;
	if (fabs(speed) < phase_two_distance*(-breaking_acceleration) ){ // handling the case where the robot skips the first while loop, being too close to target_distance
		speed = - phase_two_distance * breaking_acceleration;
	}
	distance_to_target = get_distance_to(x_closest, y_closest);
	previous_distance_to_target = distance_to_target;
	int32_t phase_two_start_time = micros();
	while (distance_to_target >0.1 && distance_to_target <= previous_distance_to_target){
		lane_correction_term = (calculate_instant_target_phi(x, distance) - absolute_phi) * 20 ; // the coefficient is tunable
		set_target_speeds(speed*(1 + lane_correction_term) ,speed* (1 - lane_correction_term));
		speed = MAX(speed*0.995, 0);
		previous_distance_to_target = distance_to_target;
		vTaskDelay(2);
		distance_to_target = get_distance_to(x_closest, y_closest);

		if(micros() - phase_two_start_time> 4000000){
			break;
		}
//		if (distance_to_target >= previous_distance_to_target + 2){
//			stop_motors();
//			return 1;
//		}

		send_values_to_usart(current_speed_L, current_speed_R, target_speed_L, target_speed_R);
	}
	stop_motors();
	wheel_state_R = DEAD_STOP;
	wheel_state_L = DEAD_STOP;
	return 0;
}
int move_backward(float distance, float speed){
	speed = (speed < 0)? -speed: speed;
	distance = (distance < 0)? -distance: distance;
	direction_R = -1;
	direction_L = -1;

	// all of these formulas are derived from a ramp graph: speed = f(distance)
	float x1 = speed/ramping_acceleration;
	float x2 = speed/breaking_acceleration + distance;

	if (x2 <= x1){ // correcting the cruising speed if it is too high to reach using the current ramp and break
		float intersection_x = (breaking_acceleration*distance)/(breaking_acceleration - ramping_acceleration);
		float intersection_speed = ramping_acceleration * intersection_x;
		speed = 0.9 * intersection_speed;
		x1 = speed/ramping_acceleration;
		x2 = speed/breaking_acceleration + distance;
	}

	cruise_speed_R = -speed;
	cruise_speed_L = -speed;
	float target_point_x = current_x - distance*cos(current_phi);
	float target_point_y = current_y - distance*sin(current_phi);
	float distance_to_target = distance;
	float previous_distance_to_target = distance;
	float x = 0;

	wheel_state_R = ACCELERATING;
	wheel_state_L = ACCELERATING;
	trajectory = STRAIGHT;
	starting_phi = absolute_phi;
	float lane_correction_term;
	float phase_two_distance = 20;
	while (distance_to_target > phase_two_distance){

		x = distance - distance_to_target;
		if (wheel_state_R == ACCELERATING && x > x1 ){
			wheel_state_R = CRUISING;
			wheel_state_L = CRUISING;
		}else if (wheel_state_R == CRUISING && x > x2){
			wheel_state_R = CONTROLLED_BREAK;
			wheel_state_L = CONTROLLED_BREAK;
		}else if (wheel_state_R == EMERGENCY_BREAK){
			return 0;
		}

		lane_correction_term = (calculate_instant_target_phi(x, distance) - absolute_phi) * 10; // the coefficient is tunable
		if (x < 0.1*x1){
			lane_correction_term = 0;
		}
		set_target_speeds(calculate_instant_target_speed_R(x, distance)*(1 - lane_correction_term) ,calculate_instant_target_speed_L(x, distance)* (1 + lane_correction_term));
		vTaskDelay(4);
		previous_distance_to_target = distance_to_target;
		distance_to_target = get_distance_to(target_point_x, target_point_y);
//		if (distance_to_target >= previous_distance_to_target + 2){
//			stop_motors();
//			return 1;
//		}

		send_values_to_usart(current_speed_L, current_speed_R, target_speed_L, target_speed_R);
	}
	// completing the last part of distance separately, so the break happens on point
		// calculating the distance to move that results in the robot being the closest to target point, i can't explain the math in code unfortunately
	float x_closest = ( target_point_y - current_y + tan(current_phi)*current_x - tan(current_phi - PI/2)*target_point_x ) / ( tan(current_phi) - tan(current_phi - PI/2) );
	float y_closest = tan(current_phi)*(x_closest - current_x) + current_y;
	speed = (target_speed_R + target_speed_L)/ 2;
	if (fabs(speed) < phase_two_distance*(-breaking_acceleration) ){ // handling the case where the robot skips the first while loop, being too close to target_distance
		speed = - phase_two_distance * breaking_acceleration;
	}
	distance_to_target = get_distance_to(x_closest, y_closest);
	previous_distance_to_target = distance_to_target;

	int32_t phase_two_start_time = micros();
	while (distance_to_target >0.1 && distance_to_target <= previous_distance_to_target){
		lane_correction_term = (calculate_instant_target_phi(x, distance) - absolute_phi) * 20 ; // the coefficient is tunable
		set_target_speeds(speed*(1 - lane_correction_term) ,speed* (1 + lane_correction_term));
		speed = MIN(speed*0.995, 0);
		previous_distance_to_target = distance_to_target;
		vTaskDelay(2);
		distance_to_target = get_distance_to(x_closest, y_closest);

		if(micros() - phase_two_start_time> 4000000){
			break;
		}

//		if (distance_to_target >= previous_distance_to_target + 2){
//			stop_motors();
//			return 1;
//		}

		send_values_to_usart(current_speed_L, current_speed_R, target_speed_L, target_speed_R);
	}
	stop_motors();
	wheel_state_R = DEAD_STOP;
	wheel_state_L = DEAD_STOP;
	return 0;
}
float calculate_instant_target_speed_R_full(float x, float distance, float ramping__acceleration, float breaking__acceleration){
	if (wheel_state_R == ACCELERATING){
		x = (x < 10)? 100/ramping__acceleration : x;
		if (direction_R == 1){
			return MIN(ramping__acceleration*x, cruise_speed_R);
		}else{
			return MAX(-ramping__acceleration*x, cruise_speed_R);
		}

	}else if (wheel_state_R == CRUISING){
		return cruise_speed_R;

	}else if (wheel_state_R == CONTROLLED_BREAK){
		if (direction_R == 1){
			return MAX(breaking__acceleration*(x-distance), 0);
		}else{
			return MIN(-breaking__acceleration*(x-distance), 0);
		}

	}else if (wheel_state_R == EMERGENCY_BREAK){
		if (direction_R == 1){
			return MAX(target_speed_R + emergency_breaking_acceleration*x, 0);
		}else{
			return MIN(target_speed_R - emergency_breaking_acceleration*x, 0);
		}
	}
	return 0; // just to prevent debugger error, will be erased once a definitive else is added
}
float calculate_instant_target_speed_L_full(float x, float distance, float ramping__acceleration, float breaking__acceleration){
	if (wheel_state_L == ACCELERATING){
		x = (x < 10)? 100/ramping__acceleration : x;
		if (direction_L == 1){
			return MIN(ramping__acceleration*x, cruise_speed_L);
		}else{
			return MAX(-ramping__acceleration*x, cruise_speed_L);
		}

	}else if (wheel_state_L == CRUISING){
		return cruise_speed_L;

	}else if (wheel_state_L == CONTROLLED_BREAK){
		if (direction_L == 1){
			return MAX(breaking__acceleration*(x-distance), 0);
		}else{
			return MIN(-breaking__acceleration*(x-distance), 0);
		}

	}else if (wheel_state_L == EMERGENCY_BREAK){
		if (direction_L == 1){
			return MAX(target_speed_L + emergency_breaking_acceleration*x, 0);
		}else{
			return MIN(target_speed_L - emergency_breaking_acceleration*x, 0);
		}
	}
	return 0; // just to prevent debugger error, will be erased once a definitive else is added
}
float calculate_instant_target_speed_R(float x, float distance){
	return calculate_instant_target_speed_R_full( x, distance, ramping_acceleration, breaking_acceleration);
}
float calculate_instant_target_speed_L(float x, float distance){
	return calculate_instant_target_speed_R_full( x, distance, ramping_acceleration, breaking_acceleration);
}
float calculate_instant_target_phi(float x, float distance){
	if (trajectory == STRAIGHT){
		return starting_phi;
	}else if ( trajectory == CIRCLE_CURVE ){
		return starting_phi - x/turning_radius;
	}
	return 0;
}
int rotate(float angle, int8_t is_radian){
	float rotating_breaking_acceleration = -3.7;
	angle = is_radian? angle: deg_to_rad(angle);
	float arc_length = (wheels_spacing/2) * angle;

	direction_R = SGN(arc_length);
	direction_L = -direction_R;

	// all of these formulas are derived from a ramp graph: speed = f(distance)
	float speed = rotating_speed;
	float distance = fabs(arc_length);
	float x1 = speed/ramping_acceleration;
	float x2 = speed/rotating_breaking_acceleration + distance;

	if (x2 <= x1){ // correcting the cruising speed if it is too high to reach using the current ramp and break
		float intersection_x = (rotating_breaking_acceleration*distance)/(rotating_breaking_acceleration - ramping_acceleration);
		float intersection_speed = ramping_acceleration * intersection_x;
		speed = 0.9 * intersection_speed;
		x1 = speed/ramping_acceleration;
		x2 = speed/rotating_breaking_acceleration + distance;
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
	while ( fabs(target_phi - absolute_phi) > deg_to_rad(phase_two_angle)){
		x = distance - distance_to_target;
		if (wheel_state_R == ACCELERATING && x > x1 ){
			wheel_state_R = CRUISING;
			wheel_state_L = CRUISING;
		}else if (wheel_state_R == CRUISING && x > x2){
			wheel_state_R = CONTROLLED_BREAK;
			wheel_state_L = CONTROLLED_BREAK;
		}else if (wheel_state_R == EMERGENCY_BREAK){
			return 0;
		}

		distance_traveled_R = fabs(encoder_count_R - starting_encoder_count_R) * wheel_diameter_R * PI / CPR;
		distance_traveled_L = fabs(encoder_count_L - starting_encoder_count_L) * wheel_diameter_L * PI / CPR;

		yaw_centering_term = ( distance_traveled_R - distance_traveled_L ) * 0.02; // the coefficient is tunable
		if (x < 0.1*x1){
			yaw_centering_term = 0;
		}
		speed = calculate_instant_target_speed_R_full(x, distance, ramping_acceleration, rotating_breaking_acceleration);
		set_target_speeds(speed*(1 - yaw_centering_term) ,-speed* (1 + yaw_centering_term));
		vTaskDelay(4);
		previous_distance_to_target = distance_to_target;
		distance_to_target = (wheels_spacing/2) * fabs(target_phi - absolute_phi);

//		if (distance_to_target >= previous_distance_to_target + 2){
//			stop_motors();
//			return 1;
//		}

		send_values_to_usart(current_speed_L, current_speed_R, target_speed_L, target_speed_R);
	}
	// completing the last part of distance separately, so the break happens on point

	speed = (target_speed_R - target_speed_L)/ 2; // one of the speeds is negative, that's why there is a minus
	if (fabs(speed) < (deg_to_rad(phase_two_angle) * wheels_spacing/2)*(-rotating_breaking_acceleration) ){ // handling the case where the robot skips the first while loop, being too close to target_phi
		speed = (deg_to_rad(phase_two_angle) * wheels_spacing/2)*(-rotating_breaking_acceleration) * direction_R;
	}
	distance_to_target = wheels_spacing * fabs(target_phi - absolute_phi);
	previous_distance_to_target = distance_to_target;

	uint32_t phase_two_start_time = micros();
	while (fabs(target_phi - absolute_phi) > deg_to_rad(0.2) && distance_to_target <= previous_distance_to_target + 10){
		distance_traveled_R = (encoder_count_R - starting_encoder_count_R) * wheel_diameter_R * PI / CPR;
		distance_traveled_L = (encoder_count_L - starting_encoder_count_L) * wheel_diameter_L * PI / CPR;
		yaw_centering_term = ( fabs(distance_traveled_R) - fabs(distance_traveled_L) ) * 0.05;

		set_target_speeds(speed*(1 - yaw_centering_term) ,-speed* (1 + yaw_centering_term));
		speed = MAX(fabs(speed)*0.985, 0) * direction_R;
		previous_distance_to_target = distance_to_target;
		vTaskDelay(2);
		distance_to_target = wheels_spacing * fabs(target_phi - absolute_phi);

		if(micros() - phase_two_start_time> 3000000){
			break;
		}

//		if (distance_to_target >= previous_distance_to_target+2){
//			stop_motors();
//			return 1;
//		}

		send_values_to_usart(current_speed_L, current_speed_R, target_speed_L, target_speed_R);
	}
	stop_motors();
	wheel_state_R = DEAD_STOP;
	wheel_state_L = DEAD_STOP;
	return 0;
}
int move_forward_with_deviation(float distance, float deviation, float speed){
	// there is a lotta math going on here, can't explain it in code
	float radius = ( SQR(distance/2) + SQR(deviation) ) / ( 2*deviation );
	float alpha = atan( (distance/2) / ( fabs(radius) - fabs(deviation) ) ) * SGN(deviation);
	float arc_length = 2 * fabs(alpha * radius);
	float arc_length_R = fabs( 2 * alpha * ( radius - wheels_spacing/2) );
	float arc_length_L = fabs( 2 * alpha * ( radius + wheels_spacing/2) );

	float target_point_x = current_x + distance*cos(current_phi);
	float target_point_y = current_y + distance*sin(current_phi);

	starting_phi = absolute_phi + alpha;
	turning_radius = radius;

	float center_point_x = current_x + radius*cos(current_phi - PI/2 + alpha);
	float center_point_y = current_y + radius*sin(current_phi - PI/2 + alpha);

	rotate(alpha, 1);
	vTaskDelay(1000);

	direction_R = 1;
	direction_L = 1;
	cruise_speed_R = speed * ( arc_length_R/arc_length );
	cruise_speed_L = speed * ( arc_length_L/arc_length );

	float x1 = speed/4; // 4 is the ramping acceleration of this specific function
	float x2 = speed/breaking_acceleration + arc_length;

	while (x2 <= x1){ // correcting the cruising speed if it is too high to reach using the current ramp and break
		float intersection_x = (breaking_acceleration*distance)/(breaking_acceleration - ramping_acceleration);
		float intersection_speed = ramping_acceleration * intersection_x;
		speed = 0.9 * intersection_speed;
		x1 = speed/ramping_acceleration;
		x2 = speed/breaking_acceleration + distance;
		cruise_speed_R = speed * ( arc_length_R/arc_length );
		cruise_speed_L = speed * ( arc_length_L/arc_length );
	}

	float ramping_acceleration_R = ramping_acceleration * ( arc_length_R/arc_length );
	float ramping_acceleration_L = ramping_acceleration * ( arc_length_L/arc_length );
	float breaking_acceleration_R = breaking_acceleration * ( arc_length_R/arc_length );
	float breaking_acceleration_L = breaking_acceleration * ( arc_length_L/arc_length );

	wheel_state_R = ACCELERATING;
	wheel_state_L = ACCELERATING;
	trajectory = CIRCLE_CURVE;
	float lane_correction_term;
	float phase_two_distance = 30;
	float x = 0;
	float distance_to_target = arc_length;
	float previous_distance_to_target = distance_to_target;
	while (distance_to_target > phase_two_distance){
		x = arc_length - distance_to_target;
		if (wheel_state_R == ACCELERATING && x > x1 ){
			wheel_state_R = CRUISING;
			wheel_state_L = CRUISING;
		}else if (wheel_state_R == CRUISING && x > x2){
			wheel_state_R = CONTROLLED_BREAK;
			wheel_state_L = CONTROLLED_BREAK;
		}else if (wheel_state_R == EMERGENCY_BREAK){
			return 0;
		}

		lane_correction_term = (calculate_instant_target_phi(x, arc_length) - absolute_phi) * 20; // the coefficient is tunable
		if (x < 0.1*x1){
			lane_correction_term = 0;
		}
		set_target_speeds(calculate_instant_target_speed_R_full(x, arc_length, ramping_acceleration_R, breaking_acceleration_R)*(1 + lane_correction_term) ,calculate_instant_target_speed_L_full(x, arc_length, ramping_acceleration_L, breaking_acceleration_L)* (1 - lane_correction_term));
		vTaskDelay(4);
		previous_distance_to_target = distance_to_target;

		distance_to_target = get_arc_distance_to(target_point_x, target_point_y, center_point_x, center_point_y, radius);

//		if (distance_to_target >= previous_distance_to_target + 2){
//			stop_motors();
//			return 1;
//		}

		send_values_to_usart(current_speed_L, current_speed_R, target_speed_L, target_speed_R);
	}
	// completing the last part of distance separately, so the break happens on point
	distance_to_target = get_arc_distance_to(target_point_x, target_point_y, center_point_x, center_point_y, radius);
	previous_distance_to_target = distance_to_target;
	float speed_R = calculate_instant_target_speed_R_full(x, arc_length, ramping_acceleration_R, breaking_acceleration_R);
	float speed_L = calculate_instant_target_speed_L_full(x, arc_length, ramping_acceleration_L, breaking_acceleration_L);
	int32_t phase_two_start_time = micros();
	while (distance_to_target >1 && distance_to_target <= previous_distance_to_target){
		lane_correction_term = (calculate_instant_target_phi(x, distance) - absolute_phi) * 50 ; // the coefficient is tunable
		set_target_speeds(speed_R*(1 + lane_correction_term) ,speed_L* (1 - lane_correction_term));
		speed_R = MAX(speed_R*0.995, 0);
		speed_L = MAX(speed_L*0.995, 0);
		previous_distance_to_target = distance_to_target;
		vTaskDelay(2);
		distance_to_target = get_arc_distance_to(target_point_x, target_point_y, center_point_x, center_point_y, radius);

		if(micros() - phase_two_start_time> 4000000){
			break;
		}
//		if (distance_to_target >= previous_distance_to_target + 2){
//			stop_motors();
//			return 1;
//		}

		send_values_to_usart(current_speed_L, current_speed_R, target_speed_L, target_speed_R);
	}
	stop_motors();
	wheel_state_R = DEAD_STOP;
	wheel_state_L = DEAD_STOP;
	return 0;
}
int orientate(float orientation, int8_t is_radian){
	orientation = (is_radian)? orientation : deg_to_rad(orientation);
	if (abs(orientation - current_phi) > deg_to_rad(0.5) ){
		float angle = orientation - current_phi;
		if (angle > PI){
			angle -= 2*PI;
		}else if (angle < -PI){
			angle += 2*PI;
		}
		return rotate(angle, 1);
	}
	return 0;
}
int circle_curve(float target_x, float target_y, float target_phi, float speed){
	// there is a lotta math going on here, can't explain it in code
	float trajectory_phi = atan( (target_y - current_y) / (target_x - current_x) ); // -30
	float alpha = trajectory_phi - target_phi;
	float distance = get_distance_to(target_x, target_y);
	float radius = distance / ( 2*sin(alpha) );
	float arc_length = 2 * fabs( alpha * radius);
	float arc_length_R = 2 * fabs( alpha * ( radius - wheels_spacing/2) );
	float arc_length_L = 2 * fabs( alpha * ( radius + wheels_spacing/2) );

	starting_phi = 2*trajectory_phi - target_phi + (absolute_phi - current_phi); // the part between paratheses is to offset the phi to make it in the prespective of absolute phi not current_phi

	// !!!!!!!!!!!!!! el mochkla men starting phi, 9a3da tabda fi blasa bi 7aythou l get instant phi mahich s7i7a
	turning_radius = radius;

	float center_point_x = target_x + radius*cos(target_phi - PI/2 );
	float center_point_y = target_y + radius*sin(target_phi - PI/2 );

	orientate( 2*trajectory_phi - target_phi , 1);
	vTaskDelay(1000);

	direction_R = 1;
	direction_L = 1;
	cruise_speed_R = speed * ( arc_length_R/arc_length );
	cruise_speed_L = speed * ( arc_length_L/arc_length );

	float x1 = speed/4; // 4 is the ramping acceleration of this specific function
	float x2 = speed/breaking_acceleration + arc_length;

	while (x2 <= x1){ // correcting the cruising speed if it is too high to reach using the current ramp and break
		float intersection_x = (breaking_acceleration*distance)/(breaking_acceleration - ramping_acceleration);
		float intersection_speed = ramping_acceleration * intersection_x;
		speed = 0.9 * intersection_speed;
		x1 = speed/ramping_acceleration;
		x2 = speed/breaking_acceleration + distance;
		cruise_speed_R = speed * ( arc_length_R/arc_length );
		cruise_speed_L = speed * ( arc_length_L/arc_length );
	}

	float ramping_acceleration_R = ramping_acceleration * ( arc_length_R/arc_length );
	float ramping_acceleration_L = ramping_acceleration * ( arc_length_L/arc_length );
	float breaking_acceleration_R = breaking_acceleration * ( arc_length_R/arc_length );
	float breaking_acceleration_L = breaking_acceleration * ( arc_length_L/arc_length );

	wheel_state_R = ACCELERATING;
	wheel_state_L = ACCELERATING;
	trajectory = CIRCLE_CURVE;
	float lane_correction_term;
	float phase_two_distance = 20;
	float x = 0;
	float distance_to_target = arc_length;
	float previous_distance_to_target = distance_to_target;
	while (distance_to_target > phase_two_distance){
		x = arc_length - distance_to_target;
		if (wheel_state_R == ACCELERATING && x > x1 ){
			wheel_state_R = CRUISING;
			wheel_state_L = CRUISING;
		}else if (wheel_state_R == CRUISING && x > x2){
			wheel_state_R = CONTROLLED_BREAK;
			wheel_state_L = CONTROLLED_BREAK;
		}else if (wheel_state_R == EMERGENCY_BREAK){
			return 0;
		}

		lane_correction_term = (calculate_instant_target_phi(x, arc_length) - absolute_phi) * 20; // the coefficient is tunable
		if (x < 0.1*x1){
			lane_correction_term = 0;
		}
		set_target_speeds(calculate_instant_target_speed_R_full(x, arc_length, ramping_acceleration_R, breaking_acceleration_R)*(1 + lane_correction_term) ,calculate_instant_target_speed_L_full(x, arc_length, ramping_acceleration_L, breaking_acceleration_L)* (1 - lane_correction_term));
		vTaskDelay(4);
		previous_distance_to_target = distance_to_target;

		distance_to_target = get_arc_distance_to(target_x, target_y, center_point_x, center_point_y, radius);

//		if (distance_to_target >= previous_distance_to_target + 2){
//			stop_motors();
//			return 1;
//		}

		send_values_to_usart(current_speed_L, current_speed_R, target_speed_L, target_speed_R);
	}
	// completing the last part of distance separately, so the break happens on point
	distance_to_target = get_arc_distance_to(target_x, target_y, center_point_x, center_point_y, radius);
	previous_distance_to_target = distance_to_target;
	float speed_R = calculate_instant_target_speed_R_full(x, arc_length, ramping_acceleration_R, breaking_acceleration_R);
	float speed_L = calculate_instant_target_speed_L_full(x, arc_length, ramping_acceleration_L, breaking_acceleration_L);
	int32_t phase_two_start_time = micros();
	while (distance_to_target >1 && distance_to_target <= previous_distance_to_target){
		lane_correction_term = (calculate_instant_target_phi(x, distance) - absolute_phi) * 50 ; // the coefficient is tunable
		set_target_speeds(speed_R*(1 + lane_correction_term) ,speed_L* (1 - lane_correction_term));
		speed_R = MAX(speed_R*0.996, 0);
		speed_L = MAX(speed_L*0.996, 0);
		previous_distance_to_target = distance_to_target;
		vTaskDelay(2);
		distance_to_target = get_arc_distance_to(target_x, target_y, center_point_x, center_point_y, radius);

		if(micros() - phase_two_start_time> 4000000){
			break;
		}
//		if (distance_to_target >= previous_distance_to_target + 2){
//			stop_motors();
//			return 1;
//		}

		send_values_to_usart(current_speed_L, current_speed_R, target_speed_L, target_speed_R);
	}
	stop_motors();
	wheel_state_R = DEAD_STOP;
	wheel_state_L = DEAD_STOP;
	return 0;
}

int go_to(float x, float y, float speed){
	if ( orientate( get_angle_to(x, y), 1 ) ){
		return 1;
	}
	vTaskDelay(1000);
	return move_forward(get_distance_to(x, y), speed) * 2; // why *2 ? view in .h file
}
void emergency_break(){
	wheel_state_R = EMERGENCY_BREAK;
	wheel_state_L = EMERGENCY_BREAK;
	int32_t last_encoder_count_R = encoder_count_R;
	int32_t last_encoder_count_L = encoder_count_L;
	vTaskDelay(4);
	while (current_speed_R > 0.5 || current_speed_L > 0.5){
		float dx_R = ticks_to_distance(fabs(encoder_count_R - last_encoder_count_R), wheel_diameter_R);
		float dx_L = ticks_to_distance(fabs(encoder_count_L - last_encoder_count_L), wheel_diameter_L);

		set_target_speeds(calculate_instant_target_speed_R(dx_R, 0), calculate_instant_target_speed_L(dx_L, 0));

		last_encoder_count_R = encoder_count_R;
		last_encoder_count_L = encoder_count_L;
		vTaskDelay(4);
	}
	stop_motors();
}
