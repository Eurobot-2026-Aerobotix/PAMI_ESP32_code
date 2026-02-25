#include "odometry.h"

float current_x = 0;
float current_y = 0;
float current_phi = 0;
float absolute_phi = 0;
int32_t previous_encoder_count_R = 0;
int32_t previous_encoder_count_L = 0;
float phi_offset = 0;

void update_speed(float dt){
    current_speed_L = current_speed_L*0.95 + 0.05*(encoder_count_L - previous_encoder_count_L)*(wheel_diameter_L * PI / CPR * directionL) / dt;
	current_speed_R = current_speed_R*0.95 + 0.05*(encoder_count_R - previous_encoder_count_R)*(wheel_diameter_R * PI / CPR * directionR) / dt;
	speedCalculationPreviousTime = micros();
}

void update_position(float dt){
	
	encoder_count_L = encoder_get_L();
	encoder_count_R = encoder_get_R();

	update_speed(dt);
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
}
void set_position(float current__x, float current__y, float current__phi, float absolute__phi){
	current_x = current__x;
	current_y = current__y;
	current_phi = current__phi;
	absolute_phi = absolute__phi;
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
float get_bezier_distance(float t ,float x0, float y0,float x1, float y1,float x2 , float y2 ,float x3, float y3){
    int N = 150 * sqrt(SQR(x3-x0)+SQR(y3-y0)) / 150.0;
    float length = 0.0f;

    for(float i = (t * N); i <= N; i++){
        float t_prev = (float)(i - 1) / N;
        float t_curr = (float)i / N;

        float mt_prev = 1.0f - t_prev;
        float mt_curr = 1.0f - t_curr;

        float x_prev =  mt_prev*mt_prev*mt_prev * x0
                      + 3*mt_prev*mt_prev*t_prev * x1
                      + 3*mt_prev*t_prev*t_prev * x2
                      + t_prev*t_prev*t_prev * x3;

        float y_prev =  mt_prev*mt_prev*mt_prev * y0
                      + 3*mt_prev*mt_prev*t_prev * y1
                      + 3*mt_prev*t_prev*t_prev * y2
                      + t_prev*t_prev*t_prev * y3;

        float x_curr =  mt_curr*mt_curr*mt_curr * x0
                      + 3*mt_curr*mt_curr*t_curr * x1
                      + 3*mt_curr*t_curr*t_curr * x2
                      + t_curr*t_curr*t_curr * x3;

        float y_curr =  mt_curr*mt_curr*mt_curr * y0
                      + 3*mt_curr*mt_curr*t_curr * y1
                      + 3*mt_curr*t_curr*t_curr * y2
                      + t_curr*t_curr*t_curr * y3;

        length += sqrtf((x_curr - x_prev)*(x_curr - x_prev)
                      + (y_curr - y_prev)*(y_curr - y_prev));
    }
    return length;
}
void bezier(float t, float x0, float y0, float x1, float y1, float x2, float y2, float x3, float y3, float *x, float *y){
    float mt = 1.0f - t;
    float mt2 = mt * mt;
    float t2  = t * t;

    *x = mt2*mt*x0 + 3*mt2*t*x1 + 3*mt*t2*x2 + t2*t*x3;
    *y = mt2*mt*y0 + 3*mt2*t*y1 + 3*mt*t2*y2 + t2*t*y3;
}
void bezier_derivative(float t, float x0, float y0, float x1, float y1, float x2, float y2, float x3, float y3, float *x, float *y){
    float mt = 1.0f - t;

    *x = 3*mt*mt*(x1 - x0)
       + 6*mt*t*(x2 - x1)
       + 3*t*t*(x3 - x2);

    *y = 3*mt*mt*(y1 - y0)
       + 6*mt*t*(y2 - y1)
       + 3*t*t*(y3 - y2);
}
float project_bezier(float px, float py, float x0, float y0, float x1, float y1, float x2, float y2, float x3, float y3){
    float t = 0.5;
	for(int i=0;i<10;i++){
		float bx, by, dx, dy;

		bezier(t, x0,y0,x1,y1,x2,y2,x3,y3, &bx,&by);
		bezier_derivative(t, x0,y0,x1,y1,x2,y2,x3,y3, &dx,&dy);

		float rx = bx - px;
		float ry = by - py;

		float denom = dx*dx + dy*dy;
		if(denom < 1e-6f) break;

		float step = (rx*dx + ry*dy) / denom;

		t -= step;

		if(t < 0.0f) t = 0.0f;
		if(t > 1.0f) t = 1.0f;
	}
	return t;
}
float get_bezier_radius(float t,float x0, float y0,float x1, float y1, float x2, float y2,float x3, float y3){
	float dx1, dy1, dx2, dy2;
    bezier_derivative(t, x0, y0, x1, y1, x2, y2, x3, y3, &dx1, &dy1);

    float mt = 1.0f - t;
    dx2 = 6*mt*(x2 - 2*x1 + x0) + 6*t*(x3 - 2*x2 + x1);
    dy2 = 6*mt*(y2 - 2*y1 + y0) + 6*t*(y3 - 2*y2 + y1);

    float speed2 = dx1*dx1 + dy1*dy1;
    float cross  = dx1*dy2 - dy1*dx2;

    if(fabsf(cross) < 1e-6f)
         return 1e6f;

    float R = powf(speed2, 1.5f) / fabsf(cross);

    return (cross > 0.0f) ? R : -R;
}
double signed_distance_to_bezier(double px, double py, double x0, double y0, double x1, double y1, double x2, double y2, double x3, double y3, double t){
    double mt = 1.0 - t;

    double cx =
        (mt*mt*mt)*x0 +
        3.0*(mt*mt)*t*x1 +
        3.0*mt*(t*t)*x2 +
        (t*t*t)*x3;

    double cy =
        (mt*mt*mt)*y0 +
        3.0*(mt*mt)*t*y1 +
        3.0*mt*(t*t)*y2 +
        (t*t*t)*y3;

    double tx =
        3.0*mt*mt*(x1 - x0) +
        6.0*mt*t*(x2 - x1) +
        3.0*t*t*(x3 - x2);

    double ty =
        3.0*mt*mt*(y1 - y0) +
        6.0*mt*t*(y2 - y1) +
        3.0*t*t*(y3 - y2);

    double vx = px - cx;
    double vy = py - cy;

    double dist = sqrt(vx*vx + vy*vy);
    double cross = tx*vy - ty*vx;

    return (cross < 0.0) ? dist : -dist;
}