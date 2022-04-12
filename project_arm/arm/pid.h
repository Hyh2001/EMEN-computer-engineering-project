#ifndef PID_H
#define PID_H


class PID_Controller{
private:
    float KI;
    float KP;
    float KD;
    float error_now;
    float error_pre;
    float integral;
    float integral_max;
    float output_max;

public:
    void set_error(float error){
        error_now = error;
    }
    void set_error_pre(float error){
        error_pre = error;
    }
    float get_error_now(){
        return error_now;
    }
    float get_error_pre(){
        return error_pre;
    }
    float get_KD(){
        return KD;
    }
    float get_KP(){
        return KP;
    }
    float get_max(){
        return output_max;
    }
    float start_integral(){
        integral += KI * error_now;
        integral = integral > integral_max ? integral_max : integral;
        integral = integral < -integral ? -integral_max : integral;
    }

};

float PID_cal(PID_Controller* pid_x, int Now_value, int Aim_value, char flag = 0){  // flag means using integral part
  float error = Now_value - Aim_value;
  pid_x->set_error(error);

  float output = pid_x->get_KD() * (pid_x->get_error_now() - pid_x->get_error_pre())+
  pid_x->get_KP()*pid_x->get_error_now();
  if(flag == 1){
    float integral = pid_x->start_integral();
    output = output + integral;
  }
  output = output > pid_x-> get_max() ? pid_x-> get_max() : output;
  output = output > -pid_x-> get_max() ? -pid_x-> get_max() : output;
  pid_x->set_error_pre(error);
  return output;
}

int* speed_control(){
    int base_output = PID_cal(pid_base,angle_now[base_pin],angle_recorded[base_pin]);
    int catch_output = PID_cal(pid_catch,angle_now[catch_pin],angle_recorded[catch_pin]);
    int big_arm_output = PID_cal(pid_big_arm,angle_now[big_arm_pin],angle_recorded[big_arm_pin]);
    int small_arm_output = PID_cal(pid_small_arm,angle_now[small_arm_pin],angle_recorded[small_arm_pin]);
    int* output = new int[4];
    output[0] = base_output;
    output[1] = catch_output;
    output[2] = big_arm_output;
    output[3] = small_arm_output;
    return output;
}


#endif