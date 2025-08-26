/**********************************************
 * Self-Driving Car Nano-degree - Udacity
 *  Created on: December 11, 2020
 *      Author: Mathilde Badoual
 **********************************************/

#include "pid_controller.h"
#include <vector>
#include <iostream>
#include <math.h>

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kpi, double Kii, double Kdi, double output_lim_maxi, double output_lim_mini) {
   /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   **/
  coef_P = Kpi;
  coef_I = Kii;
  coef_D = Kdi;
  output_lim_max = output_lim_maxi;
  output_lim_min = output_lim_mini;
  prev_cte = 0;
  sum_cte = 0;
}


void PID::UpdateError(double cte) {
   /**
   * TODO: Update PID errors based on cte.
   **/
  err_P = -coef_P * cte;

  err_D = delta_time == 0.0 ? 0.0 : -coef_D * (cte - prev_cte) / delta_time;
  prev_cte = cte;
  
  sum_cte += cte;
  err_I = -coef_I * sum_cte;  
}

double PID::TotalError() {
   /**
   * TODO: Calculate and return the total error
    * The code should return a value in the interval [output_lim_mini, output_lim_maxi]
   */
    double control = err_P + err_I + err_D;
    return min(max(control, output_lim_min), output_lim_max);
    // if (control < output_lim_min) return output_lim_min;
    // else if (control > output_lim_max) return output_lim_max;
    // else return control;
}

double PID::UpdateDeltaTime(double new_delta_time) {
   /**
   * TODO: Update the delta time with new value
   */
  delta_time = new_delta_time;
}