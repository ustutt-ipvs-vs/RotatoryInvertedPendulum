/**
 * This file is part of Rotatory Inverted Pendulum.
 *
 * Copyright 2016, 2017 Institute of Parallel and Distributed Systems (IPVS).
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 * http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "pid_ctrl.h"

float pid_ctrl_control(struct pid_ctrl *pidctrl, float y)
{
     // Current deviation from setpoint (error).
     float e = y - pidctrl->setpoint;

     pidctrl->e_i += e;
     // Clamp integral part to avoid integral wind-up.
     if (pidctrl->e_i > pidctrl->i_clamp) {
	  pidctrl->e_i = pidctrl->i_clamp;
     } else if (pidctrl->e_i < -pidctrl->i_clamp) {
	  pidctrl->e_i = -pidctrl->i_clamp;
     }

     float e_diff = e - pidctrl->e_previous;
     pidctrl->e_previous = e;
     
     // Output of PID controller.
     float output = pidctrl->kp*e + pidctrl->ki*pidctrl->e_i + pidctrl->kd*e_diff;

     // Clamp output.
     if (output > pidctrl->out_clamp) {
	  output = pidctrl->out_clamp;
     } else if (output < -pidctrl->out_clamp) {
	  output = -pidctrl->out_clamp;
     }

     return output;
}
