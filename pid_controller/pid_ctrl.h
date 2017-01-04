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
 *
 * Change history:
 * - 2017-01-04, Frank Duerr (frank.duerr@ipvs.uni-stuttgart.de):
 *   Initial implementation.
 */

#ifndef PID_CTRL_H
#define PID_CTRL_H

/**
 * Data of a PID controller.
 */
struct pid_ctrl {
     // Factors for P, I, D parts.
     float kp;
     float ki;
     float kd;

     float setpoint;

     // Integrated error for I part.
     float e_i;
     
     // Previous error for calculating D part.
     float e_previous;

     // Maximum output value can be clamped to [-out_clamp,+out_clamp].
     float out_clamp;

     // Integral part can be clamped to [-i_clamp,+i_clamp] to avoit integral
     // wind-up.
     float i_clamp;
};

/**
 * PID controller logic.
 *
 * @param pidctrl the pid controller.
 * @param y current state of plant.
 * @return output to actuator.
 */
float pid_ctrl_control(struct pid_ctrl *pidctrl, float y);

#endif
