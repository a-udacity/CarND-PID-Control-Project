# Term 2 PID Control Project

---

A PID controller project was implemented in C++ based on the Python source code examples and quizzes from the class.
Minor `PID.h` file was made to add an attribute

Reflection
---

### PID Controller and effect of each component

A proportional–integral–derivative controller (PID controller) is a control loop feedback mechanism widely used in industrial control to calculate an error value E(n) as the difference between a desired set point and a measured process variable and applies a correction based on proportional, integral, and derivative terms. In the context of this project, a vehicle simulator produces the error signal as the distance between actual position of the car on the road and a reference trajectory called cross-track error (cte). The PID controller output is the steering angle such that it minimises the distance to this reference trajectory.

```
U(n) = - Kp * E(n) - Ki * Ei(n) - Kd * Ed(n)
```

where
* `E(n)` - cross-track error which is the error between the currently measured value and setpoint
* `Ei(n)` - sum of the instantaneous error over time till current moment
* `Ed(n)` - time derivative of error
* `Kp, Ki, Kd` - proportional, integral and derivative gains

#### P - proportional

`Kp * E(n)` is the proportional term that produce correction signal that is proportional to the cross-track error. If proportional gain `Kp` is too high, the system can become unstable whereas, a small gain results in a small corrections to a large errors.

#### I - integral

`Ki * Ei(n)` is the integral term that accelerates the movement of the process towards setpoint and eliminates the residual steady-state error that occurs with a pure proportional term. Since the integral term sums up the cross-track error over time, it is useful in high speeds to sum a large error signal quickly. Integral term also mitigates any biases such as if a zero steering angle does not correspond to a straight trajectory.

#### D - differential

`Kd * Ed(n)` is the derivative term that predicts error behavior and mitigates the system from deviating from setpoint in the future. However, if `Kd` is too large, the controller can become less sensitive.

## Function Implementation

### PID::UpdateError

The PID errors were updated using the following code. As noted earlier, the previous CTE is stored for computing the D-error in subsequent time steps.

```
  p_error = cte;
  i_error = i_error + cte;
  d_error = cte - prev_cte;
  prev_cte = cte;
```

### PID::TotalError

This function evaluates the actual steering value and returns it to `main.cpp`.
The proportional, integral and derivative error terms are combined into the total error.
The final return value is restricted to be in the range [-1,1] are required by the simulator.
The last two `if` statements threshold values to these ranges.

```
  double steer = -Kp * p_error - Kd * d_error - Ki * i_error;
  // ensure that steering value is within the allowed range.
  if (steer < -1.0) steer = -1.0;
  if (steer > 1.0) steer = 1.0;
  return steer;
```

```
pid.UpdateError(cte);
steer_value = pid.TotalError();
```
## Parameter Tuning

The PID parameters were chosen using manual tuning. The following steps were used:

1. Since steering angle and CTE are of order of 1.0, quarter (0.25) value was used as a starting point for Kp.
2. Gradually Kd was reduced to remove oscillations - a twiddle-inspired of manual tuning was used.
3. Finally, Ki was tuned, where the order of mangnitude was reduced upon each iteration till satisfactory beahvior was observed.

The final tune parameters are: `Kp=0.287`, `Kd=0.00072`, and `Ki=5.675`.