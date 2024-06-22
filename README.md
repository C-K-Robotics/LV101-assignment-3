# LV101-assignment-3
# Closed-Loop (Feedback) Control
A closed-loop controller or feedback controller is a control loop which incorporates feedback, in contrast to an open-loop controller or non-feedback controller. A closed-loop controller uses feedback to control states or outputs of a dynamical system.

![Alt text](assets\Closed_Loop_Block_Diagram.png)
- Control action from the controller is dependent on the process (plant) output
- Has a feedback loop which ensures the controller exerts a control action to give a process output the same as the "reference input" or "set point”
# PID Control
PID is a basic feedback controller we've been using a lot on the systems on our robot.
The main concept for a [PID controller](https://en.wikipedia.org/wiki/Proportional–integral–derivative_controller) is to take a desired value and an actual value, and minimize the difference between the two, which is referred to as "error". When the error is near zero then you have succesfully reached the setpoint. PID does this with three components, the Proportional component, the Integral component, and the Derivative component.

![Alt text](assets/PID.png)

This is the underlying equation behind PID, and over the course of this week we will be teaching you what each component means

$u(t) = K_p\cdot e(t) + K_i\cdot\int^t_0{e(\tau)d\tau}~+ K_d\cdot \frac{d}{dt}\left(e\left(t\right)\right)$

*where e(t) is the error at time t, and u(t) is the output (often known as the input of the plant) of the PID controller. In our case, when working with motors, <ins>u(t) will always be the duty cycle demand (related to motor voltage) we give them</ins>*.

Effectively, we have a controller for a single action, so that aspect is tuned to use. We have individually tuned PIDs for the position and velocity of each motor, because each motor is in a unique position on the robot, with different load, friction, and other random qualities.

### Proportional

The first section of the PID Algorithm is proportional term, P. The output here is simply directly proportional to the error.
- $K_p$ , known as the proportional gain.
- $P$ , known as the proportional term of the PID controller.

**_An example scenario:_** We have a PID controller, which has a set of PID gains {0.3, 0, 0}, controlling motors on two sides of a differential drivetrain. Thus, it only has P. We want the position to be at 5 m, but we are currently at 3 m. In this situation, our error is 2 m. The equation now looks as follows.

$u(t) = K_p\cdot e(t)\to u(t) = 0.3(2) = 0.6$

As you can see, since $u(t)$ stands for our motor demands, ranging from [-1.0, 1.0], the $K_p$ gain has a unit of $\frac{demand}{meters}$. it's obvious to see that your controller gain has units that depends on what you $u(t)$ is, including $K_i$ and $K_d$.

The I and D components are simply ignored, because their weight coefficients (gains) are set to 0. In a situation when we are only using P, we get an output directly proportional to the error, 0.6.
The P component is for quick temporary bursts, and it will often be the first thing you attempt to tune.

### Integral

The second section of the PID Algorithm is integral term, I. The output here is the integral of the error.
- $K_i$ , known as the integral gain.
- $I$ , known as the integral term of the PID controller.

Lets consider an example where the PID controller has a set of gains {0, 0.45, 0}. Here is a potential error vs time graph

![image](https://github.com/C-K-Robotics/LV101-assignment-3/assets/68310078/1dc29fa8-d2e7-44d9-887e-e90bf7828b92)

Lets imagine we are now at 3.4 seconds, and the total integral of everything prior is -1.65. Our equation now looks like this.

$u(t) = K_i\cdot\int^t_0{e(\tau)d\tau} \to u(t) = 0.45(-1.65) = -0.7425$

Now, due purely to integral, our output value is -0.7425.
The integral is meant to be more persistent, as if the error is zero, integral will not shift, and the output from the integral component will be relatively stable. Often times, we tune integral term with both its gain and a maximum value (Integral Caps), so that we don't have an integral that compounds into infinity.

_Note that since we are working with a *discrete* error function we need to use [numerical integration](https://en.wikipedia.org/wiki/Numerical_integration) methods_

### Derivative

The final aspect of PID Algorithm is derivative term. The output here is based upon the derivative (rate against time) of error. 
- $K_d$ , known as the derivative gain.
- $D$ , known as the derivative term of the PID controller.

Let's consider an example where the PID controller has a set of gains {0, 0, 0.124}.
Here's the same error vs time graph from before, except this time we are at 2.6 seconds.

![image](https://github.com/C-K-Robotics/LV101-assignment-3/assets/68310078/615a747e-fca8-45cc-a962-afec7a720f13)

Now, we are at 2.6 seconds, observe the green line. The slope of that green line is the derivative of the error at 2.6 seconds, which is 2.48. This is what the derivative section of the equation looks like now.

$u(t) = K_d\cdot \frac{d}{dt}\left[e\left(t\right)\right] \to u(t) = 0.05(2.48) = 0.124$

The derivative has set our output to be 0.124. Derivative is meant to dampen the $u(t)$ signal. If the error is moving toward zero, the derivative will add a component to make the error grow, and if the error is growing its component will make it move toward zero. It effectively dampens change.

_Note that since we are working with a **discrete** error function we need to use [numerical differentiation](https://en.wikipedia.org/wiki/Numerical_differentiation) methods_

# Exercise #1

For this exercise, we'll be writing a PID controller to do a position control on an single-jointed arm plant. There is some [starter code](https://github.com/C-K-Robotics/LV101-assignment-3/blob/main/PID%20Controller.vi), but you'll be implementing the things you've learned above yourself. You have full freedom to change anything in the `PID Controller.vi` section. Of course, you can add indicators as you'd like to test out your values, and execute the `main.vi` to test out your controller.
You can modify the following VIs:
- `PID Controller.vi` (No modification need. But feel free to add indicators if you want to check the values, or try the LabVIEW Probe Tool)
- `Proportional Cal.vi`
- `Integration Cal.vi`
- `Derivative Cal.vi`
- `main.vi` Only __PID gains__

### Integral Calculation

If you've taken calculus, you probably know about Riemann sums. Riemann sums are how we calculate integrals in code. While data in the real world is continuous, code is not. Code, however, can get pretty close if the time interval is small enough. What we get from this is a series of points that describe the curve.

![image](https://github.com/C-K-Robotics/LV101-assignment-3/assets/68310078/031d4c13-e7f8-43fe-9ce8-56ad5f9dfb6d)

Imagine this is a graph of error. What we are showing here is a Right-Hand Riemann sum. This is how we recommend you implement integral calculations (as its pretty simple), but you can do it in other ways. You can experiment with this data set here: [Rieman Sum Simulator](https://www.desmos.com/calculator/kye17rgo1b)

### Derivative Calculations

The way we do derivatives is also the same way we do derivative in calculus. A derivative is the slope between two points, where the dt is a limit to infinity. In our case, with code, the dt is simply as fast as we get data, which gets us pretty close. If you'd like to experiment with how the dt changes the results, I have another desmos sim here: [Derivatives Approximation](https://www.desmos.com/calculator/yhwf0jrps8). It uses the same data as the integral sim.

![image](https://github.com/C-K-Robotics/LV101-assignment-3/assets/68310078/d9c38ef6-93e9-44b9-8c44-de20651e296f)

Between each data point, you see the yellow slope line and the two blue component lines. for PID, we consider the dE at each point as the slope of the line leading to it. The dE at the green marked point would be the slope between the two marked points.

# Exercise #2
![Alt text](assets\2nd_Order_System_Response.png)

In the exercise, the closed-loop system has generated an output signal $y(t)$ as a response to the given reference signal $r(t)$, where 
- $r(t)$ is the setpoint (reference) signal.
- $y(t)$ is the sensor value.

Here, we introduce new concepts: Rise time, Overshoot. Sometimes, we use them as conditions for us to design controller.

Please tune the PID controller you designed in the exercise #1 to make the closed-loop system generate an output signal that satisfies:
- Overshoot must be lower than 30%.
- Rise time must be less than 0.4 sec.

### Rise Time
As the graph shown above, the rise time stands for how much time the output signal spends to reach the reference for the first time.
Specifically, if we normalize the output signal with our reference, the time it takes for the output siganl to reach from 10% to 90% of the given refernce.

### Overshoot
As the graph shown above, if we still normalize the output signal with our reference, Overshoot stands for how many percent it surpass the reference.

For example, the maximum value of your closed-loop system response (aka the output signal) shows where the overshoot takes place. Let's say it's 125% of the refernce. Then, the overshoot is 25%.

# Submission
Eventually, you should get a plot in `main.vi` like this. Make sure you don't change any parameter of the controller, except the __PID gains__.

If you get a similiar plot, you're good to go and submit the assignment! (do a `git push`)

![Alt text](assets/solution_results.png)
