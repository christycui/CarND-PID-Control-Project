# PID Controller

Car successfully drives around the loop in the simulator with an implemention of the PID controller.


## P, I, D
The steering angle of the vehecle is decided by three components: P, I and D.
* P: P is proportional to the CTE (cross-track error)
* I: I is the integral of CTE from past iterations
* D: D is the difference between current CTE and last CTE measured

For example, when the vehicle is first started, its initial CTE is 0.7598 meters. The P error is therefore 0.7598; the D error is zero because there was no previous measurement and the I error is 0.7598 because that's the sum of all measurements so far (1 measurement). 

## Selection of hyperparameters
The coefficients of P, I, D are dynamically chosen by the twiddle algorithm (aka Coordinate Ascent). The initial values I chose are {0, 0, 0} and the change values are {1, 1, 1}. 

In addition to the twiddle algorithm, I manually added different coefficients in front of the P error, I error and D error. They are {0.1, 0.01 and 1}. This is because we know desirable steering angle is in the range of [-1, 1] and CTE measurement is in the range of [-5, 5], assuming the road is 10 meters wide. Therefore, we want the magnitude of P error to be 0.1 of its original value. For the D error, it is the difference between two measurements given delta_t which is 0.1s in the simulator, it is already in the expected range so we don't change the scale and keep it at 1. Lastly, for the I error, considering the fact that every 0.1s a new CTE is added, we scale it down to 0.01 of its original value. 

Continuing with the example in the previous section (with initial CTE 0.7598), the steering angle therefore is calculated as 0.1\*0.7598 (P) + 0.01*0.7598 (I) + 0 (D) = 0.084.