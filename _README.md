# How does MPC works

* Receive X and Y points of perfect trajectory ahead.
* Use polyfit 3rd order (cubic) polynomial to fit best line between give coordinates as a result - a, b  and c coefficients.

f(x) = ax^3 + bx^2 + cx + d
```
auto coeffs = polyfit(ptsx, ptsy, 3);
```

* Set initial coordinates, psi and velocity values.
* Calculate cross track error (CTE) - error between center of the road and the vehicles position.

CTE_t+1 = cte_t + v_t * sin(epsi_t) * dt

cte_t = f(x_t) - y_t  # for 1st order polynomial

```
double cte = polyeval(coeffs, x) - y;
```
cte_t: Difference between the line and the current vehicle position y.
epsi_t: Orientation error

* Calculate Orientation Error.

epsi_t+1 = epsi_t + v_t / Lf * delta_t * dt

epsi_t = psi_t - psi_des_t

```
double epsi = psi - atan(coeffs[1]);
```

Lf: Distance between cars center of mass and front
delta_t: current steering angle
