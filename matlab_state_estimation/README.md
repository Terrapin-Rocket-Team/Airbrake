# Matlab State Estimation

This folder provides the tools to easily test various types of mock and 
real flight data against proposed state estimation filters. The entrance
point for this tool is main.m. Running main.m will do three things.

1. It will pull in or create mock flight data
1. Run the flight data through a filter
1. Plot the actual vs. measured vs. filter output

## Data Types

### Mock Data
This is entirely mock rocket data from just propogating equations of motion.
To set this up please edit the rocket parameters in section 1 under the Mock
data section.

### OpenRocket Data
This allows you to take an open rocket output data csv and use those values
to run through the state estimation. When exporting an open rocket sim csv
it is important to export all data (all columns) and to export in SI units
(m, kg, s).

### Flight Data
This is data that was logged by MMFS. Currently it supports data logged by 
MMFS v1.0.0 (aka 2023-24 Payload data).

## Filters

This folder will look at a ton of different filters. The over arching goal is to take our measurements and what we know about the dynamics and put that together to produce a state of the rocket (pos, velo) that is more precise than any one thing by itself.

Our measurements will consist of a GPS, 2 Barometers (pressure and temperature), and an IMU (acclerameter, gyroscope and magnetometer). 

## Dyanmics

## Linear Kalman Filter

## Extended Kalman Filter (Drag)

For this filter we are assuming that our measurements are coming in the inertial frame already.

$$
\vec{X} = 
\begin{bmatrix} 
^I\vec{r}_{P/O} \\
^I\vec{v}_{P/O}
\end{bmatrix} =

\begin{bmatrix} 
x \\ y \\ z \\ \dot{x} \\ \dot{y} \\ \dot{z}
\end{bmatrix} 
$$

Our state space model will be defined by:

$$
\dot{\vec{X}} = \vec{f}(\vec{X})
$$

$$
\vec{Y} = \vec{h}(\vec{X})
$$

$$
\dot{\vec{X}} = \begin{bmatrix} 
\dot{x} \\ \dot{y} \\ \dot{z} \\ \ddot{x} \\ \ddot{y} \\ \ddot{z}
\end{bmatrix} = 
\begin{bmatrix} 
\dot{x} \\ \dot{y} \\ \dot{z} \\
\frac{-F_{D_R}}{m(t)}sin([IB]_{33})cos([IB]_{11})\\
\frac{-F_{D_R}}{m(t)}sin([IB]_{33})sin([IB]_{11})\\
\frac{-F_{D_R}}{m(t)}cos([IB]_{33}) + \frac{-g}{m(t)}
\end{bmatrix}
$$

Where

$$
m(t) = \begin{cases}
m_{wet} & \text{if } t < t_{boost} \\
m - (\frac{m_{wet} - m_{dry}}{t_{coast} - t_{boost}}) \Delta t & \text{if } t_{boost} \leq t < t_{coast}\\
m_{dry} & \text{if } t > t_{coast}
\end{cases}
$$

Constant $\dot{m}$ approximation. Given a wet and dry mass from open rocket/solids team.

$$
F_{D_R} = \frac{1}{2} \rho v^2 C_d A
$$

$$
\rho = \frac{P}{R_{air}T}
$$

For $\rho$, P and T will be measured from the barometer and $R_{air} = 287$ J/Kg-K.

$$
v^2 = \dot{x}^2 + \dot{y}^2 + \dot{z}^2
$$

$$
\vec{Y} = 
\begin{bmatrix} 
x_{gps} \\ y_{gps} \\ z_{gps} \\ z_{baro1} \\ z_{baro2} \\ \ddot{x}_{imu} \\ \ddot{y}_{imu} \\ \ddot{z}_{imu} - g
\end{bmatrix} = 
\vec{h}(\vec{X}) =
\begin{bmatrix} 
x \\ y \\ z \\ z \\ z \\
\frac{-F_{D_R}}{m(t)}sin([IB]_{33})cos([IB]_{11})\\
\frac{-F_{D_R}}{m(t)}sin([IB]_{33})sin([IB]_{11})\\
\frac{-F_{D_R}}{m(t)}cos([IB]_{33}) + \frac{-g}{m(t)}
\end{bmatrix}
$$

Notice the subtraction of the gravity term from the accelerameter z direction before it enters the filter.

### EKF Equations

#### Initialization

Set $\Delta t$, $X_0$, $P_0$ to a value.

Find $F = e^{A \Delta t}$

#### Predict

$$
\hat{X}_k^- = \hat{X}_{k-1}^+ + \vec{f}(\hat{X}_{k-1}^+) \Delta t
$$

$$
P_k^- = FP_{k-1}^+F + Q_k
$$

#### Update

$$
K_k = P_k^- H^T(H P_k^- H^T + R)^{-1}
$$

$$
\hat{X}_k^+ = \hat{X}_k^- + K_k(z_k - H\hat{X}_k^-)
$$

$$
P_{k}^+ = (I - K_kH)P_{k-1}^-(I - K_kH)^T + K_k R K_k^T
$$

### EKF Matrices

$$
A = \frac{\partial \vec{f}}{\partial \vec{X}} =
\begin{bmatrix}
0 & 0 & 0 & 1 & 0 & 0 \\
0 & 0 & 0 & 0 & 1 & 0 \\
0 & 0 & 0 & 0 & 0 & 1 \\
0 & 0 & 0 & \dot{x}Dsin([IB]_{33})cos([IB]_{11}) & \dot{y}Dsin([IB]_{33})cos([IB]_{11}) & \dot{z}Dsin([IB]_{33})cos([IB]_{11}) \\
0 & 0 & 0 & \dot{x}Dsin([IB]_{33})sin([IB]_{11}) & \dot{y}Dsin([IB]_{33})sin([IB]_{11}) & \dot{z}Dsin([IB]_{33})sin([IB]_{11}) \\
0 & 0 & 0 & \dot{x}Dcos([IB]_{33}) & \dot{y}Dcos([IB]_{33}) & \dot{z}Dcos([IB]_{33}) \\
\end{bmatrix}
$$

$$
D = \frac{- \rho C_{D_R} A}{m(t)}
$$

$$
F = e^{A \Delta t} = I + A \Delta t + \frac{(A \Delta t)^2}{2!} + \frac{(A \Delta t)^3}{3!} + \frac{(A \Delta t)^4}{4!} + ...
$$

$$
C = H = \frac{\partial \vec{h}}{\partial \vec{X}} =
\begin{bmatrix}
1 & 0 & 0 & 0 & 0 & 0 \\
0 & 1 & 0 & 0 & 0 & 0 \\
0 & 0 & 1 & 0 & 0 & 0 \\
0 & 0 & 1 & 0 & 0 & 0 \\
0 & 0 & 1 & 0 & 0 & 0 \\
0 & 0 & 0 & \dot{x}Dsin([IB]_{33})cos([IB]_{11}) & \dot{y}Dsin([IB]_{33})cos([IB]_{11}) & \dot{z}Dsin([IB]_{33})cos([IB]_{11}) \\
0 & 0 & 0 & \dot{x}Dsin([IB]_{33})sin([IB]_{11}) & \dot{y}Dsin([IB]_{33})sin([IB]_{11}) & \dot{z}Dsin([IB]_{33})sin([IB]_{11}) \\
0 & 0 & 0 & \dot{x}Dcos([IB]_{33}) & \dot{y}Dcos([IB]_{33}) & \dot{z}Dcos([IB]_{33}) \\
\end{bmatrix}
$$

$$
Q_k = \sigma_{\ddot{x}}^2 \begin{bmatrix}
\frac{\Delta T^4}{4} & 0 & 0 & \frac{\Delta T^3}{2} & 0 & 0 \\
0 & \frac{\Delta T^4}{4} & 0 & 0 & \frac{\Delta T^3}{2} & 0 \\
0 & 0 & \frac{\Delta T^4}{4} & 0 & 0 & \frac{\Delta T^3}{2} \\
\frac{\Delta T^3}{2} & 0 & 0 & \Delta T^2 & 0 & 0 \\
0 & \frac{\Delta T^3}{2} & 0 & 0 & \Delta T^2 & 0 \\
0 & 0 & \frac{\Delta T^3}{2} & 0 & 0 & \Delta T^2 \\ 
\end{bmatrix}
$$

Where $\sigma_{\ddot{x}}^2$ is a tunable parameter (https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/07-Kalman-Filter-Math.ipynb continuous white noise model).

$$
R = 
\begin{bmatrix}
\sigma_{gps}^2 & 0 & 0 & 0 & 0 & 0 & 0 & 0 \\
0 & \sigma_{gps}^2 & 0 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & \sigma_{gps}^2 & 0 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & \sigma_{baro1}^2 & 0 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & \sigma_{baro2}^2 & 0 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & \sigma_{imu}^2 & 0 & 0 \\
0 & 0 & 0 & 0 & 0 & 0 & \sigma_{imu}^2 & 0 \\
0 & 0 & 0 & 0 & 0 & 0 & 0 & \sigma_{imu}^2 \\
\end{bmatrix}
$$

## Extended Kalman Filter (Drag + Orientation)