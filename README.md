# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

This project goal is to create fully functional MPC (Model Predictive Control) which is able autonomously drive car in virtual environment by following trajectory coordinates.

The goals / steps of this project are the following:

* Apply MPC to steering and throttle of a simulated car.
* Compensate latency in control system (100ms).
* Tune hyperparameters of controller so that car could autonomously drive a lap without living the track. No tire may leave the drivable portion of the track surface. The car may not pop up onto ledges or roll over any surfaces that would otherwise be considered unsafe (if humans were in the vehicle).

[//]: # (Image References)

[image1]: ./CarND_Controls_MPC.gif "Demo"
[image2]: ./classic_model.png "Classic Model"
[image3]: ./simplified_model.png "Simplified Model"
[image4]: ./model_init.png "Model init"
[image5]: ./psidot.png "Psi Dot"
[image6]: ./model_cont.png "Model Cont"
[image7]: ./model_descrete.png "Model Descrete"

## Demo

Full video available : https://www.youtube.com/watch?v=ba4u26A7mL4

![alt text][image1]

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

---

## Model

MPC stands for Model Predictive Control, so let's start with the model.

In this project [kinematic bicycle model](http://www.me.berkeley.edu/~frborrel/pdfpub/IV_KinematicMPC_jason.pdf) is used. Here is classic drawing for derivation of model's equations:

![alt text][image2]

**x**, **y**, **psi** and **v** are state variables of the vehicle. Control inputs are **delta** (steering angle) and **a** - acceleration that isn't shown on drawing, but we assume it is directed along the vehicle axis.

Drawing shows that velocity vector **v** forms angle **beta** with vehicle main axis. This is important for long vehicles or for vehicles with ability to steer by front and rear wheels, or drift.

None of this is applicable to project, so we can assume  velocity vector is directed along main axis of vehicle. So **beta** is zero and drawing for this case looks like this:

![alt text][image3]

Using trig and formula for [angular velocity](https://en.wikipedia.org/wiki/angular_velocity), we can formulate following system of equations:

![alt text][image4]

where **a** is vehicle acceleration.  

From right triangle with catheti of **R** and **Lf** we can derive equation for **R** , substitute to psi dot equation, and assume tan(delta) = delta for small values:

![alt text][image5]

So the final system of equations for project model is:

![alt text][image6]

Or in discrete case:

![alt text][image7]

## N and dt

Two of all hyperparameters to tune throughout the project. N - number of timesteps and dt - prediction timstep between actuations. Rule of thumb of those two parameters are that we try to increase N and decrease dt as much as we can - result is compromise between those two values. In thise project after multiple test N=20 and dt=0.1 showed best results. If dt is to small or too big as previously tried 0.2 and 0.01 - the prediction line is not correct. Same with when increasing N to 30 or 10.

## Waypoints

To pass correct waypoints to simultor we first need to transform them in car coordinates as follow:

```
// Transform waypoints in car coordinates
for (int i = 0; i < ptsx.size(); i++) {
  way_x.push_back((ptsx[i] - px)*cos(psi) + (ptsy[i] - py)*sin(psi));
  way_y.push_back(-(ptsx[i] - px)*sin(psi) + (ptsy[i] - py)*cos(psi));
}
```

Ans then we format those waypoints from vector to Eigen. Then they are ready for polyfit. For this project py points were ignore to simplify model.

## Latency

To make project closer to real life artificial latency of 100ms was added between prediction and applying actuations. Thas can be easily solved by using discrete predicting function before solve.

```
// Predict state after latency before passing to the solver
double dt = 0.1;
px = v * dt;
psi = -v * steer_angle * dt / 2.67;
```

---

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./
