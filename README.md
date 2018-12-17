# **! Do not distribute this repository or any part of it without permission !**

# OpenDS-CTS-01

Benchmarking simulated autonomous cars with OpenDS

## Getting Started

To get started, you need to set up your system for OpenDS. We recommend using a recent Linux distro, as existing car controllers only work with Linux. Currently implemented cars are all implemented on Ubuntu 17/18 machines. Other Linux distros might work as well.

! Commands provided for installation below might vary depending on your environment. !

### OpenDS installation:
1) Install OpenJDK 1.8

    `sudo apt-get install openjdk-8-jdk`

2) Install JavaFX/OpenJFX
 
    `sudo apt-get install openjfx`
  
3) Clone this repository or download the zip file and extract to a folder of your choice. We call this folder `baseDir` now.

4) Go to `baseDir/LearningAssets` and unpack `assets.tar.xz` into `LearningAssets` folder
5) Go to `baseDir/assets/Scenes/EisenbahnNew` and unpack `data.tar.xz` contents into `EisenbahnNew` folder
6) Download, install, and start IntelliJ (Eclipse might work too).

    `https://www.jetbrains.com/idea/download/#section=linux`
    
   You can start/install Intellij by running `IntellijFolder/bin/idea.sh`. Import the project located at `baseDir`. (Import Project -> Create Project from existing sources -> Next -> Next -> Next -> Next -> Make sure that OpenJDK 1.8 is selected and press Next -> Finish)
  
7) To verify the installation, run `src.eu.opends.main.Simulator` from inside IntelliJ, by right clicking the object and selecting "Run 'Simulator.main()'". After compilation, OpenDS opens and you can drive around in the 3D environment.

8) If an error occurs you might need to change the Java language level to Java 8 (File -> Project structure -> Project. Set Project language level to "8 - Lambdas, type annotations etc.")


### Setting up existing car controllers

To run the existing car controllers, additional software is required.

#### IS-DESPOT-p

1) A C++11 compatible compiler is required. IS-DESPOT-p has been developed using g++ 7.3.0. Newer version might work too.

    `sudo apt-get install build-essential `

2) Makefile

    `sudo apt-get install make`
   
3) Navigate to `baseDir/ISDESPOT/isdespot-car-sim-master/is-despot` and run `make`

4) Navigate to `baseDir/ISDESPOT/isdespot-car-sim-master/is-despot/problems/isdespotp_car` and run `make`. This will create a binary `car`.
    
#### Reactive Controller

1) A C++11 compatible compiler is required. The reactive controller has been developed using g++ 7.3.0. Newer version might work too.

    `sudo apt-get install build-essential `

2) Makefile

    `sudo apt-get install make`
   
3) Navigate to `baseDir/ISDESPOT/reactive-smart/is-despot` and run `make`

4) Navigate to `baseDir/ISDESPOT/reactive-smart/is-despot/problems/reactive_car` and run `make`. This will create a binary `car`.

#### NavA3C-p

1) Install Python3 if not installed. A recent Ubuntu distro should have Python3 already installed. Type `python3` to verify. NavA3C-p has been developed using Python 3.6
2) Install pip3 if not installed. 

    `sudo apt install python3-pip`
3) Additionally install following python libraries:
    - numpy
    - matplotlib
    - scipy 
    - scikit-image 
    - ipython 
    - jupyter
    - pandas
    - sympy
    - nose
    
   Libraries can be installed with 
   
      `pip3 install --user numpy scipy matplotlib ipython jupyter pandas sympy nose scikit-image`
      
   or
   
      `pip install --user numpy scipy matplotlib ipython jupyter pandas sympy nose scikit-image`
      
   depending on how your Python installation is called.
   
  4) Install TensorFlow. NavA3C-p is implemented using Tensorflow 1.8. Newer versions might work too.
     - For general TensorFlow installation instructions: `https://www.tensorflow.org/install/pip`
     - TensorFlow CPU might be fast enough for simple testing. For training and testing with multiple instances, Tensorflow GPU is recommended. All tests were performed with the GPU version installed. You might need to change the settings to make CPU version compatible.
     - To install TensorFlow 1.8 (CPU) run
     
         `pip3 install tensorflow==1.8`
         
       or to install the GPU version run
       
          `pip3 install tensorflow-gpu==1.8`
          
     - Tensorflow GPU additionally requires CUDA 9.0 and cuDNN 7.0
     
#### ADRQN-p

See NavA3C-p.

#### HyLEAP

See NavA3C-p. Additionally:

1) Navigate to `baseDir/ISDESPOT/smart-car-sim-master/is-despot` and run `make`

2) Navigate to `baseDir/ISDESPOT/smart-car-sim-master/is-despot/problems/hybridVisual_car` and run `make`. This will create a binary `car`.

### Running existing car controllers

Existing car controllers can be run using their predefined helper classes located in `baseDir/tools/`. There are 5 variables you can change:

- MAX_INSTANCES: Number of instances to run simultanously. Not for !ADRQN/HyLEAP!
- type: Experiment to run. This can be any of Varying ped. speeds and crossing distances `(= SPEED_DISTANCE)`, pedestrian walking in ZigZag line `(= ZIGZAG_FOLLOW)`, or multiple pedestrians crossing the street `(= MULTIPLE_PEDESTRIANS)`
- mode: Running the simulation in testing mode `(= TESTING)` or training mode `(= TRAINING)`
- TRAINING_SET: The scenario to run. 
    1. Parameter: Pedestrian crossing the street from `LEFT` or `RIGHT`
    2. Parameter: Intersection type. Car approaching an intersection `(= APPROACHING)`, leaving an instersection `(= LEAVING)`, or no intersecton `(= NONE)`
    3. Parameter: Is the car turning on an intersection. If `true`, the other two parameters are ignored.
- obstaclePosition: How to occlude the pedestrian. Obstacles can be on the street `(= ON_STREET),` on the pavement `(= ON_PAVEMENT)` or no obstacle at all `(= NONE)`

##### IS-DESPOT-p

- Run `StartupISDESPOTp.java` inside Intellij

##### Reactive Controller

- Run `StartupReactiveCar.java` inside Intellij

##### NavA3C-p

- Run `StartupNavA3Cp.java` inside Intellij
- Make sure that `baseDir/Python/GA3C-DeepNavigationWithNavigation/Config.py` has the same number of agents set.
- Run `python3 baseDir/Python/GA3C-DeepNavigationWithNavigation/GA3C.py` 

##### ADRQN-p

- Run `StartupADRQNp.java` inside Intellij
- Run `python3 baseDir/Python/main_ADRQNp.py` 

##### HyLEAP

- Run `python3 baseDir/Python/main_hybridVisual.py`
- Run `StartupHybridCar.java` inside Intellij


### Modifying OpenDS-CTS01

OpenDS-CTS01 has two java source folders:
- `baseDir/src`: Contains mostly OpenDS related classes. The most important ones are located in `eu/opends/main` (contains the simulator itself) and `eu/opends/settingsController` (handles external connections in OpenDS; can be modified to generate new observations or add new car control)
- `baseDir/tools/settingscontroller_client/src`: Contains the car controllers located in `Controllers`, path planner in `PathPlanning`, and the evaluator in `Evaluation`.

#### Modifying simulation parameters:

- You can modify the simulation parameters by setting one or multiple variables in the constructor of your OpenDS controller

| Parameter name | Type | Description  |
|---|---|---|
| MAX_TRIALS | Integer | Number of trials per scene when testing your application. After MAX_TRIALS trials the next scene is tested. (Only TESTING mode) |
| MAX_SPEED | Float | The maximum speed the car is allowed to drive. Acceleration actions that try to go beyond this speed are ignored. |
| observationDelimiter | String | The delimiter between the `observation` part of the message your external application receives |
| useCarIntention | Boolean | Whether your external application wants to receive the carIntention as part of the message.

If you are working with your own controller you can change the tested scenario and the simulation mode by setting `type`, `mode`, `TRAINING_SET`, as defined above. Note, that you need to modify the values set in the Simulator class to be the same. It might be a good idea to implement your own helper class in the style of the `Startup*` classes to set these values automatically.

#### Modifying car controllers

- Car controllers are defined inside `baseDir/tools/settingscontroller_client/src/Controllers`. Methods are mapped to controllers as follows:

- IS-DESPOT-p uses the ISDESPOTController class.
- ADRQN-p/NavA3C-p both use the DiscretizedController class.
- HyLEAP uses the HybridController class.
- The reactive controller uses the ReactiveController class.

- Additionally, many simulation and car related variables are defined inside the `baseDir/tools/settingscontroller_client/src/Parameters`.

#### Defining new car controllers

- To create your own controller, start by creating a new class inside `baseDir/tools/settingscontroller_client/src/Controllers` and extending `AbstractController`. We call the new controller `MySimpleController`

- To make your controller runnable, add a main function:
    ```java
    public static void main(String[] args) throws IOException {
        new MySimpleController().initController();

        while (true) {
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
    ```
  This will create an initialize an instance of your controller (load cost map, path planner, ...)
  
- For testing purposes, we overwrite the existing `chooseAction` method. This function takes the current observation (location, rotation, speed of the car, and position of pedestrians) + the angle planned by the path planner and returns an action of the car to execute.
    ```java
    @Override
    public AbstractAction chooseAction(SubscribedValues currentObservation, float plannedAngle){
        return new SimpleAction(plannedAngle, ACCELERATE);
    }
    ```
  In this example, we simply use the planned angle of the path planner and accelerate. This method is only used for simple testing. For non-toy examples, there is a Socket connection opened automatically. You can write your own controller applications and connect them to your OpenDS controller on port `CONTROLLER_PORT [= 1245]`. As soon as your external application is ready, you can send a `RESET\n` message to reset the current scene. Observations will be send to this port and the controller needs to send back an text answer.
  
- Observations have the following form:
    ```java
   terminal;reward;converted_angle;observation[;car_intention]
    ```
    where
    - terminal: Is the current state a terminal state
    - reward: The last received reward
    - converted_angle: The planned steering angle between -1 and 1
    - observation: The current oversvation of the form `carPositionX,carPositionY,carSpeed,ped_1.x,ped_1.y,...,ped_n.x,ped_n.z` where n is the number of pedestrians
    - car_intention: The car intention image of size 100*100*3, separated by `,`. Needs to be enabled by setting `useCarIntention = true` in your controller.

- By default, an answer of the form `angle,acc`, where angle is a steering angle \in [-40, 40] and acc is an acceleration action \in {0,1,2}, where `0 = ACCELERATE`, `1 = MAINTAIN`, `2 = DECELLERATE` is expected. To change the way the answer is processed on the Java side, overwrite the `getAction` method.
    ```java
    @Override
    public AbstractAction getAction(String answer, float plannedAngle) {
        return SimpleAction(answer);
    }
   ```
You can also create your own action by implementing the `baseDir/tools/settingscontroller_client/src/Actions/AbstractAction` interface.
   
- To finish, you can implement your own reward function by overwriting the `calculateReward` method.
   ```java
    @Override
    public Reward calculateReward(SubscribedValues currentState, AbstractAction lastAction, SubscribedValues lastState){
        Reward r = new Reward();
        r.reward = -1;
        return  r;
    }
    ```
   This reward will be computed automatically and is send back to your controller in the next iteration. By default the reward is always `0`. You can manually add new final states by setting `r.terminal = true`. If the goal is reached, this will be set automatically.
   
- As soon as you have reached a terminal state, indicated by the `terminal` variable in the observation, you can process your results and then send `RESET\n` to reset the scene and continue.
   
- To run your new car, first start up the simulator and then continue with your controller. Finally, connect with your external program.

- Only when testing your application (**not** as in testing the performance): To manually reset the scene use <kbd>L</kbd>, to continue with the next scene use <kbd>K</kbd>

### Modifying scenarios

You can modify existing scenarios by editing the corresponding OpenDS DrivingTasks inside `baseDir/assets/DrivingTasks/Projects`. Note that this will change the scenario for **all** controllers automatically. You can only edit the starting positions, waypoints and segments of the pedestrians. For the movement speed variable is only used for the multiple pedestrian exerpiments in test mode and the zigzag experiment. For the Speed/Distance experiment, the movement speed is set by OpenDS online.
