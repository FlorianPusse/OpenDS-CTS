package settingscontroller_client.src;


import eu.opends.main.Simulator;

public class Parameters {

    /**
     * Num pedestrians in the scene
     */
    public static final int NUM_PEDESTRIAN = 4;

    /**
     * Resultion of the cost map. 0.1 originally
     */
    public static final float mapResolution = (float) Simulator.stepsize;

    /**
     * Discretization used for Hybrid A*
     */
    public static final float discretization = 0.8f;

    /**
     * Orientation discretization used for Hybrid A*
     */
    public static final float orientationDiscretization = 0.1f;

    /**
     * Discretization of the belief used in APPL based methods
     */
    public static final float BELIEF_ANGLE_DISCRETIZATION = 2;

    /**
     * Length of the car in meters
     */
    public static final float car_length = 4.25f;

    /**
     * Width of the car in meters
     */
    public static final float car_width = 1.7f;

    /**
     * Width of the cost map in pixels
     */
    public static final float map_width = 3057;

    /**
     * Height of the cost map in pixels
     */
    public static final float map_height = 3873;

    /**
     * Goal range radius
     */
    public static final float R_GOAL = 5;

    /**
     * Reward for reaching the goal
     */
    public static final float GOAL_REWARD = 1000f;

    /**
     * Penalty for hitting a pedestrian
     */
    public static final float HIT_PENALTY = 1000f;

    /**
     * Maximum speed of the car
     */
    public static final float MAX_SPEED = 50;

    /**
     * Possible steering angles of the car
     */
    public static final Integer[] possibleSteeringAngles = new Integer[]{0, -5, 5, -10, 10, -15 , 15, -20, 20, -25, 25, -30, 30, -35, 35};

    /**
     * Number of angles
     */
    public static final int angles = possibleSteeringAngles.length;

    /**
     * Discretization used in non-holonomic heuristics
     */
    public static final float non_holonomic_discretization = 1.0f;

    /**
     * Orientation discretization used in non-holonomic heuristics
     */
    public static final float non_holonomic_direction_discretization = 5; // degree

    /**
     * Initial X-coordinate of the goal position
     */
    public static final int goalX = 544;

    /**
     * Initial Z-coordinate of the goal position
     */
    public static final int goalZ = 3333;

    /**
     * Angle used for the InFront function
     */
    public static final float IN_FRONT_ANGLE_DEG = 70;

    /**
     * Last steps to remember for car intention generation
     */
    public static final int MAX_PATH_STEPS = 40;

    /**
     * Width/Height of car intention snipped ORIGINAL
     */
    public static final int INTENTION_SNIPPED_SIZE = 250;

    /**
     * End width/height of the car intention
     */
    public static final int CAR_INTENTION_SIZE = 100;

}
