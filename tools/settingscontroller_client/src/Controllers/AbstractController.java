package settingscontroller_client.src.Controllers;

import com.jme3.math.Vector2f;
import eu.opends.main.Simulator;
import org.imgscalr.Scalr;
import settingscontroller_client.src.AccelerationType;
import settingscontroller_client.src.Actions.AbstractAction;
import settingscontroller_client.src.Actions.SimpleAction;
import settingscontroller_client.src.Controllers.OpenDSConnection.MessageReceiver;
import settingscontroller_client.src.Controllers.OpenDSConnection.SubscribedValues;
import settingscontroller_client.src.Controllers.Visualization.DrawPane;
import settingscontroller_client.src.Evaluation.Config;
import settingscontroller_client.src.Evaluation.ScenarioConfig;
import settingscontroller_client.src.Evaluation.SimulationSequence;
import settingscontroller_client.src.Parameters;
import settingscontroller_client.src.PathPlanning.ApproximatePositionEntry;
import settingscontroller_client.src.PathPlanning.Dijkstra;
import settingscontroller_client.src.PathPlanning.HybridAStarPhysical;
import settingscontroller_client.src.TrafficObject.Obstacle;
import settingscontroller_client.src.TrafficObject.Pedestrian;
import settingscontroller_client.src.Util.PythonConnector;

import javax.swing.*;
import java.awt.*;
import java.awt.geom.Line2D;
import java.awt.image.BufferedImage;
import java.io.*;
import java.net.ServerSocket;
import java.net.Socket;
import java.net.UnknownHostException;
import java.util.*;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.ReentrantLock;
import java.util.stream.Collectors;

import static eu.opends.main.DataSet.NUM_SETS;
import static settingscontroller_client.src.AccelerationType.ACCELERATE;
import static settingscontroller_client.src.AccelerationType.DECELLERATE;
import static settingscontroller_client.src.AccelerationType.MAINTAIN;
import static settingscontroller_client.src.Controllers.OpenDSConnection.MessageBuilder.*;
import static settingscontroller_client.src.Controllers.OpenDSConnection.MessageSender.*;
import static settingscontroller_client.src.Evaluation.Config.ExperimentType.MULTIPLE_PEDESTRIANS;
import static settingscontroller_client.src.Evaluation.Config.ExperimentType.SPEED_DISTANCE;
import static settingscontroller_client.src.Evaluation.Config.ExperimentType.ZIGZAG_FOLLOW;
import static settingscontroller_client.src.Evaluation.Config.SimulationMode.TESTING;
import static settingscontroller_client.src.Evaluation.Config.SimulationMode.TRAINING;
import static settingscontroller_client.src.Evaluation.Config.sampleDistance;
import static settingscontroller_client.src.Evaluation.Config.sampleMarginSpeeds;
import static settingscontroller_client.src.Parameters.*;
import static settingscontroller_client.src.Util.MathUtil.toAngle;
import static settingscontroller_client.src.Util.PGMUtils.readPGMFile;
import static settingscontroller_client.src.Util.Util.*;

public abstract class AbstractController {

    /* Definition of scenario and mode */

    /**
     * Simulation mode to run. Needs to be in sync with value set in Simulator.
     */
    public Config.SimulationMode mode = TESTING;

    /**
     * Experiment type to run. Needs to be in sync with value set in Simulator.
     */
    public Config.ExperimentType type = SPEED_DISTANCE;

    /**
     * Scenario to chose. Obstacle position can only be set in Simulator!
     */
    int TRAINING_SET = ScenarioConfig.toTrainingSet(ScenarioConfig.PedestrianSide.RIGHT, ScenarioConfig.Intersection.NONE, false);


    /* Parameters used for individual experiment types */

    /* ZIG ZAG experiment **/

    /**
     * Interval to increase length in ZigZag experiment
     */
    float lengthInterval = 0.25f;

    /**
     * Interval to increase angle in ZigZag experiment
     */
    float angleInterval = 10;

    /**
     * Min length in ZigZag experiment
     */
    float zigZagLength = 0.5f;

    /**
     * Min angle in ZigZag experiment
     */
    float zigZagAngle = 30;


    /* Multiple pedestrians experiment */

    /**
     * Min distance to first ped
     */
    float distanceToFirstPed = 1;

    /**
     * Interval of distance to first ped
     */
    float distanceToFirstPedInterval = 1f;

    /**
     * Min delay to first ped
     */
    float delay = 0f;

    /**
     * Interval of delay to first ped
     */
    float delayInterval = 0.5f;

    /**
     * Maximum delay to first ped
     */
    float maxDelay = 4;


    /* Single pedestrian experiment */

    /**
     * Walking speed index of pedestrian
     */
    public int speedIndex = 0;

    /**
     * Crossing distance index of pedestrian
     */
    public int distanceIndex = 0;

    /**
     * Set of possible pedestrian speeds
     */
    List<Float> pedSpeeds = sampleMarginSpeeds(mode);

    /**
     * Set of possible pedestrian crossing distances
     */
    List<Float> distances = sampleDistance(mode);

    /**
     * Print evaluation only if outside connection via socket persists
     */
    public boolean onlyEvaluateConnected = true;

    /**
     * Planner plans a path around pedestrian
     */
    public boolean planAroundPedestrian = true;

    /**
     * Add car intention to observation
     */
    public boolean useCarIntention = false;

    /**
     * Add path to observation
     */
    public boolean usePath = false;

    /**
     * Delimiter of individual observation parameters
     */
    public String observationDelimiter = ",";



    /* Things starting from here are not used for configuration of scenarios */

    /**
     * Visualization of the POMDP
     * **/
    private JFrame frame = null;
    DrawPane pane = new DrawPane();

    /**
     * Display the POMDP
     */
    public boolean isHeadLess = false;

    /**
     * Controller of the car in OpenDS
     */
    CarController carController;

    /**
     * Last exectured action by the car
     */
    AbstractAction lastAction = null;

    /**
     * Connection to the outside world
     */
    PythonConnector pythonConnector;

    /**
     * Outside world connection
     */
    Socket socket;

    /**
     * Input gotten from the outside world
     */
    BufferedReader r;

    /**
     * Buffered input gotten from the outside world
     */
    DataInputStream in;

    /**
     * Output send to the outside world
     */
    OutputStream out;

    /**
     * IP of the OpenDS server
     */
    final String IP = "127.0.0.1";

    /**
     * Port of the OpenDS server
     */
    int OPENDS_PORT = 1244;

    /**
     * Port that is listened on for outside world connections
     */
    int CONTROLLER_PORT = 1245;

    /**
     * Speed up of simulation in openDS. Needs to be in sync with value
     * set in Simulator class
     */
    static final double speedUp = 2.0;

    /**
     * Interval to decide actions in
     */
    static final double interval = 250 / speedUp;

    /**
     * Current path planned
     */
    List<HybridAStarPhysical.ContinuousSearchState> currentPath = null;

    /**
     * Costmap to use for path planning
     */
    static short[][] costMap = null;

    /**
     * Path planner
     */
    HybridAStarPhysical planner;

    /**
     * If we already sent a reset to OpenDS and wait for the car
     * to be reset
     */
    boolean resetSent = false;

    /**
     * If we reached a terminal state and wait for the car
     * to be reset
     */
    boolean terminalReached = false;

    /**
     * Terminal reward received (i.e. crash, hit, or near miss)
     */
    Reward terminalReward = null;

    /**
     * Message sent the first time when terminal message was reached
     */
    String terminalMessage = null;

    /**
     * Visual representation of car belief
     */
    BufferedImage beliefImage = new BufferedImage((int) Parameters.map_width, (int) Parameters.map_height, BufferedImage.TYPE_BYTE_GRAY);

    /**
     * Number of run of current scene in scenario in TRAINING
     */
    public int runs = 0;

    /**
     * Max runs per scenario in TRAINING
     */
    int maxRuns = 50;

    /**
     * If scenarios should be switched after maxRuns in TRAINING
     */
    boolean rotateScenarios = true;

    /**
     * Number of trials in TESTING
     */
    int sceneNumber = 0;
    int sceneTrials = -1;

    /**
     * Maximum number of trials in TESTING
     */
    int MAX_TRIALS = 8;

    /**
     * Currently stored simulation sequence of POMDP
     */
    SimulationSequence simulationSequence = new SimulationSequence();

    /**
     * Maximum speed of the car
     */
    float MAX_SPEED = 50;

    /**
     * Currently targeted speed of the car
     */
    float targetSpeed = 0;

    /**
     * Current X-coordinate of goal of the car
     */
    float goalX = 544;

    /**
     * Current Z-coordinate of goal of the car
     */
    float goalZ = 3333;

    /**
     * Current belief of the car over pedestrian destinations
     */
    Map<Integer,List<Double>> currentBelief = null;

    /**
     * Last used acceleration
     */
    double lastAcceleration = -1;

    /**
     * Last planned angle
     */
    double lastPlannedAngle = -1;

    /**
     * The number of steps the car is stuck somewhere
     */
    int stuckSteps = 0;

    /**
     * The position of the car where it is stuck
     */
    ApproximatePositionEntry stuckPos = null;

    /**
     * The number of times the car may be stuck at before being reset
     */
    final int STUCK_MAX = 20;

    /**
     * Current state of the POMDP
     */
    SubscribedValues parsedValue = null;

    /**
     * Lock to guard the current state of the POMDP
     */
    ReentrantLock valueLock = new ReentrantLock();

    /**
     * Condition that is signaled if new state of POMDP is set
     */
    Condition valueSetCondition = valueLock.newCondition();

    /**
     * Max number of ticks the car is forced to wait before driving
     */
    int MAX_START_TICKS = 20;

    /**
     * Current number of ticks left
     */
    int startTick = MAX_START_TICKS;

    /**
     * Obstacle in the scene
     */
    public Obstacle obstacle = null;

    /**
     * Initializes the car controller
     */
    public void initController(Float param1, Float param2, Integer OPENDS_PORT,
                               Integer PYTHON_PORT, Config.SimulationMode mode, Config.ExperimentType type, Integer trainingSet, Boolean isHeadLess) throws IOException {

        if(OPENDS_PORT != null){
            this.OPENDS_PORT = OPENDS_PORT;
        }
        if(PYTHON_PORT != null){
            this.CONTROLLER_PORT = PYTHON_PORT;
        }
        if(mode != null){
            this.mode = mode;
        }
        if(type != null){
            this.type = type;
        }
        if(trainingSet != null){
            this.TRAINING_SET = trainingSet;
        }

        if(type == MULTIPLE_PEDESTRIANS){
            TRAINING_SET = 10;
        }

        if(param1 != null && param2 != null){
            if(type == SPEED_DISTANCE){
                this.speedIndex = Math.round(param1);
                this.distanceIndex = Math.round(param2);
            }
            if(type == ZIGZAG_FOLLOW){
                zigZagLength = param1;
                zigZagAngle = param2;

            }
            if(type == MULTIPLE_PEDESTRIANS){
                this.distanceToFirstPed = param1;
                this.delay = param2;
            }
        }

        if(isHeadLess != null){
            this.isHeadLess = isHeadLess;
        }

        initController();
    }

    /**
     * Initializes the car controller
     */
    public void initController() throws IOException {
        if(staticInitalized.compareAndSet(false,true)){
            init();
        }

        if (!isHeadLess) {
            frame = new JFrame();
            frame.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE);
            frame.setContentPane(pane);
            frame.setPreferredSize(new Dimension((int) Math.round(2 * map_width * Simulator.stepsize), (int) Math.round(2 * map_height * Simulator.stepsize)));
            frame.setSize((int) Math.round(2 * map_width * Simulator.stepsize), (int) Math.round(2 * map_height * Simulator.stepsize));
            frame.setVisible(true);
        }

        changeGoal();

        planner = new HybridAStarPhysical();

        Graphics2D g2 = beliefImage.createGraphics();
        g2.setPaint(Color.WHITE);
        g2.fillRect(0, 0, beliefImage.getWidth(), beliefImage.getHeight());

        ServerSocket echoSocket = new ServerSocket(CONTROLLER_PORT);
        pythonConnector = new PythonConnector(echoSocket);

        Thread t = new Thread(pythonConnector);
        t.start();

        resetConnection(false);

        Thread t1 = new Thread(new MessageReceiver(in, this));
        t1.start();

        carController = new CarController();
        Thread t2 = new Thread(carController);
        t2.start();

        sendInitMessage(out, (int) interval);
    }

    /**
     * Creates the car intention for the current state
     * @param subscribedValues The current state
     * @return The car intention generated
     */
    public double[] getMapExtract(SubscribedValues subscribedValues) {
        int extractScalingFactor = 2;

        final int dWidth = (int) Math.ceil(map_width / extractScalingFactor);
        final int dHeight = (int) Math.ceil(map_height / extractScalingFactor);

        BufferedImage img = new BufferedImage(dWidth, dHeight, BufferedImage.TYPE_INT_RGB);
        Graphics imageGraphics = img.createGraphics();

        pane.paintComponent(imageGraphics, 2);

        int startX = Math.min(Math.max((int) Math.round((subscribedValues.x / 2) - (INTENTION_SNIPPED_SIZE / 2)), 0), dWidth - (INTENTION_SNIPPED_SIZE / 2));
        int startZ = Math.min(Math.max((int) Math.round((subscribedValues.z / 2) - (INTENTION_SNIPPED_SIZE / 2)), 0), dHeight - (INTENTION_SNIPPED_SIZE / 2));

        BufferedImage intentionSnipped = img.getSubimage(startX, startZ, INTENTION_SNIPPED_SIZE, INTENTION_SNIPPED_SIZE);
        BufferedImage rescaledImage = Scalr.resize(intentionSnipped, Scalr.Method.BALANCED, CAR_INTENTION_SIZE, CAR_INTENTION_SIZE);

        int[] convertedImage = rescaledImage.getRGB(0, 0, CAR_INTENTION_SIZE, CAR_INTENTION_SIZE, null, 0, CAR_INTENTION_SIZE);
        double[] extract = new double[CAR_INTENTION_SIZE * CAR_INTENTION_SIZE * 3];

        for (int i = 0; i < convertedImage.length; ++i) {
            Color c = new Color(convertedImage[i]);
            extract[i * 3] = c.getRed() / 255.0;
            extract[i * 3 + 1] = c.getGreen() / 255.0;
            extract[i * 3 + 2] = c.getBlue() / 255.0;
        }

        return extract;
    }

    /**
     * Creates a simple action from the answer of the local controller
     */
    public AbstractAction getAction(String answer, float plannedAngle){
        return new SimpleAction(answer);
    }

    /**
     * Calculates the reward for the current state and the last executed action
     */
    public Reward calculateReward(SubscribedValues currentState, AbstractAction lastAction, SubscribedValues lastState){
        if(!warningPrinted){
            warningPrinted = true;
            System.err.println("Using base class calculateReward function. Consider overwriting it.");
        }
        return new Reward();
    }
    private boolean warningPrinted = false;

    /**
     * Choses an action locally from the current observation and angle
     * planned by the path planner. For real testing, connect to an outside
     * controller instead
     */
    public AbstractAction chooseAction(SubscribedValues currentObservation, float plannedAngle){
        return new SimpleAction(0, MAINTAIN);
    }

    /**
     * Search for a steering angle based on the current state
     */
    int[] searchSteeringAngle(SubscribedValues parsedValue) {
        float startX = (float) parsedValue.x;
        float startZ = (float) parsedValue.z;

        float startTheta = (float) parsedValue.orientation;

        float delta_t = (float) ((interval * speedUp) + 70) / 1000f;
        float speed;

        if (targetSpeed <= 5) {
            speed = 5f;
        } else {
            speed = targetSpeed;
        }

        planner.changeSettings(speed * 0.278f, delta_t);

        HybridAStarPhysical.ContinuousSearchState goal = planner.search(startX, startZ, startTheta, goalX, goalZ, 0, false, beliefImage);

        if (goal == null) {
            return null;
        }

        List<HybridAStarPhysical.ContinuousSearchState> path = planner.getPath(goal);

        pane.updatePath(path, parsedValue);
        currentPath = path;

        return new int[]{path.get(1).parentAngle, path.get(1).reverse ? -1 : 1};
    }

    /**
     * Whether the environment has already been initialized
     */
    private static AtomicBoolean staticInitalized = new AtomicBoolean(false);

    /**
     * Initialize the environment
     */
    public static void init() {
        try {
            costMap = readPGMFile("LearningAssets/combinedmapSimpleDiscretized.pgm");
            HybridAStarPhysical.costMap = costMap;
        } catch (FileNotFoundException e) {
            throw new RuntimeException("Costmap not found!");
        }

        FileInputStream fos;
        try {
            fos = new FileInputStream("LearningAssets/hashmap1");
            ObjectInputStream oos = new ObjectInputStream(fos);
            HybridAStarPhysical.nonHolonomicShortestPaths = (short[][][]) oos.readObject();
        } catch (IOException | ClassNotFoundException e) {
            e.printStackTrace();
        }

        System.out.println("Finished: Create Holonomic with Obstacles heuristic.");

        System.out.println("Create Holonomic with Obstacles heuristic.");
        HybridAStarPhysical.holonomicWithObstacles = new Dijkstra(costMap);
    }

    /** Whether to force a speedup of the car in certain citations
     * Independent of controller choice.
     */
    boolean forceSpeedUp(SubscribedValues thisValue, SubscribedValues lastState) {
        return false;
    }

    public void resetConnection(boolean sendInit) {
        while (true) {
            try {
                socket = new Socket(IP, OPENDS_PORT);
                out = socket.getOutputStream();
                in = new DataInputStream(socket.getInputStream());
                r = new BufferedReader(new InputStreamReader(in));

                if (sendInit) {
                    sendInitMessage(out, (int) interval);
                }

                System.out.println("Connected.");
                break;
            } catch (UnknownHostException e) {
                System.err.println("Don't know about host");
                try {
                    Thread.sleep(5000);
                } catch (InterruptedException e1) {
                    e1.printStackTrace();
                }
            } catch (IOException e) {
                System.err.println("Couldn't get I/O for the connection");
                try {
                    Thread.sleep(5000);
                } catch (InterruptedException e1) {
                    e1.printStackTrace();
                }
            } catch (NumberFormatException e) {
                System.err.println("Port is not a valid number");
                try {
                    Thread.sleep(5000);
                } catch (InterruptedException e1) {
                    e1.printStackTrace();
                }
            }
        }
    }

    Random randomTicker = new Random();

    /**
     * Reset the properties used in the current scene
     */
    public void resetProperties(){
        terminalReached = false;
        terminalReward = null;
        terminalMessage = null;

        resetSent = true;
        pane.reset();

        simulationSequence.reset();
        stuckPos = null;
        stuckSteps = 0;
        lastAcceleration = -1;
        lastPlannedAngle = -1;
        targetSpeed = 0;

        if (mode == TRAINING) {
            startTick = randomTicker.nextInt(MAX_START_TICKS);
        } else {
            startTick = randomTicker.nextInt(5);
        }
    }

    /**
     * Change the current goal of the car
     */
    void changeGoal() {
        if (TRAINING_SET >= 9 && TRAINING_SET <= 11 || TRAINING_SET == 16 || TRAINING_SET == 17 || TRAINING_SET == 21 || TRAINING_SET == 22) {
            goalX = 1464.43f;
            goalZ = 1294.79f;
        } else if ((TRAINING_SET >= 12 && TRAINING_SET <= 13) || TRAINING_SET == 15) {
            goalX = 1194.0f;
            goalZ = 1936.0f;
        } else if (TRAINING_SET == 14) {
            goalX = 1860.0f;
            goalZ = 1328.0f;
        } else if (TRAINING_SET == 18 || TRAINING_SET == 19){
            goalX = 1403.38f;
            goalZ = 1445.41f;
        } else if (TRAINING_SET == 20) {
            goalX = 1015.30f;
            goalZ = 2280.85f;
        } else {
            switch (TRAINING_SET % 3) {
                case 0:
                    goalX = 1464.43f;
                    goalZ = 1294.79f;
                    break;
                case 1:
                    goalX = 1194.0f;
                    goalZ = 1936.0f;
                    break;
                case 2:
                    goalX = 682.0f;
                    goalZ = 2996.0f;
                    break;
                default:
                    throw new RuntimeException("Something is wrong here");

            }
        }

        pane.updateGoal(goalX,goalZ);
    }

    /**
     * Draws the current belief of the car
     */
    public void drawBelief(List<Pedestrian> pedestrians, SubscribedValues currVal) {
        Graphics2D g2 = beliefImage.createGraphics();
        g2.setPaint(Color.WHITE);
        g2.fillRect(0, 0, beliefImage.getWidth(), beliefImage.getHeight());

        if (pedestrians != null && currentBelief != null) {
            g2.setStroke(new BasicStroke(6));
            g2.setPaint(new Color(50, 50, 50));
            g2.setColor(new Color(50, 50, 50));

            double brakingDist = ((currVal.speed * currVal.speed) / (250 * 0.8)) * 1.5f;

            if(planAroundPedestrian) {

                int i = 0;
                for (Pedestrian p : pedestrians) {
                    double pedDistance = Math.max(0, Math.sqrt(Math.pow(currVal.x - p.x, 2) + Math.pow(currVal.z - p.z, 2)) * mapResolution - 5);

                    if (pedDistance <= 17 && pedDistance < brakingDist) {
                        double maxBelief = 0;

                        drawCenteredCircle(g2, (float) p.x, (float) p.z, 20);

                        for (int angleIndex = 0; angleIndex < currentBelief.get(i).size() - 1; ++angleIndex) {
                            double belief = currentBelief.get(i).get(angleIndex);
                            Vector2f direction = toAngle(angleIndex * BELIEF_ANGLE_DISCRETIZATION, belief * p.speed * 2);
                            g2.draw(new Line2D.Double(p.x, p.z, (p.x + direction.x), (p.z + direction.y)));
                            if (belief > maxBelief) {
                                maxBelief = belief;
                            }
                        }

                        if (maxBelief < 0.3) {
                            drawCenteredCircle(g2, (float) p.x, (float) p.z, 25);
                        }
                    }
                    ++i;
                }

            }
        }

        if (obstacle != null) {
            g2.setStroke(new BasicStroke(4));

            g2.setColor(Color.GRAY);
            List<float[]> f = planner.getCornerPositionsIncreased(obstacle.x, obstacle.z, obstacle.theta);
            g2.draw(new Line2D.Float(f.get(0)[0], f.get(0)[1], f.get(1)[0], f.get(1)[1]));
            g2.draw(new Line2D.Float(f.get(1)[0], f.get(1)[1], f.get(2)[0], f.get(2)[1]));
            g2.draw(new Line2D.Float(f.get(2)[0], f.get(2)[1], f.get(3)[0], f.get(3)[1]));
            g2.draw(new Line2D.Float(f.get(3)[0], f.get(3)[1], f.get(0)[0], f.get(0)[1]));

            g2.setColor(Color.DARK_GRAY);
            f = planner.getCornerPositions(obstacle.x, obstacle.z, obstacle.theta);
            g2.draw(new Line2D.Float(f.get(0)[0], f.get(0)[1], f.get(1)[0], f.get(1)[1]));
            g2.draw(new Line2D.Float(f.get(1)[0], f.get(1)[1], f.get(2)[0], f.get(2)[1]));
            g2.draw(new Line2D.Float(f.get(2)[0], f.get(2)[1], f.get(3)[0], f.get(3)[1]));
            g2.draw(new Line2D.Float(f.get(3)[0], f.get(3)[1], f.get(0)[0], f.get(0)[1]));
        }
    }

    public class Reward {
        double reward;
        boolean terminal;

        public Reward() {
            terminal = false;
        }
    }

    public void setSubscribedValues(SubscribedValues parsedValue) {
        valueLock.lock();
        this.parsedValue = parsedValue;
        valueSetCondition.signal();
        valueLock.unlock();
    }

    public double obstacleCost(double x, double z, double orientation) {
        int closestX = (int) Math.round(x);
        int closestZ = (int) Math.round(z);
        int lowestReward;
        try {
            lowestReward = costMap[closestZ][closestX];
        } catch (ArrayIndexOutOfBoundsException e) {
            lowestReward = 0;
        }

        for (float[] cornerPosition : HybridAStarPhysical.getAllCornerPositions((float) x, (float) z, (float) orientation)) {
            closestX = Math.round(cornerPosition[0]);
            closestZ = Math.round(cornerPosition[1]);

            int reward = 0;
            if (cornerPosition[0] >= 0 && cornerPosition[1] >= 0 && cornerPosition[0] < (304 / mapResolution) && cornerPosition[1] < (385 / mapResolution)) {
                reward = costMap[closestZ][closestX];
            }

            if (reward < lowestReward) {
                lowestReward = reward;
            }
        }

        return 255 - lowestReward;
    }

    public double[] observation(SubscribedValues subscribedValues, float targetSpeed) {
        double[] obs = new double[3 + (subscribedValues.pedestrians != null ? subscribedValues.pedestrians.size() * 2 : 0)];
        obs[0] = subscribedValues.x * mapResolution;
        obs[1] = subscribedValues.z * mapResolution;
        obs[2] = targetSpeed * 0.2777778;

        if (subscribedValues.pedestrians != null) {
            int i = 0;
            for (Pedestrian p : subscribedValues.pedestrians) {
                obs[3 + i * 2] = p.x * mapResolution;
                obs[3 + i * 2 + 1] = p.z * mapResolution;
                ++i;
            }

        }

        return obs;
    }

    private class CarController implements Runnable {
        void reset(){
            if (mode == TRAINING) {
                simulationSequence.evaluate("");

                int NUM_SETS = 16;

                if (rotateScenarios && runs >= maxRuns) {
                    runs = 0;
                    TRAINING_SET = (TRAINING_SET + 1) % (NUM_SETS);

                    changeGoal();
                    sendNextScenarioMessage(out, TRAINING_SET);
                } else {
                    ++runs;
                    changeGoal();
                    sendResetMessage(out);
                }
                resetProperties();
            } else if (mode == TESTING) {
                changeGoal();

                if(!pythonConnector.connected() && onlyEvaluateConnected){
                    sendSceneResetMessage(out);
                    resetProperties();
                    simulationSequence.reset();
                    return;
                }

                if (sceneTrials >= 0 && !simulationSequence.emptySequence()) {
                    if (type == ZIGZAG_FOLLOW) {
                        simulationSequence.evaluate("Length: " + zigZagLength + "\tAngle: " + zigZagAngle);
                    }
                    if (type == MULTIPLE_PEDESTRIANS) {
                        simulationSequence.evaluate("Dist: " + distanceToFirstPed + "\tDelay: " + delay);
                    }
                    if (type == SPEED_DISTANCE) {
                        simulationSequence.evaluate("Dist: " + distances.get(distanceIndex) + "\tSpeed: " + pedSpeeds.get(speedIndex));
                    }

                    if (sceneTrials == MAX_TRIALS) {
                        sceneTrials = 1;
                        sceneNumber++;

                        sendNextSceneMessage(out);
                        resetProperties();

                        if (type == ZIGZAG_FOLLOW) {
                            zigZagAngle += angleInterval;
                            if (zigZagAngle >= 180 - angleInterval) {
                                zigZagAngle = 30;
                                zigZagLength += lengthInterval;
                            }
                        }

                        if (type == MULTIPLE_PEDESTRIANS) {
                            if (delay >= maxDelay) {
                                delay = 0;
                                distanceToFirstPed += distanceToFirstPedInterval;
                            } else {
                                delay += delayInterval;
                            }
                        }
                        if (type == SPEED_DISTANCE) {
                            distanceIndex++;
                            if (distanceIndex >= distances.size()) {
                                distanceIndex = 0;
                                speedIndex++;
                            }
                        }
                    } else {
                        sceneTrials++;
                        sendSceneResetMessage(out);
                        resetProperties();
                    }
                }
                simulationSequence.reset();

                if (sceneTrials == -1) {
                    sceneTrials = 0;
                }
            }
        }

        @Override
        public void run() {
            byte[] msg;

            SubscribedValues thisValue = null;
            SubscribedValues lastState = null;

            while (true) {

                valueLock.lock();
                while (parsedValue == null) {
                    try {
                        valueSetCondition.await();
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }
                thisValue = parsedValue;
                parsedValue = null;
                valueLock.unlock();

                if (resetSent) {
                    // we now are reset
                    float startX = -1;
                    float startZ = -1;

                    if (mode == TRAINING) {
                        startX = 1873;
                        startZ = 104;
                    }
                    if (mode == TESTING) {
                        if (TRAINING_SET >= 9 && TRAINING_SET <= 11 || TRAINING_SET == 16 || TRAINING_SET == 17 || TRAINING_SET == 21 || TRAINING_SET == 22) {
                            startX = 1828.5f;
                            startZ = 309;
                        } else if ((TRAINING_SET >= 12 && TRAINING_SET <= 13) || TRAINING_SET == 15) {
                            startX = 1607;
                            startZ = 967.4f;
                        } else if (TRAINING_SET == 14) {
                            startX = 1700.7f;
                            startZ = 650.4f;
                        } else if (TRAINING_SET == 18 || TRAINING_SET == 19){
                            startX = 1793.05f;
                            startZ = 407.48f;
                        } else if (TRAINING_SET == 20) {
                            startX = 1379.7f;
                            startZ = 1498.9f;
                        } else {
                            switch (TRAINING_SET % 3) {
                                case 0:
                                    startX = 1828.5f;
                                    startZ = 309;
                                    break;
                                case 1:
                                    startX = 1607;
                                    startZ = 967.4f;
                                    break;
                                case 2:
                                    startX = 0;
                                    startZ = 0;
                                    break;
                                default:
                                    throw new RuntimeException("Something is wrong here");

                            }
                        }
                    }

                    if (Math.sqrt(Math.pow(thisValue.x - startX, 2)
                            + Math.pow(thisValue.z - startZ, 2)) < (12 / mapResolution)) {
                        resetSent = false;
                    } else {
                        if (mode == TRAINING) {
                            resetSent = false;
                        } else if (mode == TESTING) {
                            sendSceneResetMessage(out);
                            resetProperties();
                            try {
                                Thread.sleep(5);
                            } catch (InterruptedException e) {
                                e.printStackTrace();
                            }
                            continue;
                        }
                    }
                }

                int angle = 0;
                try {
                    if (thisValue.x < 0 || thisValue.z < 0 || thisValue.x >= map_width || thisValue.z >= map_height) {
                        if (mode == TRAINING) {
                            sendResetMessage(out);
                        } else if (mode == TESTING) {
                            sendSceneResetMessage(out);
                        }
                        resetProperties();
                        continue;
                    }

                    ApproximatePositionEntry approxPos = new ApproximatePositionEntry((float) thisValue.x, (float) thisValue.z, (float) thisValue.orientation);
                    if (stuckPos != null && targetSpeed > 0 && stuckPos.equals(approxPos)) {
                        stuckSteps++;
                        if (stuckSteps >= STUCK_MAX) {
                            stuckSteps = 0;
                            stuckPos = approxPos;
                            if (mode == TRAINING) {
                                sendResetMessage(out);
                            } else if (mode == TESTING) {
                                sendSceneResetMessage(out);
                            }
                            resetProperties();
                            continue;
                        }
                    } else {
                        stuckPos = approxPos;
                        stuckSteps = 0;
                    }

                    double goalDist = Math.sqrt(Math.abs(Math.pow(thisValue.x - goalX, 2) + Math.pow(thisValue.z - goalZ, 2))) * mapResolution;

                    int[] tmp;
                    if (goalDist > R_GOAL + 1) {
                        tmp = searchSteeringAngle(thisValue);

                        if (tmp != null) {
                            angle = tmp[0];
                        } else {
                            if (mode == TRAINING) {
                                sendResetMessage(out);
                            } else if (mode == TESTING) {
                                sendSceneResetMessage(out);
                            }
                            resetProperties();
                            System.out.println("Could not plan path or received illegal position value.");
                            continue;
                        }
                    }

                    double converted_angle = (-angle / 57.0);
                    lastPlannedAngle = converted_angle;

                    Reward r;
                    if(pythonConnector.connected()){
                        r = calculateReward(thisValue, lastAction, lastState);
                    }else{
                        r = new Reward();
                    }

                    if (goalDist <= R_GOAL + 3) {
                        r.terminal = true;
                    }

                    double reward = (lastAcceleration != -1) ? r.reward : 0;
                    lastAcceleration = 0;

                    double[] observation = observation(thisValue, targetSpeed);
                    String cast_obs = Arrays.stream(observation)
                            .mapToObj(String::valueOf)
                            .collect(Collectors.joining(observationDelimiter));

                    String cast_map = null;
                    if(useCarIntention){
                        try {
                            cast_map = Arrays.stream(getMapExtract(thisValue))
                                    .mapToObj(String::valueOf)
                                    .collect(Collectors.joining(","));
                        } catch (ArrayIndexOutOfBoundsException e) {
                            e.printStackTrace();
                            if (mode == TRAINING) {
                                sendResetMessage(out);
                            } else if (mode == TESTING) {
                                sendSceneResetMessage(out);
                            }
                            resetProperties();
                            continue;
                        }
                    }

                    String convertedPathString = null;
                    if(usePath){
                        List<String> convertedPath = new LinkedList<>();

                        int pointsPerEntry = planner.getPathStep() / 2; // distance per step
                        int pathLength = currentPath.size();

                        for (int i = 0; i < pathLength - 1; ++i) {
                            HybridAStarPhysical.ContinuousSearchState current = currentPath.get(i);
                            HybridAStarPhysical.ContinuousSearchState next = currentPath.get(i + 1);

                            int parAngle = next.parentAngle;

                            List<Float> tmpPath = planner.simpleStep(current.x, current.z, current.theta, Arrays.asList(possibleSteeringAngles).indexOf(parAngle), 0.2f / mapResolution, pointsPerEntry);

                            for (Float f : tmpPath) {
                                convertedPath.add(String.valueOf(f));
                            }
                        }

                        HybridAStarPhysical.ContinuousSearchState last = currentPath.get(pathLength - 1);
                        convertedPath.add(String.valueOf(last.x * mapResolution));
                        convertedPath.add(String.valueOf(last.z * mapResolution));
                        convertedPath.add(String.valueOf(last.theta * mapResolution));

                        convertedPathString = String.join(",", convertedPath);
                    }

                    String totalMessage = r.terminal + ";" + reward + ";" + converted_angle + ";" + cast_obs;
                    if(usePath){
                        totalMessage += ";" + convertedPathString;
                    }
                    if(useCarIntention){
                        totalMessage += ";" + cast_map;
                    }
                    totalMessage += "\n";

                    if(r.terminal && (!pythonConnector.connected())){
                        reset();
                    }

                    if (r.terminal && (!terminalReached)) {
                        terminalReached = true;
                        terminalReward = r;
                        terminalMessage = totalMessage;
                    }

                    simulationSequence.add(thisValue.orientation, thisValue.x, thisValue.z, thisValue.speed, targetSpeed, thisValue.pedestrians);

                    if (startTick > 0) {
                        startTick--;
                        msg = buildMessage("drivingCar", (float) 0, 0f, 0f, targetSpeed).getBytes("UTF-8");
                        out.write(msg);
                        out.flush();
                    } else if (pythonConnector.connected()) {
                        if (terminalReached) {
                            pythonConnector.sendMessage(terminalMessage);
                        } else {
                            pythonConnector.sendMessage(totalMessage);
                        }

                        String answer = pythonConnector.receiveMessage();

                        switch (answer) {
                            case "START":
                                msg = buildMessage("drivingCar", (float) converted_angle, 0f, 0f, targetSpeed).getBytes("UTF-8");
                                out.write(msg);
                                break;
                            case "RESET":
                                reset();
                                break;
                            default:
                                AbstractAction a = getAction(answer, angle);

                                drawBelief(thisValue.pedestrians, thisValue);
                                pane.updateBelief(currentBelief);
                                lastAction = a;

                                AccelerationType accelerationAction = a.getAccelerationAction();

                                if ((targetSpeed == MAX_SPEED && accelerationAction == ACCELERATE) || (targetSpeed == 0 && accelerationAction == DECELLERATE)) {
                                    a.setAccelerationAction(MAINTAIN);
                                }

                                if (forceSpeedUp(thisValue, lastState)) {
                                    if (targetSpeed < MAX_SPEED) {
                                        targetSpeed += 5;
                                        a.setAccelerationAction(ACCELERATE);
                                    } else {
                                        a.setAccelerationAction(MAINTAIN);
                                    }
                                }else{
                                    if (accelerationAction == ACCELERATE) {
                                        targetSpeed += 5;
                                    } else if (accelerationAction == DECELLERATE) {
                                        targetSpeed -= 5;
                                    }
                                }

                                targetSpeed = Math.min(targetSpeed, MAX_SPEED);
                                targetSpeed = Math.max(targetSpeed, 0);

                                //System.out.println(a.acc + ", " + thisValue.speed + ", " + targetSpeed);

                                converted_angle = (-a.getSteeringAngle() / 57.0);

                                if ((Math.abs(thisValue.speed) < 2.5 && targetSpeed == 0)) {
                                    converted_angle = 0;
                                }

                                simulationSequence.addAction(a.getSteeringAngle(), a.getAccelerationAction().ordinal());

                                double speedDifference = Math.min(targetSpeed, MAX_SPEED) - thisValue.speed;
                                // we are basically correct speed. Don't do anything
                                if (Math.abs(speedDifference) < 0.5) {
                                    msg = buildMessage("drivingCar", (float) converted_angle, 0f, 0f, targetSpeed).getBytes("UTF-8");
                                } else if (speedDifference > 0) {
                                    double percentageOfSpeed = thisValue.speed / targetSpeed;
                                    if (percentageOfSpeed < 0.9) {
                                        msg = buildMessage("drivingCar", (float) converted_angle, 1f, 0f, targetSpeed).getBytes("UTF-8");
                                    } else {
                                        msg = buildMessage("drivingCar", (float) converted_angle, 0.1f, 0f, targetSpeed).getBytes("UTF-8");
                                    }
                                } else {
                                    double percentageOfSpeed = (thisValue.speed - targetSpeed) / targetSpeed;
                                    if (percentageOfSpeed < 0.1) {
                                        if (targetSpeed == 0) {
                                            msg = buildMessage("drivingCar", (float) converted_angle, 0f, 1f, targetSpeed).getBytes("UTF-8");
                                        } else {
                                            msg = buildMessage("drivingCar", (float) converted_angle, 0f, 0.2f, targetSpeed).getBytes("UTF-8");
                                        }
                                    } else {
                                        msg = buildMessage("drivingCar", (float) converted_angle, 0f, 1f, targetSpeed).getBytes("UTF-8");
                                    }
                                }
                                out.write(msg);
                                out.flush();
                        }
                    } else {
                        drawBelief(thisValue.pedestrians, thisValue);
                        AbstractAction a = chooseAction(thisValue, angle);

                        AccelerationType accelerationAction = a.getAccelerationAction();

                        if ((targetSpeed == MAX_SPEED && accelerationAction == ACCELERATE) || (targetSpeed == 0 && accelerationAction == DECELLERATE)) {
                            a.setAccelerationAction(MAINTAIN);
                        }

                        if (forceSpeedUp(thisValue, lastState)) {
                            if (targetSpeed < MAX_SPEED) {
                                targetSpeed += 5;
                                a.setAccelerationAction(ACCELERATE);
                            } else {
                                a.setAccelerationAction(MAINTAIN);
                            }
                        }else{
                            if (accelerationAction == ACCELERATE) {
                                targetSpeed += 5;
                            } else if (accelerationAction == DECELLERATE) {
                                targetSpeed -= 5;
                            }
                        }

                        targetSpeed = Math.min(targetSpeed, MAX_SPEED);
                        targetSpeed = Math.max(targetSpeed, 0);

                        //System.out.println(a.acc + ", " + thisValue.speed + ", " + targetSpeed);

                        converted_angle = (-a.getSteeringAngle() / 57.0);

                        if ((Math.abs(thisValue.speed) < 2.5 && targetSpeed == 0)) {
                            converted_angle = 0;
                        }

                        simulationSequence.addAction(a.getSteeringAngle(), a.getAccelerationAction().ordinal());

                        double speedDifference = Math.min(targetSpeed, MAX_SPEED) - thisValue.speed;
                        // we are basically correct speed. Don't do anything
                        if (Math.abs(speedDifference) < 0.5) {
                            msg = buildMessage("drivingCar", (float) converted_angle, 0f, 0f, targetSpeed).getBytes("UTF-8");
                        } else if (speedDifference > 0) {
                            double percentageOfSpeed = thisValue.speed / targetSpeed;
                            if (percentageOfSpeed < 0.9) {
                                msg = buildMessage("drivingCar", (float) converted_angle, 1f, 0f, targetSpeed).getBytes("UTF-8");
                            } else {
                                msg = buildMessage("drivingCar", (float) converted_angle, 0.1f, 0f, targetSpeed).getBytes("UTF-8");
                            }
                        } else {
                            double percentageOfSpeed = (thisValue.speed - targetSpeed) / targetSpeed;
                            if (percentageOfSpeed < 0.1) {
                                if (targetSpeed == 0) {
                                    msg = buildMessage("drivingCar", (float) converted_angle, 0f, 1f, targetSpeed).getBytes("UTF-8");
                                } else {
                                    msg = buildMessage("drivingCar", (float) converted_angle, 0f, 0.2f, targetSpeed).getBytes("UTF-8");
                                }
                            } else {
                                msg = buildMessage("drivingCar", (float) converted_angle, 0f, 1f, targetSpeed).getBytes("UTF-8");
                            }
                        }
                        out.write(msg);
                        out.flush();
                    }

                } catch (UnsupportedEncodingException e) {
                    e.printStackTrace();
                    return;
                } catch (IOException e) {
                    resetConnection(true);
                }

                lastState = thisValue;
            }
        }
    }
}
