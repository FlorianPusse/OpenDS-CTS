/*
 *  This file is part of OpenDS (Open Source Driving Simulator).
 *  Copyright (C) 2016 Rafael Math
 *
 *  OpenDS is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  OpenDS is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with OpenDS. If not, see <http://www.gnu.org/licenses/>.
 */


package eu.opends.main;

import com.jme3.app.StatsAppState;
import com.jme3.bounding.BoundingBox;
import com.jme3.input.Joystick;
import com.jme3.math.Line;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.niftygui.NiftyJmeDisplay;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.scene.Spatial.CullHint;
import com.jme3.system.AppSettings;
import com.jme3.system.JmeContext.Type;
import com.sun.javafx.application.PlatformImpl;
import de.lessvoid.nifty.Nifty;
import eu.opends.analyzer.DataWriter;
import eu.opends.analyzer.DrivingTaskLogger;
import eu.opends.audio.AudioCenter;
import eu.opends.basics.InternalMapProcessing;
import eu.opends.basics.SimulationBasics;
import eu.opends.camera.SimulatorCam;
import eu.opends.cameraFlight.CameraFlight;
import eu.opends.cameraFlight.NotEnoughWaypointsException;
import eu.opends.canbus.CANClient;
import eu.opends.car.ResetPosition;
import eu.opends.car.SteeringCar;
import eu.opends.chrono.ChronoPhysicsSpace;
import eu.opends.drivingTask.DrivingTask;
import eu.opends.drivingTask.settings.SettingsLoader.Setting;
import eu.opends.effects.EffectCenter;
import eu.opends.environment.TrafficLightCenter;
import eu.opends.eyetracker.EyetrackerCenter;
import eu.opends.hmi.HMICenter;
import eu.opends.infrastructure.RoadNetwork;
import eu.opends.infrastructure.Segment;
import eu.opends.infrastructure.Waypoint;
import eu.opends.input.ForceFeedbackJoystickController;
import eu.opends.input.KeyBindingCenter;
import eu.opends.knowledgeBase.KnowledgeBase;
import eu.opends.multiDriver.MultiDriverClient;
import eu.opends.niftyGui.DrivingTaskSelectionGUIController;
import eu.opends.profiler.BasicProfilerState;
import eu.opends.reactionCenter.ReactionCenter;
import eu.opends.settingsController.SettingsControllerServer;
import eu.opends.taskDescription.contreTask.SteeringTask;
import eu.opends.taskDescription.tvpTask.MotorwayTask;
import eu.opends.taskDescription.tvpTask.ThreeVehiclePlatoonTask;
import eu.opends.tools.*;
import eu.opends.traffic.Pedestrian;
import eu.opends.traffic.PhysicalTraffic;
import eu.opends.traffic.TrafficObject;
import eu.opends.trigger.TriggerCenter;
import eu.opends.visualization.LightningClient;
import eu.opends.visualization.MoviePlayer;
import org.apache.log4j.Logger;
import org.apache.log4j.PropertyConfigurator;
import settingscontroller_client.src.Evaluation.Config;
import settingscontroller_client.src.Evaluation.ScenarioConfig;

import java.awt.geom.Point2D;
import java.io.File;
import java.io.IOException;
import java.nio.file.*;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.*;

import static eu.opends.main.DataSet.MULTIPLE_PEDESTRIAN_SETS;
import static eu.opends.main.DataSet.NUM_SETS;
import static java.lang.Math.*;
import static settingscontroller_client.src.Evaluation.Config.ExperimentType.MULTIPLE_PEDESTRIANS;
import static settingscontroller_client.src.Evaluation.Config.ExperimentType.SPEED_DISTANCE;
import static settingscontroller_client.src.Evaluation.Config.ExperimentType.ZIGZAG_FOLLOW;
import static settingscontroller_client.src.Evaluation.Config.SimulationMode.TESTING;
import static settingscontroller_client.src.Evaluation.Config.sampleDistance;
import static settingscontroller_client.src.Evaluation.Config.sampleMarginSpeeds;
import static settingscontroller_client.src.Evaluation.ScenarioConfig.ObstaclePositions.NONE;
import static settingscontroller_client.src.Evaluation.ScenarioConfig.ObstaclePositions.ON_PAVEMENT;
import static settingscontroller_client.src.Evaluation.ScenarioConfig.ObstaclePositions.ON_STREET;

/**
 * @author Rafael Math
 */
public class Simulator extends SimulationBasics {

    /* Definition of scenario and mode */

    /**
     * Simulation mode to run. Needs to be in sync with value set in controller.
     */
    public Config.SimulationMode mode = TESTING;

    /**
     * Experiment type to run. Needs to be in sync with value set in controller.
     */
    public Config.ExperimentType type = MULTIPLE_PEDESTRIANS;

    /**
     * Scenario to chose. Needs to be in sync with value set in controller.
     */
    public int TRAINING_SET = ScenarioConfig.toTrainingSetMultiplePedestrian(false);//ScenarioConfig.toTrainingSet(ScenarioConfig.PedestrianSide.RIGHT, ScenarioConfig.Intersection.NONE, false);

    /**
     * Position of the obstacle.
     */
    public ScenarioConfig.ObstaclePositions obstaclePosition = NONE;

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
     * Whether pedstrian is currently crossing
     */
    public boolean isCrossing = false;

    /**
     * Prevent pedestrian from crossing based on proximity
     */
    boolean doNotCross = false;


    float crossingDistance = 50;

    /* Only normal OpenDS-related things starting from here */

    public static Float getGravityConstant() {
        return gravityConstant;
    }

    private Nifty nifty;
    private boolean drivingTaskGiven = false;
    private boolean initializationFinished = false;
    private static Float gravityConstant;

    public List<ResetPosition> trainingResetPoisitions = new LinkedList<>();
    public static boolean isHeadLess = false;
    private final static Logger logger = Logger.getLogger(Simulator.class);
    private int serverPort = -1;
    public String drivingTaskFileName;

    private RoadNetwork roadNetwork;

    public RoadNetwork getRoadNetwork() {
        return roadNetwork;
    }

    private SteeringCar car;

    public SteeringCar getCar() {
        return car;
    }

    private static DrivingTaskLogger drivingTaskLogger;

    public static DrivingTaskLogger getDrivingTaskLogger() {
        return drivingTaskLogger;
    }

    private boolean dataWriterQuittable = false;
    private DataWriter dataWriter;

    public DataWriter getMyDataWriter() {
        return dataWriter;
    }

    private LightningClient lightningClient;

    public LightningClient getLightningClient() {
        return lightningClient;
    }

    private static CANClient canClient;

    public static CANClient getCanClient() {
        return canClient;
    }

    private MultiDriverClient multiDriverClient;

    public MultiDriverClient getMultiDriverClient() {
        return multiDriverClient;
    }

    private TriggerCenter triggerCenter = new TriggerCenter(this);

    public TriggerCenter getTriggerCenter() {
        return triggerCenter;
    }

    private boolean showStats = false;

    public void showStats(boolean show) {
        showStats = show;
        setDisplayFps(show);
        setDisplayStatView(show);

        if (show)
            getCoordinateSystem().setCullHint(CullHint.Dynamic);
        else
            getCoordinateSystem().setCullHint(CullHint.Always);
    }

    public void toggleStats() {
        showStats = !showStats;
        showStats(showStats);
    }

    private CameraFlight cameraFlight;

    public CameraFlight getCameraFlight() {
        return cameraFlight;
    }

    private SteeringTask steeringTask;

    public SteeringTask getSteeringTask() {
        return steeringTask;
    }

    private ThreeVehiclePlatoonTask threeVehiclePlatoonTask;

    public ThreeVehiclePlatoonTask getThreeVehiclePlatoonTask() {
        return threeVehiclePlatoonTask;
    }

    private MotorwayTask motorwayTask;

    public MotorwayTask getMotorwayTask() {
        return motorwayTask;
    }

    private MoviePlayer moviePlayer;

    public MoviePlayer getMoviePlayer() {
        return moviePlayer;
    }

    private ReactionCenter reactionCenter;

    public ReactionCenter getReactionCenter() {
        return reactionCenter;
    }

    private EffectCenter effectCenter;

    public EffectCenter getEffectCenter() {
        return effectCenter;
    }

    private ObjectManipulationCenter objectManipulationCenter;

    public ObjectManipulationCenter getObjectManipulationCenter() {
        return objectManipulationCenter;
    }

    private String instructionScreenID = null;

    public void setInstructionScreen(String ID) {
        instructionScreenID = ID;
    }

    private SettingsControllerServer settingsControllerServer;

    public SettingsControllerServer getSettingsControllerServer() {
        return settingsControllerServer;
    }

    private EyetrackerCenter eyetrackerCenter;

    public EyetrackerCenter getEyetrackerCenter() {
        return eyetrackerCenter;
    }

    public static double minX = 0;
    public static double minZ = 0;
    public static double maxX = 0;
    public static double maxZ = 0;

    //public static final double stepsize = 0.1;
    public static final double stepsize = 0.1;

    private static String outputFolder;

    public static String getOutputFolder() {
        return outputFolder;
    }

    public static boolean oculusRiftAttached = false;

    private ForceFeedbackJoystickController joystickSpringController;

    public ForceFeedbackJoystickController getJoystickSpringController() {
        return joystickSpringController;
    }

    private ChronoPhysicsSpace chronoPhysicsSpace;

    public ChronoPhysicsSpace getChronoPhysicsSpace() {
        return chronoPhysicsSpace;
    }

    @Override
    public void simpleInitApp() {
        showStats(false);

        if (drivingTaskGiven)
            simpleInitDrivingTask(this.drivingTaskFileName, SimulationDefaults.driverName);
        else
            initDrivingTaskSelectionGUI();
    }


    private void initDrivingTaskSelectionGUI() {
        NiftyJmeDisplay niftyDisplay = new NiftyJmeDisplay(assetManager, inputManager, audioRenderer, guiViewPort);

        // Create a new NiftyGUI object
        nifty = niftyDisplay.getNifty();

        String xmlPath = "Interface/DrivingTaskSelectionGUI.xml";

        // Read XML and initialize custom ScreenController
        nifty.fromXml(xmlPath, "start", new DrivingTaskSelectionGUIController(this, nifty));

        // attach the Nifty display to the gui view port as a processor
        guiViewPort.addProcessor(niftyDisplay);

        // disable fly cam
        flyCam.setEnabled(false);
    }


    public void closeDrivingTaskSelectionGUI() {
        nifty.exit();
        inputManager.setCursorVisible(false);
        flyCam.setEnabled(true);
    }


    /**
     * Switches the scenario to simulate
     * @param initDrivingTask
     */
    public void switchScenario(boolean initDrivingTask) {
        trainingResetPoisitions.clear();

        if (mode == TESTING) {
            this.drivingTaskFileName = DataSet.paths[TRAINING_SET];

            Quaternion rotation;
            Point2D.Float[] positions;

            if (TRAINING_SET >= 9 && TRAINING_SET <= 11) {
                rotation = DataSet.startingOrientation0;
                positions = DataSet.startingPos0;
            } else if ((TRAINING_SET >= 12 && TRAINING_SET <= 13) || TRAINING_SET == 15) {
                rotation = DataSet.startingOrientation1;
                positions = DataSet.startingPos1;
            } else if (TRAINING_SET == 14) {
                rotation = DataSet.startingOrientation3;
                positions = DataSet.startingPos3;
            } else if(TRAINING_SET == 16){
                rotation = DataSet.startingOrientation0;
                positions = DataSet.startingPos0;
            } else if(TRAINING_SET == 17){
                rotation = DataSet.startingOrientation0;
                positions = DataSet.startingPos0;
            } else if (TRAINING_SET == 18 || TRAINING_SET == 19){
                rotation = DataSet.startingOrientation4;
                positions = DataSet.startingPos4;
            } else if (TRAINING_SET == 20){
                rotation = DataSet.startingOrientation5;
                positions = DataSet.startingPos5;
            }else if (TRAINING_SET == 21){
                rotation = DataSet.startingOrientation0;
                positions = DataSet.startingPos0;
            } else if (TRAINING_SET == 22){
            rotation = DataSet.startingOrientation0;
            positions = DataSet.startingPos0;
            } else {
                if (TRAINING_SET % 3 == 0) {
                    rotation = DataSet.startingOrientation0;
                    positions = DataSet.startingPos0;
                } else if (TRAINING_SET % 3 == 1) {
                    rotation = DataSet.startingOrientation1;
                    positions = DataSet.startingPos1;
                } else {
                    rotation = DataSet.startingOrientation2;
                    positions = DataSet.startingPos2;
                }
            }

            Vector3f startPos = new Vector3f(positions[0].x, -0.4f, positions[0].y);

            ResetPosition rp = new ResetPosition(startPos, rotation);
            trainingResetPoisitions.add(rp);
        }

        if (mode == Config.SimulationMode.TRAINING) {
            this.drivingTaskFileName = DataSet.paths[TRAINING_SET % NUM_SETS];

            Quaternion rotation;
            Point2D.Float[] positions;

            if (TRAINING_SET >= 9 && TRAINING_SET <= 11) {
                rotation = DataSet.startingOrientation0;
                positions = DataSet.startingPos0;
            } else if ((TRAINING_SET >= 12 && TRAINING_SET <= 13) || TRAINING_SET == 15) {
                rotation = DataSet.startingOrientation1;
                positions = DataSet.startingPos1;
            } else if (TRAINING_SET == 14) {
                rotation = DataSet.startingOrientation3;
                positions = DataSet.startingPos3;
            } else if(TRAINING_SET == 16){
                rotation = DataSet.startingOrientation0;
                positions = DataSet.startingPos0;
            } else if(TRAINING_SET == 17){
                rotation = DataSet.startingOrientation0;
                positions = DataSet.startingPos0;
            } else {
                if (TRAINING_SET % 3 == 0) {
                    rotation = DataSet.startingOrientation0;
                    positions = DataSet.startingPos0;
                } else if (TRAINING_SET % 3 == 1) {
                    rotation = DataSet.startingOrientation1;
                    positions = DataSet.startingPos1;
                } else {
                    rotation = DataSet.startingOrientation2;
                    positions = DataSet.startingPos2;
                }
            }

            if (positions.length > 1) {
                Vector3f startPos = new Vector3f(positions[0].x, -0.4f, positions[0].y);
                Vector3f endPos = new Vector3f(positions[1].x, -0.4f, positions[1].y);

                for (float i = 0; i <= 1.0; i += 0.05) {
                    Vector3f location = startPos.clone().interpolateLocal(endPos, i);
                    ResetPosition rp = new ResetPosition(location, rotation);
                    trainingResetPoisitions.add(rp);
                }
            } else {
                Vector3f startPos = new Vector3f(positions[0].x, -0.4f, positions[0].y);

                ResetPosition rp = new ResetPosition(startPos, rotation);
                trainingResetPoisitions.add(rp);
            }

        }

        if (mode == Config.SimulationMode.TRAINING) {
            if (TRAINING_SET == 9) {
                System.err.println("Training: Pedestrians coming from left");
                type = SPEED_DISTANCE;
                obstaclePosition = NONE;
            } else if (TRAINING_SET == 10) {
                System.err.println("Training: Multiple pedestrians");
                type = MULTIPLE_PEDESTRIANS;
                obstaclePosition = NONE;
            } else if (TRAINING_SET == 11) {
                System.err.println("Training: Occluded pedestrian, pavement");
                type = SPEED_DISTANCE;
                obstaclePosition = ON_PAVEMENT;
            } else if (TRAINING_SET == 12) {
                System.err.println("Training: Pedestrians coming from left");
                type = SPEED_DISTANCE;
                obstaclePosition = NONE;
            } else if (TRAINING_SET == 13) {
                System.err.println("Training: Pedestrians coming from left, occluded");
                type = SPEED_DISTANCE;
                obstaclePosition = ON_PAVEMENT;
            } else if (TRAINING_SET == 14) {
                System.err.println("Training: Car turning");
                type = SPEED_DISTANCE;
                obstaclePosition = NONE;
            } else if (TRAINING_SET == 15) {
                System.err.println("Training: Occluded pedestrian, street");
                type = SPEED_DISTANCE;
                obstaclePosition = ON_STREET;
            } else {
                System.err.println("Training: Normal training " + mode);
                type = SPEED_DISTANCE;
                obstaclePosition = NONE;
            }
        }

        if (initDrivingTask) {
            initDrivingTaskLayers();
            roadNetwork = new RoadNetwork(this);

            for (Spatial s : sceneNode.getChildren()) {
                if (s.getName() != null && s.getName().equals("Obstacle")) {
                    obstacle = s;
                    break;
                }
            }
        }
    }

    public void simpleInitDrivingTask(String drivingTaskFileName, String driverName) {
        switchScenario(false);

        chronoPhysicsSpace = new ChronoPhysicsSpace();

        stateManager.attach(new BasicProfilerState(false));

        physicalTraffic = new PhysicalTraffic();

        Util.makeDirectory("analyzerData");
        outputFolder = "analyzerData/" + Util.getDateTimeString();

        initDrivingTaskLayers();

        // show stats if set in driving task
        showStats(settingsLoader.getSetting(Setting.General_showStats, false));


        oculusRiftAttached = false;

        // sets up physics, camera, light, shadows and sky
        super.simpleInitApp();

        // set gravity
        gravityConstant = drivingTask.getSceneLoader().getGravity(SimulationDefaults.gravity);
        getBulletPhysicsSpace().setGravity(new Vector3f(0, -gravityConstant, 0));

        if (!Simulator.isHeadLess)
            PanelCenter.init(this);

        Joystick[] joysticks = inputManager.getJoysticks();
        if (joysticks != null)
            for (Joystick joy : joysticks)
                System.out.println("Connected joystick: " + joy.toString());

        //load map model
        new InternalMapProcessing(this);

        // start trafficLightCenter
        trafficLightCenter = new TrafficLightCenter(this);

        roadNetwork = new RoadNetwork(this);

        // create and place steering car
        car = new SteeringCar(this);

        // initialize physical vehicles
        physicalTraffic.init(this);

        // open TCP connection to KAPcom (knowledge component) [affects the driver name, see below]
        if (settingsLoader.getSetting(Setting.KnowledgeManager_enableConnection, SimulationDefaults.KnowledgeManager_enableConnection)) {
            String ip = settingsLoader.getSetting(Setting.KnowledgeManager_ip, SimulationDefaults.KnowledgeManager_ip);
            if (ip == null || ip.isEmpty())
                ip = "127.0.0.1";
            int port = settingsLoader.getSetting(Setting.KnowledgeManager_port, SimulationDefaults.KnowledgeManager_port);

            //KnowledgeBase.KB.setConnect(true);
            KnowledgeBase.KB.setCulture("en-US");
            KnowledgeBase.KB.Initialize(this, ip, port);
            KnowledgeBase.KB.start();
        }

        if (driverName == null || driverName.isEmpty())
            driverName = settingsLoader.getSetting(Setting.General_driverName, SimulationDefaults.driverName);
        SimulationDefaults.driverName = driverName;

        // setup key binding
        keyBindingCenter = new KeyBindingCenter(this);

        if (!isHeadLess) {
            AudioCenter.init(this);
        }

        // setup camera settings
        cameraFactory = new SimulatorCam(this, car);

        // init trigger center
        triggerCenter.setup();

        // init HMICenter
        HMICenter.init(this);


        // open TCP connection to Lightning
        if (settingsLoader.getSetting(Setting.ExternalVisualization_enableConnection, SimulationDefaults.Lightning_enableConnection)) {
            lightningClient = new LightningClient(this);
        }

        // open TCP connection to CAN-bus
        if (settingsLoader.getSetting(Setting.CANInterface_enableConnection, SimulationDefaults.CANInterface_enableConnection)) {
            canClient = new CANClient(this);
            canClient.start();
        }

        if (settingsLoader.getSetting(Setting.MultiDriver_enableConnection, SimulationDefaults.MultiDriver_enableConnection)) {
            multiDriverClient = new MultiDriverClient(this, driverName);
            multiDriverClient.start();
        }

        drivingTaskLogger = new DrivingTaskLogger(outputFolder, driverName, drivingTask.getFileName());

        SpeedControlCenter.init(this);

        try {

            // attach camera to camera flight
            cameraFlight = new CameraFlight(this);

        } catch (NotEnoughWaypointsException e) {

            // if not enough way points available, attach camera to driving car
            car.getCarNode().attachChild(cameraFactory.getMainCameraNode());
        }

        //reactionCenter = new ReactionCenter(this);

        //steeringTask = new SteeringTask(this, driverName);

        //threeVehiclePlatoonTask = new ThreeVehiclePlatoonTask(this, driverName);

        //motorwayTask = new MotorwayTask(this);

        //moviePlayer = new MoviePlayer(this);

        // start effect center
        effectCenter = new EffectCenter(this);

        objectManipulationCenter = new ObjectManipulationCenter(this);

        if (settingsLoader.getSetting(Setting.SettingsControllerServer_startServer, SimulationDefaults.SettingsControllerServer_startServer)) {
            if (serverPort != -1) {
                settingsControllerServer = new SettingsControllerServer(this, serverPort);
            } else {
                settingsControllerServer = new SettingsControllerServer(this);
            }
            settingsControllerServer.start();
        }

        StatsAppState statsAppState = stateManager.getState(StatsAppState.class);
        if (statsAppState != null && statsAppState.getFpsText() != null && statsAppState.getStatsView() != null) {
            statsAppState.getFpsText().setLocalTranslation(3, getSettings().getHeight() - 145, 0);
            statsAppState.getStatsView().setLocalTranslation(3, getSettings().getHeight() - 145, 0);
            statsAppState.setDarkenBehind(false);
        }

        // add physics collision listener
        CollisionListener collisionListener = new CollisionListener();
        getBulletPhysicsSpace().addCollisionListener(collisionListener);

        String videoPath = settingsLoader.getSetting(Setting.General_captureVideo, "");
        if ((videoPath != null) && (!videoPath.isEmpty()) && (Util.isValidFilename(videoPath))) {
            System.err.println("videoPath: " + videoPath);
            File videoFile = new File(videoPath);
            stateManager.attach(new VideoRecorderAppState(videoFile, this));
        }

        if (settingsLoader.getSetting(Setting.Eyetracker_enableConnection, SimulationDefaults.Eyetracker_enableConnection)) {
            eyetrackerCenter = new EyetrackerCenter(this);
        }

        joystickSpringController = new ForceFeedbackJoystickController(this);


        /*
         * STUFF
         */


        Node cityNode = (Node) getSceneNode().getChild(4);
        Node tmpCityNode = (Node) cityNode.getChild(0);

        BoundingBox worldBound = (BoundingBox) cityNode.getWorldBound();
        Vector3f mapCenter = worldBound.getCenter();
        Vector3f extents = worldBound.getExtent(null);

        float min_x = mapCenter.getX() - extents.getX(), max_x = mapCenter.getX() + extents.getX();
        float min_z = mapCenter.getZ() - extents.getZ(), max_z = mapCenter.getZ() + extents.getZ();

        minX = min_x;
        minZ = min_z;
        maxX = max_x;
        maxZ = max_z;


		/*
		if(true) {
			System.out.println(minX);
			System.out.println(minZ);
			System.out.println(maxX);
			System.out.println(maxZ);

			int x_entries = (int) Math.floor((2 * extents.getX()) / stepsize) + 1;
			int z_entries = (int) Math.floor((2 * extents.getZ()) / stepsize) + 1;

			System.out.println(x_entries);
			System.out.println(z_entries);

			short[][] values = new short[x_entries][z_entries];
			int i = 0, j = 0;

			for (float x_iter = min_x; x_iter < max_x; x_iter += stepsize) {
				for (float z_iter = min_z; z_iter < max_z; z_iter += stepsize) {
					BoundingBox bb = new BoundingBox(new Vector3f(x_iter, 0f, z_iter), (float) stepsize/2 - 0.001f, 0.8f, (float) stepsize/2 - 0.001f);

					for (Spatial tmpSpatial : tmpCityNode.getChildren()) {
						Node object = (Node) tmpSpatial;
						BoundingBox objectBox = (BoundingBox) object.getWorldBound();

						String name = object.getName();
						List<String> specialNames = Arrays.asList("Strasse","Asphalt","Crosswalk","Zebra","Einbuchtung","Bordstein","Gras","sub06_MeshPart0");
						50boolean foundSpecialName = false;
						for(String s : specialNames){
							if(name.contains(s)){
								foundSpecialName = true;
								break;
							}
						}

						boolean foundAny = false;

						final float[] xDirections = new float[]{0,0.05f,0.05f,0.05f,0,-0.05f,-0.05f,-0.05f,0};
						final float[] zDirections = new float[]{-0.05f,-0.05f,0,0.05f,0.05f,0.05f,0,-0.05f,0};

						for(int it = 0; it < xDirections.length; ++it){
							if(foundSpecialName || objectBox.contains(new Vector3f(x_iter,-0.5f, z_iter + zDirections[it]))
									|| objectBox.contains(new Vector3f(x_iter + xDirections[it],0.01f, z_iter + zDirections[it]))
									|| objectBox.contains(new Vector3f(x_iter + xDirections[it],0.3f, z_iter + zDirections[it]))
									|| objectBox.contains(new Vector3f(x_iter + xDirections[it],1f, z_iter + zDirections[it]))
									|| objectBox.contains(new Vector3f(x_iter + xDirections[it],0.7f, z_iter + zDirections[it]))){
								foundAny = true;
								break;
							}
						}

						if(!foundAny){
							continue;
						}


						CollisionResults res = new CollisionResults();
						object.collideWith(bb, res);
						if (res.size() > 0) {
							if (name.contains("Strasse") || name.contains("Asphalt") || name.contains("Crosswalk") || name.contains("Zebra")) {
								values[i][j] = values[i][j] == 0 ? 255 : (short) Math.min(values[i][j], 255);
							}
							else if (name.contains("Einbuchtung")) {
								values[i][j] = values[i][j] == 0 ? 250 : (short) Math.min(values[i][j], 250);
							}else if (name.contains("Cylinder15") || name.contains("Bordstein")) {
								values[i][j] = values[i][j] == 0 ? 200 : (short) Math.min(values[i][j], 200);
							} else if (name.contains("Object027")) {
								values[i][j] = values[i][j] == 0 ? 255 : (short) Math.min(values[i][j], 255);
							} else if (name.contains("Lawn") || name.contains("Gras") || name.contains("sub06_MeshPart0") || name.contains("Boden") || name.contains("Zuweg") || name.contains("Sand")) {
								values[i][j] = values[i][j] == 0 ? 100 : (short) Math.min(values[i][j], 100);
							} else{
								values[i][j] = values[i][j] == 0 ? 1 : (short) Math.min(values[i][j], 1);
							}
						}
					}
					++j;
				}
				++i;
				j = 0;
				if(i % 20 == 0){
					System.out.println(i);
				}
			}

			System.out.println("Map created.");

			File f = new File("LearningAssets/test1");
			try {
				BufferedWriter bw = new BufferedWriter(new FileWriter(f));
				bw.write("[");
				for (i = 0; i < values.length; ++i) {
					for (j = 0; j < values[i].length; ++j) {
						int v = values[i][j];
						if (j + 1 < values[i].length) {
							bw.write(String.valueOf(v) + ',');
						} else {
							bw.write(String.valueOf(v) + '\n');
						}
					}
				}
				bw.write("]");
				bw.flush();
			} catch (IOException e) {
				e.printStackTrace();
			}

		}*/

        for (Spatial s : sceneNode.getChildren()) {
            if (s.getName() != null && s.getName().equals("Obstacle")) {
                obstacle = s;
                break;
            }
        }

        if (mode == Config.SimulationMode.TRAINING) {
            car.setToRandomResetPosition(false);
        }

        if (mode == TESTING) {
            Vector3f location = trainingResetPoisitions.get(0).getLocation();
            Quaternion rotation = trainingResetPoisitions.get(0).getRotation();

            car.setPosition(location);
            car.carControl.setPhysicsRotation(rotation);
            car.carControl.resetVelocity();
        }

        initializationFinished = true;
    }

    public Spatial obstacle = null;

    /**
     * Stores the points where the pedestrian starts.
     * @param drivingTask
     */
    void setBackupPoint(DrivingTask drivingTask) {
        sceneLoader = drivingTask.getSceneLoader();
        scenarioLoader = drivingTask.getScenarioLoader();
        sceneLoader = drivingTask.getSceneLoader();

        HashMap<String, Waypoint> waypointMap = scenarioLoader.getWaypointMap();
        Waypoint startCrossWaypoint = waypointMap.get("WP_StartCross");
        Waypoint endCrossWaypoint = waypointMap.get("WP_EndCross");
        Waypoint startWaypoint = waypointMap.get("WP_Start");

        if (startCrossWaypoint != null && endCrossWaypoint != null && startWaypoint != null) {
            originalCrossStart = startCrossWaypoint.getPosition().clone();
            originalCrossEnd = endCrossWaypoint.getPosition().clone();
            originalStart = startWaypoint.getPosition().clone();
        }

        Waypoint startCrossWaypoint2 = waypointMap.get("WP_StartCross2");
        Waypoint endCrossWaypoint2 = waypointMap.get("WP_EndCross2");
        Waypoint startWaypoint2 = waypointMap.get("WP_Start2");

        if (startCrossWaypoint2 != null && endCrossWaypoint2 != null && startWaypoint2 != null) {
            originalCrossStart2 = startCrossWaypoint2.getPosition().clone();
            originalCrossEnd2 = endCrossWaypoint2.getPosition().clone();
            originalStart2 = startWaypoint2.getPosition().clone();
        } else {
            originalCrossStart2 = null;
            originalCrossEnd2 = null;
            originalStart2 = null;
        }
    }

    private void initDrivingTaskLayers() {
        File drivingTaskFile = new File(drivingTaskFileName);
        drivingTask = new DrivingTask(this, drivingTaskFile);

        setBackupPoint(drivingTask);

        interactionLoader = drivingTask.getInteractionLoader();
        settingsLoader = drivingTask.getSettingsLoader();
    }


    /**
     * That method is going to be executed, when the dataWriter is
     * <code>null</code> and the S-key is pressed.
     *
     * @param trackNumber Number of track (will be written to the log file).
     */
    public void initializeDataWriter(int trackNumber) {
        dataWriter = new DataWriter(outputFolder, car, SimulationDefaults.driverName,
                this.drivingTaskFileName, trackNumber);
    }

    Vector3f originalStart;
    Vector3f originalCrossStart;
    Vector3f originalCrossEnd;

    Vector3f originalStart2 = null;
    Vector3f originalCrossStart2 = null;
    Vector3f originalCrossEnd2 = null;

    /**
     * Creates the ZigZags of the ZigZag experiment
     */
    public void makeZigZag() {
        double angle = 90 - (zigZagAngle / 2);
        float length = (float) (zigZagLength / 2.0f);
        double lengthPerAngle = 2 * (sin(toRadians(zigZagAngle / 2.0)) * length);

        doNotCross = true;

        drivingTask = new DrivingTask(this, drivingTask.drivingTaskFileName, drivingTask.drivingTaskPath, drivingTask.dtData);

        scenarioLoader = drivingTask.getScenarioLoader();
        HashMap<String, Waypoint> waypointMap = scenarioLoader.getWaypointMap();
        Waypoint startCrossWaypointTmp = waypointMap.get("WP_StartCross");
        startCrossWaypointTmp.setPosition(originalCrossStart.clone());
        Waypoint endCrossWaypointTmp = waypointMap.get("WP_EndCross");
        endCrossWaypointTmp.setPosition(originalCrossEnd.clone());
        Waypoint startWaypointTmp = waypointMap.get("WP_Start");
        startWaypointTmp.setPosition(originalStart.clone());

        Map<String, Segment> segmentMap = scenarioLoader.getSegmentMap();

        Vector3f direction = startCrossWaypointTmp.getPosition().subtract(startWaypointTmp.getPosition());

        float walkingDistance = direction.length();

        direction = direction.normalize().mult(length);

        Vector3f newDirection = rotateVector(direction, angle);

        Segment newSegment = new Segment("newSegment", startWaypointTmp.getName(), new ArrayList<>(),
                "newWP_0", null,
                null, 50, false,
                0.5f, new ArrayList<>(), 0);
        segmentMap.put(newSegment.getName(), newSegment);

        startWaypointTmp.outgoingSegmentList = new ArrayList<>(Arrays.asList(newSegment));
        waypointMap.put(startWaypointTmp.getName(), startWaypointTmp);

        Vector3f newPos = startWaypointTmp.getPosition().add(newDirection);

        int MAX_NEW_WP = (int) (walkingDistance / lengthPerAngle);

        for (int i = 0; i < MAX_NEW_WP; ++i) {

            String nextWPName = (i + 1 < MAX_NEW_WP) ? ("newWP_" + (i + 1)) : startCrossWaypointTmp.getName();

            Segment newSegment1 = new Segment("newSegment_" + i, "newWP_" + i, new ArrayList<>(),
                    nextWPName, null,
                    null, 50, false,
                    0.5f, new ArrayList<>(), 0);
            segmentMap.put(newSegment1.getName(), newSegment1);

            Waypoint wp = new Waypoint("newWP_" + i, newPos,
                    null, null, null,
                    null, null, new ArrayList<>(Arrays.asList(newSegment1))
            );
            waypointMap.put(wp.getName(), wp);

            if (i % 2 == 0) {
                newPos = newPos.add(rotateVector(direction, -angle).mult(2));
            } else {
                newPos = newPos.add(rotateVector(direction, angle).mult(2));
            }
        }

        startCrossWaypointTmp.setPosition(newPos);

        roadNetwork = new RoadNetwork(this);
    }

    int obstacleDistance = 0;

    /**
     * Resets the current scene
     */
    public void resetScene() {
        // (-311.94266, -0.46226686, -24.452269)

        setPause(true);

        Vector3f location = trainingResetPoisitions.get(0).getLocation();
        Quaternion rotation = trainingResetPoisitions.get(0).getRotation();

        car.setPosition(location);
        car.carControl.setPhysicsRotation(rotation);
        car.carControl.resetVelocity();

        setPause(false);

        try {
            Thread.sleep(10);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        HashMap<String, Waypoint> waypointMap;

        reloadDrivingTask();

        scenarioLoader = drivingTask.getScenarioLoader();
        waypointMap = scenarioLoader.getWaypointMap();
        Waypoint startCrossWaypointTmp = waypointMap.get("WP_StartCross");
        startCrossWaypointTmp.setPosition(originalCrossStart.clone());
        Waypoint endCrossWaypointTmp = waypointMap.get("WP_EndCross");
        endCrossWaypointTmp.setPosition(originalCrossEnd.clone());
        Waypoint startWaypointTmp = waypointMap.get("WP_Start");
        startWaypointTmp.setPosition(originalStart.clone());

        if (type == SPEED_DISTANCE) {
            if (pedSpeeds.get(speedIndex) < 3.5) {
                Vector3f direction = startCrossWaypointTmp.getPosition().subtract(startWaypointTmp.getPosition());
                if(TRAINING_SET == 20){
                    direction = direction.normalize().mult(13);
                } else if(TRAINING_SET == 18 || TRAINING_SET == 19){
                    direction = direction.normalize().mult(15);
                }else{
                    direction = direction.normalize().mult(10);
                }
                startWaypointTmp.setPosition(startWaypointTmp.getPosition().add(direction));
            }
        }

        if (type == ZIGZAG_FOLLOW) {
            makeZigZag();
        }

        if (TRAINING_SET == 14 && type == SPEED_DISTANCE) {
            Vector3f directionOriginal = startCrossWaypointTmp.getPosition().subtract(startWaypointTmp.getPosition()).setY(0);
            directionOriginal = directionOriginal.mult(-1);

            float carAdvantageInSeconds = 6;
            float pedSpeed = pedSpeeds.get(speedIndex);
            float pedDistance = pedSpeed * carAdvantageInSeconds - distances.get(distanceIndex);

            if (pedDistance < 0) {
                pedDistance = 0;
            }

            Vector3f newPos = startCrossWaypointTmp.getPosition().add(directionOriginal.normalize().mult(pedDistance));
            startWaypointTmp.setPosition(newPos);
        }

        if (obstaclePosition != NONE && obstacle != null) {
            Vector3f directionOiriginal = startCrossWaypointTmp.getPosition().subtract(startWaypointTmp.getPosition()).setY(0);
            float dist = 5 + obstacleDistance;
            Vector3f direction = directionOiriginal.normalize().mult(dist);
            obstacle.setLocalTranslation(startWaypointTmp.getPosition().add(direction).setY(-0.5f));
            obstacle.setLocalRotation(DataSet.obstacleRotation);

            Pedestrian ped = (Pedestrian) getPhysicalTraffic().getTrafficObject("ped2");
            ped.followBox.settings.setMaxSpeed(0);
            ped.visible = false;

            Vector3f perpendicularDirection = getPerpendicularVector(direction.normalize());
            perpendicularDirection = perpendicularDirection.normalize().mult(0.3f);

            startWaypointTmp.setPosition(startWaypointTmp.getPosition().add(direction.normalize().mult(dist + 2.8f)).setY(-0.5f));
            startWaypointTmp.setPosition(startWaypointTmp.getPosition().add(perpendicularDirection));
            startCrossWaypointTmp.setPosition(startCrossWaypointTmp.getPosition().add(perpendicularDirection));

            if (obstaclePosition == ON_STREET) {
                perpendicularDirection = perpendicularDirection.normalize().mult(1.5f);

                if (TRAINING_SET == 12 || TRAINING_SET == 13 || TRAINING_SET == 14) {
                    perpendicularDirection = perpendicularDirection.normalize().mult(1.8f);
                    perpendicularDirection = perpendicularDirection.mult(-1);
                }

                obstacle.setLocalTranslation(obstacle.getLocalTranslation().add(perpendicularDirection));

                startWaypointTmp.setPosition(startWaypointTmp.getPosition().add(perpendicularDirection));
                startCrossWaypointTmp.setPosition(startCrossWaypointTmp.getPosition().add(perpendicularDirection));
            }

            obstacleDistance++;
            if (obstacleDistance > 15) {
                obstacleDistance = 0;
            }
        }

        if (type == MULTIPLE_PEDESTRIANS && originalStart2 != null && originalCrossEnd2 != null && originalCrossStart2 != null) {
            startCrossWaypointTmp = waypointMap.get("WP_StartCross2");
            startCrossWaypointTmp.setPosition(originalCrossStart2.clone());
            endCrossWaypointTmp = waypointMap.get("WP_EndCross2");
            endCrossWaypointTmp.setPosition(originalCrossEnd2.clone());
            startWaypointTmp = waypointMap.get("WP_Start2");
            startWaypointTmp.setPosition(originalStart2.clone());
        }

        if (type == MULTIPLE_PEDESTRIANS) {
            Pedestrian ped2 = (Pedestrian) getPhysicalTraffic().getTrafficObject("ped3");
            Vector3f direction = startCrossWaypointTmp.getPosition().subtract(startWaypointTmp.getPosition()).normalize();
            float offset = distanceToFirstPed - (delay * ped2.followBox.settings.getMaxSpeed() * 0.27778f);

            startWaypointTmp.setPosition(startWaypointTmp.getPosition().add(direction.mult(offset)));

            if (mode == Config.SimulationMode.TRAINING) {
                crossingDistance = 5 + rand.nextInt(20);
            }
        }

        roadNetwork = new RoadNetwork(this);

        for (TrafficObject obj : getPhysicalTraffic().getTrafficObjectList()) {
            if (obj instanceof Pedestrian) {
                Pedestrian ped = (Pedestrian) obj;
                ped.setToWayPoint(ped.followBox.startWayPoint.getName());

                if (obstaclePosition == NONE) {
                    if (type == ZIGZAG_FOLLOW) {
                        ped.followBox.CRASH_THRESHOLD = 0;
                    }
                    if (type == SPEED_DISTANCE) {
                        ((Pedestrian) getPhysicalTraffic().getTrafficObject("ped2")).followBox.settings.setMaxSpeed(pedSpeeds.get(speedIndex));
                    }
                } else {
                    ped.followBox.CRASH_THRESHOLD = 0;
                }

            }
        }

        isCrossing = false;
    }


    private static final DateFormat sdf = new SimpleDateFormat("HH:mm:ss");

    private int sceneNumber = 0;

    /**
     * Switches to the next scene
     */
    public void nextScene() {
        // (-311.94266, -0.46226686, -24.452269)

        setPause(true);

        Vector3f location = trainingResetPoisitions.get(0).getLocation();
        Quaternion rotation = trainingResetPoisitions.get(0).getRotation();

        car.setPosition(location);
        car.carControl.setPhysicsRotation(rotation);
        car.carControl.resetVelocity();

        setPause(false);

        if (type == ZIGZAG_FOLLOW) {
            zigZagAngle += angleInterval;
            if (zigZagAngle >= 180 - angleInterval) {
                zigZagAngle = 30;
                zigZagLength += lengthInterval;
            }

            if (mode == TESTING) {
                System.out.println("Changed pattern: Length: " + zigZagLength + ", Angle: " + zigZagAngle);
            }

            makeZigZag();
        }

        if (type == MULTIPLE_PEDESTRIANS) {
            if (delay >= maxDelay) {
                delay = 0;
                distanceToFirstPed += distanceToFirstPedInterval;
                if (distanceToFirstPed > 30) {
                    distanceToFirstPed = 0.5f;
                }
            } else {
                delay += delayInterval;
            }

            if (mode == TESTING) {
                System.out.println("Changed pattern: " + delay + ", " + distanceToFirstPed);
            } else {
                distanceToFirstPed = rand.nextInt(31);
                crossingDistance = rand.nextInt(25);
            }
        }

        if (type == SPEED_DISTANCE) {

            if (mode == TESTING) {
                distanceIndex++;
                if (distanceIndex >= distances.size()) {
                    distanceIndex = 0;
                    speedIndex++;
                    if (speedIndex >= pedSpeeds.size()) {
                        speedIndex = 0;
                    }
                }
            } else {
                distanceIndex = rand.nextInt(distances.size());
                speedIndex = rand.nextInt(pedSpeeds.size());
            }

            crossingDistance = distances.get(distanceIndex);
            ((Pedestrian) getPhysicalTraffic().getTrafficObject("ped2")).followBox.settings.setMaxSpeed(pedSpeeds.get(speedIndex));

            if (mode == TESTING) {
                System.out.println("Changed pattern: " + distances.get(distanceIndex) + ", " + pedSpeeds.get(speedIndex) + ", " + distanceIndex + ", " + speedIndex);
            }
        }

        if (type == MULTIPLE_PEDESTRIANS || type == SPEED_DISTANCE) {

            reloadDrivingTask();

            scenarioLoader = drivingTask.getScenarioLoader();
            HashMap<String, Waypoint> waypointMap = scenarioLoader.getWaypointMap();
            Waypoint startCrossWaypointTmp = waypointMap.get("WP_StartCross");
            startCrossWaypointTmp.setPosition(originalCrossStart.clone());
            Waypoint endCrossWaypointTmp = waypointMap.get("WP_EndCross");
            endCrossWaypointTmp.setPosition(originalCrossEnd.clone());
            Waypoint startWaypointTmp = waypointMap.get("WP_Start");
            startWaypointTmp.setPosition(originalStart.clone());

            if (type == SPEED_DISTANCE && pedSpeeds.get(speedIndex) < 3.5) {
                Vector3f direction = startCrossWaypointTmp.getPosition().subtract(startWaypointTmp.getPosition());
                if(TRAINING_SET == 20){
                    direction = direction.normalize().mult(13);
                } else if(TRAINING_SET == 18 || TRAINING_SET == 19){
                    direction = direction.normalize().mult(15);
                }else{
                    direction = direction.normalize().mult(10);
                }
                startWaypointTmp.setPosition(startWaypointTmp.getPosition().add(direction));
            }

            if (TRAINING_SET == 14 && type == SPEED_DISTANCE) {
                Vector3f directionOriginal = startCrossWaypointTmp.getPosition().subtract(startWaypointTmp.getPosition()).setY(0);
                directionOriginal = directionOriginal.mult(-1);

                float carAdvantageInSeconds = 6;
                float pedSpeed = pedSpeeds.get(speedIndex);
                float pedDistance = pedSpeed * carAdvantageInSeconds - distances.get(distanceIndex);

                if (pedDistance < 1) {
                    pedDistance = 1;
                }

                Vector3f newPos = startCrossWaypointTmp.getPosition().add(directionOriginal.normalize().mult(pedDistance));
                startWaypointTmp.setPosition(newPos);
            }

            if (obstaclePosition != NONE && obstacle != null) {
                if (mode == TESTING) {
                    obstacleDistance = 0;
                } else {
                    obstacleDistance = rand.nextInt(17);
                }


                Vector3f directionOiriginal = startCrossWaypointTmp.getPosition().subtract(startWaypointTmp.getPosition()).setY(0);
                float dist = 5 + obstacleDistance;
                Vector3f direction = directionOiriginal.normalize().mult(dist);
                obstacle.setLocalTranslation(startWaypointTmp.getPosition().add(direction).setY(-0.5f));
                obstacle.setLocalRotation(DataSet.obstacleRotation);

                Pedestrian ped = (Pedestrian) getPhysicalTraffic().getTrafficObject("ped2");
                ped.followBox.settings.setMaxSpeed(0);
                ped.visible = false;

                Vector3f perpendicularDirection = getPerpendicularVector(direction.normalize());
                perpendicularDirection = perpendicularDirection.normalize().mult(0.3f);

                startWaypointTmp.setPosition(startWaypointTmp.getPosition().add(direction.normalize().mult(dist + 2.8f)).setY(-0.5f));
                startWaypointTmp.setPosition(startWaypointTmp.getPosition().add(perpendicularDirection));
                startCrossWaypointTmp.setPosition(startCrossWaypointTmp.getPosition().add(perpendicularDirection));

                if (obstaclePosition == ON_STREET) {
                    perpendicularDirection = perpendicularDirection.normalize().mult(1.5f);

                    if (TRAINING_SET == 12 || TRAINING_SET == 13 || TRAINING_SET == 14) {
                        perpendicularDirection = perpendicularDirection.normalize().mult(1.8f);
                        perpendicularDirection = perpendicularDirection.mult(-1);
                    }

                    obstacle.setLocalTranslation(obstacle.getLocalTranslation().add(perpendicularDirection));

                    startWaypointTmp.setPosition(startWaypointTmp.getPosition().add(perpendicularDirection));
                    startCrossWaypointTmp.setPosition(startCrossWaypointTmp.getPosition().add(perpendicularDirection));
                }
            }

            if (type == MULTIPLE_PEDESTRIANS && originalStart2 != null && originalCrossEnd2 != null && originalCrossStart2 != null) {
                startCrossWaypointTmp = waypointMap.get("WP_StartCross2");
                startCrossWaypointTmp.setPosition(originalCrossStart2.clone());
                endCrossWaypointTmp = waypointMap.get("WP_EndCross2");
                endCrossWaypointTmp.setPosition(originalCrossEnd2.clone());
                startWaypointTmp = waypointMap.get("WP_Start2");
                startWaypointTmp.setPosition(originalStart2.clone());
            }

            if (type == MULTIPLE_PEDESTRIANS) {
                float speed;
                if(mode == Config.SimulationMode.TRAINING){
                    speed = sampleSpeed();
                }else{
                    speed = 6.5f;
                }


                ((Pedestrian) getPhysicalTraffic().getTrafficObject("ped2")).followBox.settings.setMaxSpeed(speed);
                ((Pedestrian) getPhysicalTraffic().getTrafficObject("ped3")).followBox.settings.setMaxSpeed(speed);

                Pedestrian ped2 = (Pedestrian) getPhysicalTraffic().getTrafficObject("ped3");
                Vector3f direction = startCrossWaypointTmp.getPosition().subtract(startWaypointTmp.getPosition()).normalize();
                float offset = distanceToFirstPed - (delay * ped2.followBox.settings.getMaxSpeed() * 0.27778f);
                startWaypointTmp.setPosition(startWaypointTmp.getPosition().add(direction.mult(offset)));
            }

            roadNetwork = new RoadNetwork(this);
        }

        isCrossing = false;

        sceneNumber++;

        for (TrafficObject obj : getPhysicalTraffic().getTrafficObjectList()) {
            if (obj instanceof Pedestrian) {
                Pedestrian ped = (Pedestrian) obj;
                ped.setToWayPoint(ped.followBox.startWayPoint.getName());

                if (obstaclePosition == NONE) {
                    if (type == ZIGZAG_FOLLOW) {
                        ped.followBox.CRASH_THRESHOLD = 0;
                    }
                    if (type == SPEED_DISTANCE) {
                        ((Pedestrian) getPhysicalTraffic().getTrafficObject("ped2")).followBox.settings.setMaxSpeed(pedSpeeds.get(speedIndex));
                    }
                } else {
                    ped.followBox.CRASH_THRESHOLD = 0;
                }
            }
        }
    }

    /**
     * Resets the pedestrian positions
     */
    public void resetPeds() {
        if (mode == TESTING) {
            throw new RuntimeException("Random training should be disabled");
        }

        scenarioLoader = drivingTask.getScenarioLoader();
        HashMap<String, Waypoint> waypointMap = scenarioLoader.getWaypointMap();
        Waypoint startCrossWaypointTmp = waypointMap.get("WP_StartCross");
        startCrossWaypointTmp.setPosition(originalCrossStart);
        Waypoint endCrossWaypointTmp = waypointMap.get("WP_EndCross");
        endCrossWaypointTmp.setPosition(originalCrossEnd);
        Waypoint startWaypointTmp = waypointMap.get("WP_Start");
        startWaypointTmp.setPosition(originalStart);

        roadNetwork = new RoadNetwork(this);

        for (TrafficObject obj : getPhysicalTraffic().getTrafficObjectList()) {
            if (obj instanceof Pedestrian) {
                Pedestrian ped = (Pedestrian) obj;
                ped.setToWayPoint(ped.followBox.startWayPoint.getName());
                ped.followBox.settings.setMaxSpeed(sampleSpeed());
            }
        }

        isCrossing = false;

        if (rand.nextInt(100) < 20) {
            doNotCross = true;
        } else {
            doNotCross = false;
            crossingDistance = ((float) rand.nextInt(500)) / 10.0f;

            if (TRAINING_SET == 9) {
                crossingDistance += 15;
            }
        }
    }

    Random rand = new Random();
    static final double mean = 1.34;
    static final double sd = 0.37;

    /**
     * Sample a ped speed in range [mean - 2*sd, mean + 2d*sd]
     */
    float sampleSpeed() {
        double min = (mean - 2 * sd) * 3.6;
        double max = (mean + 2 * sd) * 3.6;

        double range = max - min;
        int count = (int) Math.ceil(range / 0.1);

        int i = rand.nextInt(count);

        return (float) (min + i * 0.1);
    }

    Vector3f rotateVector(Vector3f vec, double angle) {
        double theta = Math.toRadians(angle);

        double cs = cos(theta);
        double sn = sin(theta);

        double px = vec.x * cs - vec.z * sn;
        double py = vec.x * sn + vec.z * cs;

        return new Vector3f((float) px, vec.y, (float) py);
    }

    void reloadDrivingTask() {
        try {
            drivingTask = new DrivingTask(this, drivingTask.drivingTaskFileName, drivingTask.drivingTaskPath, drivingTask.dtData);
            if (drivingTask.getScenarioLoader().getWaypointMap().isEmpty()) {
                throw new RuntimeException("Error");
            }
        } catch (Exception npe) {
            while (true) {
                try {
                    File drivingTaskFile = new File(drivingTaskFileName);
                    drivingTask = new DrivingTask(this, drivingTaskFile);
                    if (drivingTask.getScenarioLoader().getWaypointMap().isEmpty()) {
                        throw new RuntimeException("Error");
                    }
                    System.err.println("Nullpointer resolved");
                    break;
                } catch (Exception nnpe) {
                    System.err.println("Nullpointer while loading driving task.");
                }
            }
        }

        for (Spatial s : sceneNode.getChildren()) {
            if (s.getName() != null && s.getName().equals("Obstacle")) {
                obstacle = s;
                break;
            }
        }

    }

    @Override
    public void simpleUpdate(float tpf) {

        if (initializationFinished) {
            super.simpleUpdate(tpf);

            if (isPause()) {
                return;
            }

            chronoPhysicsSpace.update(tpf);

            // updates camera
            cameraFactory.updateCamera();

            if (!isPause())
                car.getTransmission().updateRPM(tpf);


            if (!isHeadLess)
                PanelCenter.update();

            triggerCenter.doTriggerChecks();

            updateDataWriter();

            if (!isPause())
                car.update(tpf, getPhysicalTraffic().getTrafficObjectList());

            physicalTraffic.update(tpf);

            SpeedControlCenter.update();

            // update necessary even in pause
            if (!isHeadLess)
                AudioCenter.update(tpf, cam);

            if (cameraFlight != null)
                cameraFlight.update();

            // update effects
            effectCenter.update(tpf);

            // forward instruction screen if available
            if (instructionScreenID != null) {
                instructionScreenGUI.showDialog(instructionScreenID);
                instructionScreenID = null;
            }

            Pedestrian ped = (Pedestrian) getPhysicalTraffic().getTrafficObject("ped2");
            if (type == ZIGZAG_FOLLOW && ped != null && ped.followBox != null) {
                Segment currentSegment = ped.followBox.getCurrentSegment();
                if (currentSegment != null) {
                    String segmentName = currentSegment.getName();
                    if (segmentName.startsWith("newSegment_")) {
                        if (Integer.parseInt(segmentName.split("_")[1]) % 2 == 0) {
                            isCrossing = true;
                        } else {
                            isCrossing = false;
                        }
                    }
                }
            }


            //System.out.println(ped.followBox.getCurrentSegment().getName());

            if (!doNotCross &&
                    (((mode == Config.SimulationMode.TRAINING)) ||
                            (mode == TESTING && type == MULTIPLE_PEDESTRIANS) ||
                            (mode == TESTING && type == SPEED_DISTANCE)
                    )
                    && (!isPause()) && (!isCrossing)) {

                Vector3f carPos = car.getCenterGeometry().getWorldTranslation();
                Vector3f carFrontPos = car.frontGeometry.getWorldTranslation();
                Vector3f carMovementDirection = carFrontPos.subtract(carPos).normalize().mult(1.4f);
                Vector3f updatedFrontPos = carPos.add(carMovementDirection);

                //Pedestrian ped = (Pedestrian) getPhysicalTraffic().getTrafficObject("ped2");

                if (ped.getPosition().length() > 0) {
                    HashMap<String, Waypoint> waypointMap = scenarioLoader.getWaypointMap();

                    Waypoint startWaypoint = waypointMap.get("WP_Start");
                    Waypoint startCrossWaypoint = waypointMap.get("WP_StartCross");

                    if (startCrossWaypoint.getPosition().distance(ped.getPosition()) < 0.5) {
                        isCrossing = true;
                    }

                    Vector3f movementDirection = startCrossWaypoint.getPosition().subtract(startWaypoint.getPosition());

                    // End point
                    Vector3f perpendicularVector = getPerpendicularVector(movementDirection);


                    if (TRAINING_SET == 9 || TRAINING_SET == 12 || TRAINING_SET == 13 || TRAINING_SET == 14 || TRAINING_SET == 17 || TRAINING_SET == 22) {
                        perpendicularVector = perpendicularVector.mult(-1);
                        perpendicularVector = perpendicularVector.normalize().mult(26f);
                    } else {
                        perpendicularVector = perpendicularVector.normalize().mult(20f);
                    }

                    perpendicularVector = ped.getPosition().add(perpendicularVector);

                    Vector3f originPosTmp = ped.getPosition().clone();
                    originPosTmp.y = 0;
                    Vector3f perpendicularTmp = perpendicularVector.clone();
                    perpendicularTmp.y = 0;
                    Vector3f updatedFrontPosTmp = updatedFrontPos.clone();
                    updatedFrontPosTmp.y = 0;

                    Line crossingLine = new Line(originPosTmp, perpendicularTmp.subtract(originPosTmp).normalize());
                    float distanceToCrossingLine = crossingLine.distance(updatedFrontPosTmp);

                    if (car.getPosition().distance(ped.getPosition()) < crossingDistance + 5 && distanceToCrossingLine < crossingDistance) {
                        if (obstaclePosition != NONE) {
                            if (type == SPEED_DISTANCE) {
                                ped.followBox.settings.setMaxSpeed(pedSpeeds.get(speedIndex));
                            } else {
                                ped.followBox.settings.setMaxSpeed(ped.followBox.settings.originalSpeed);
                            }
                        }

                        reloadDrivingTask();

                        scenarioLoader = drivingTask.getScenarioLoader();
                        waypointMap = scenarioLoader.getWaypointMap();
                        Map<String, Segment> segmentMap = scenarioLoader.getSegmentMap();

                        // start crossing now
                        Waypoint startCrossWaypointTmp = waypointMap.get("WP_StartCross");

                        if (type == MULTIPLE_PEDESTRIANS) {
                            startCrossWaypointTmp.setPosition(ped.getPosition());
                            Waypoint endCrossWaypointTmp = waypointMap.get("WP_EndCross");
                            endCrossWaypointTmp.setPosition(perpendicularVector);

                            Pedestrian ped2 = (Pedestrian) getPhysicalTraffic().getTrafficObject("ped3");
                            Waypoint startWaypointTmp2 = waypointMap.get("WP_Start2");
                            Waypoint startCrossWaypointTmp2 = waypointMap.get("WP_StartCross2");
                            Waypoint endCrossWaypointTmp2 = waypointMap.get("WP_EndCross2");

                            float offset = distanceToFirstPed + (delay * ped2.followBox.settings.getMaxSpeed() * 0.27778f);
                            //startWaypointTmp.setPosition(startWaypointTmp.getPosition().add(direction.mult(offset)));

                            startWaypointTmp2.setPosition(ped2.getPosition());
                            startCrossWaypointTmp2.setPosition(ped2.getPosition().add(movementDirection.normalize().mult(offset)));
                            endCrossWaypointTmp2.setPosition(startCrossWaypointTmp2.getPosition().add(getPerpendicularVector(movementDirection).normalize().mult(15f)));

                            roadNetwork = new RoadNetwork(this);

                            ped.setToWayPoint(startCrossWaypointTmp.getName());
                            ped2.setToWayPoint(startWaypointTmp2.getName());
                        } else {
                            startCrossWaypointTmp.setPosition(ped.getPosition());
                            Waypoint endCrossWaypointTmp = waypointMap.get("WP_EndCross");
                            endCrossWaypointTmp.setPosition(perpendicularVector);

                            roadNetwork = new RoadNetwork(this);

                            ped.setToWayPoint(startCrossWaypointTmp.getName());
                        }

                        isCrossing = true;
                        ped.visible = true;
                    }


                }
            }

            updateCoordinateSystem();
        }
    }

    Vector3f getPerpendicularVector(Vector3f v) {
        Vector3f perpendicularVector = v.clone();
        perpendicularVector.x = v.z;
        perpendicularVector.z = -v.x;

        return perpendicularVector;
    }

    public double pointToLineDistance(Vector3f origin, Vector3f end, Vector3f P) {
        double normalLength = Math.sqrt((end.x - origin.x) * (end.x - origin.x) + (end.z - origin.z) * (end.z - origin.z));
        return Math.abs((P.x - origin.x) * (end.z - origin.z) - (P.y - origin.z) * (end.x - origin.x)) / normalLength;
    }


    private void updateCoordinateSystem() {
        getCoordinateSystem().getChild("x-cone").setLocalTranslation(car.getPosition().getX(), 0, 0);
        getCoordinateSystem().getChild("y-cone").setLocalTranslation(0, car.getPosition().getY(), 0);
        getCoordinateSystem().getChild("z-cone").setLocalTranslation(0, 0, car.getPosition().getZ());
    }


    private void updateDataWriter() {
        if (dataWriter != null && dataWriter.isDataWriterEnabled()) {
            if (!isPause())
                dataWriter.saveAnalyzerData();

            if (!dataWriterQuittable)
                dataWriterQuittable = true;
        } else {
            if (dataWriterQuittable) {
                dataWriter.quit();
                dataWriter = null;
                dataWriterQuittable = false;
            }
        }
    }


    /**
     * Cleanup after game loop was left.
     * Will be called when pressing any close-button.
     * destroy() will be called subsequently.
     */
	/*
	@Override
    public void stop()
    {
		logger.info("started stop()");
		super.stop();
		logger.info("finished stop()");
    }
	*/


    /**
     * Cleanup after game loop was left
     * Will be called whenever application is closed.
     */

    @Override
    public void destroy() {
        logger.info("started destroy()");

        if (initializationFinished) {
            chronoPhysicsSpace.destroy();

            if (lightningClient != null)
                lightningClient.close();

            if (canClient != null)
                canClient.requestStop();

            if (multiDriverClient != null)
                multiDriverClient.close();

            trafficLightCenter.close();

            //steeringTask.close();

            //threeVehiclePlatoonTask.close();

            // moviePlayer.stop();

            // reactionCenter.close();

            HMICenter.close();

            KnowledgeBase.KB.disconnect();

            car.close();

            physicalTraffic.close();

            if (settingsControllerServer != null)
                settingsControllerServer.close();

            if (eyetrackerCenter != null)
                eyetrackerCenter.close();

            joystickSpringController.close();
            //initDrivingTaskSelectionGUI();
        }

        super.destroy();
        logger.info("finished destroy()");

        PlatformImpl.exit();
        //System.exit(0);
    }


    public static void main(String[] args) {
        try {
            // copy native files of force feedback joystick driver
            boolean isWindows = System.getProperty("os.name").toLowerCase().contains("win");
            if (isWindows) {
                boolean is64Bit = System.getProperty("sun.arch.data.model").equalsIgnoreCase("64");
                if (is64Bit) {
                    copyFile("lib/ffjoystick/native/win64/ffjoystick.dll", "ffjoystick.dll");
                    copyFile("lib/ffjoystick/native/win64/SDL.dll", "SDL.dll");
                } else {
                    copyFile("lib/ffjoystick/native/win32/ffjoystick.dll", "ffjoystick.dll");
                    copyFile("lib/ffjoystick/native/win32/SDL.dll", "SDL.dll");
                }
            }


            // load logger configuration file
            PropertyConfigurator.configure("assets/JasperReports/log4j/log4j.properties");

    		/*
    		logger.debug("Sample debug message");
    		logger.info("Sample info message");
    		logger.warn("Sample warn message");
    		logger.error("Sample error message");
    		logger.fatal("Sample fatal message");
    		*/

            // only show sev214.63ere jme3-logs
            java.util.logging.Logger.getLogger("").setLevel(java.util.logging.Level.SEVERE);

            if (!isHeadLess) {
                PlatformImpl.startup(() -> {
                });
            }

            Simulator sim = new Simulator();

            sim.speed = 2.0f;

            StartPropertiesReader startPropertiesReader = new StartPropertiesReader();
            AppSettings settings = startPropertiesReader.getSettings();
            settings.setFrameRate(30);

            sim.setSettings(settings);

            // show/hide settings screen
            sim.setShowSettings(startPropertiesReader.showSettingsScreen());

            if (startPropertiesReader.showBorderlessWindow())
                System.setProperty("org.lwjgl.opengl.Window.undecorated", "true");

            if (!startPropertiesReader.getDrivingTaskPath().isEmpty() &&
                    DrivingTask.isValidDrivingTask(new File(startPropertiesReader.getDrivingTaskPath()))) {
                sim.drivingTaskFileName = startPropertiesReader.getDrivingTaskPath();
                sim.drivingTaskGiven = true;
            }

            if (!startPropertiesReader.getDriverName().isEmpty())
                SimulationDefaults.driverName = startPropertiesReader.getDriverName();


            float param1 = -1;
            float param2 = -1;

            if (args.length >= 1) {
                try {
                    param1 = Float.parseFloat(args[0]);
                } catch (NumberFormatException nfe) {

                }
            }

            if (args.length >= 2) {
                try {
                    param2 = Float.parseFloat(args[1]);
                } catch (NumberFormatException nfe) {

                }
            }

            if(args.length >= 6){
                sim.mode = Config.SimulationMode.valueOf(args[5]);
            }

            if(args.length >= 7){
                sim.type = Config.ExperimentType.valueOf(args[6]);
            }

            if(sim.type == MULTIPLE_PEDESTRIANS && (!MULTIPLE_PEDESTRIAN_SETS.contains(sim.TRAINING_SET))){
                sim.TRAINING_SET = 10;
            }

            if(param1 != -1 && param2 != -1){
                if(sim.type == SPEED_DISTANCE){
                    sim.speedIndex = Math.round(param1);
                    sim.distanceIndex = Math.round(param2);
                }else if(sim.type == ZIGZAG_FOLLOW){
                    sim.zigZagLength = param1;
                    sim.zigZagAngle = param2;
                }else if(sim.type == MULTIPLE_PEDESTRIANS){
                    sim.distanceToFirstPed = param1;
                    sim.delay = param2;
                }
            }

            if (args.length >= 3) {
                sim.serverPort = Integer.parseInt(args[2]);
            }

            if(args.length >= 4){
                int chosenSet = Integer.parseInt(args[3]);
                if ((sim.type != MULTIPLE_PEDESTRIANS || MULTIPLE_PEDESTRIAN_SETS.contains(chosenSet))) {
                    sim.TRAINING_SET = Integer.parseInt(args[3]);
                }
            }

            if(args.length >= 5){
                sim.obstaclePosition = ScenarioConfig.ObstaclePositions.valueOf(args[4]);
            }

            sim.setPauseOnLostFocus(false);

            if (isHeadLess) {
                sim.setShowSettings(false);
                sim.start(Type.Headless);
            } else
                sim.start();
        } catch (Exception e1) {
            logger.fatal("Could not run main method:", e1);
        }
    }


    private static void copyFile(String sourceString, String targetString) {
        try {

            Path source = Paths.get(sourceString);
            Path target = Paths.get(targetString);

            if (Files.exists(source, LinkOption.NOFOLLOW_LINKS))
                Files.copy(source, target, StandardCopyOption.REPLACE_EXISTING);
            else
                System.err.println("ERROR: '" + sourceString + "' does not exist.");

        } catch (IOException e) {

            e.printStackTrace();
        }
    }


}