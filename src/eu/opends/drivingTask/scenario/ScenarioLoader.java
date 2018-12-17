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

package eu.opends.drivingTask.scenario;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

import javax.xml.xpath.XPathConstants;

import org.jfree.data.time.Quarter;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;

import Jama.Matrix;

import com.jme3.math.FastMath;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector2f;
import com.jme3.math.Vector3f;

import eu.opends.basics.MapObject;
import eu.opends.basics.SimulationBasics;
import eu.opends.cameraFlight.CameraFlightSettings;
import eu.opends.car.ResetPosition;
import eu.opends.drivingTask.DrivingTask;
import eu.opends.drivingTask.DrivingTaskDataQuery;
import eu.opends.drivingTask.DrivingTaskDataQuery.Layer;
import eu.opends.drivingTask.scene.SceneLoader;
import eu.opends.effects.WeatherSettings;
import eu.opends.environment.LaneLimit;
import eu.opends.environment.TrafficLight;
import eu.opends.environment.TrafficLightPhase;
import eu.opends.environment.TrafficLightPositionData;
import eu.opends.environment.TrafficLight.TrafficLightDirection;
import eu.opends.environment.TrafficLight.TrafficLightState;
import eu.opends.infrastructure.Segment;
import eu.opends.infrastructure.Waypoint;
import eu.opends.main.DriveAnalyzer;
import eu.opends.main.Simulator;
import eu.opends.traffic.FollowBoxSettings;
import eu.opends.traffic.PedestrianData;
import eu.opends.traffic.PhysicalTraffic;
import eu.opends.traffic.TrafficCarData;


/**
 * @author Rafael Math
 */
@SuppressWarnings("unchecked")
public class ScenarioLoader {
    private static final float MAX_DISTANCE_BETWEEN_TWO_IDEAL_POINTS = 0.1f;

    private DrivingTaskDataQuery dtData;
    private SimulationBasics sim;
    private SceneLoader sceneLoader;
    private float driverCarMass;
    private Vector3f driverCarStartLocation;
    private Quaternion driverCarStartRotation;
    private String driverCarModelPath;
    private CameraFlightSettings cameraFlightSettings;
    private Map<String, LaneLimit> laneList = new HashMap<String, LaneLimit>();
    private Map<String, IdealTrackContainer> idealTrackMap = new HashMap<String, IdealTrackContainer>();
    private HashMap<String, Float> frictionMap = new HashMap<String, Float>();
    private HashMap<String, Segment> segmentsMap = new HashMap<String, Segment>();
    private HashMap<String, Waypoint> waypointMap = new HashMap<String, Waypoint>();
    private List<Intersection> intersectionList = new ArrayList<Intersection>();
    private List<TrafficLight> globalTrafficLightList = new ArrayList<TrafficLight>();
    private Matrix modelToGeoMatrix;
    private Matrix geoToModelMatrix;
    private boolean visualizeRoadSegments = false;
    private boolean visualizeRoadWaypoints = false;
    private boolean isRightHandTraffic = true;


    public enum CarProperty {
        tires_type,
        tires_size,
        engine_engineOn,
        engine_minSpeed,
        engine_maxSpeed,
        engine_acceleration,
        engine_displacement,
        suspension_stiffness,
        suspension_compression,
        suspension_damping,
        brake_decelerationFreeWheel,
        brake_decelerationBrake,
        wheel_frictionSlip,
        engine_minRPM,
        engine_maxRPM,
        light_intensity,
        cruiseControl_acc,
        cruiseControl_safetyDistance_lateral,
        cruiseControl_safetyDistance_forward,
        cruiseControl_emergencyBrakeDistance,
        cruiseControl_suppressDeactivationByBrake,
        cruiseControl_initialSpeed;


        public String getXPathQuery() {
            String[] array = this.toString().split("_");
            if (array.length >= 3)
                return "/scenario:scenario/scenario:driver/scenario:car/scenario:" + array[0] + "/scenario:" + array[1] + "/scenario:" + array[2];
            if (array.length == 2)
                return "/scenario:scenario/scenario:driver/scenario:car/scenario:" + array[0] + "/scenario:" + array[1];
            else
                return "/scenario:scenario/scenario:driver/scenario:car/scenario:" + array[0];
        }
    }


    public ScenarioLoader(DrivingTaskDataQuery dtData, SimulationBasics sim, DrivingTask drivingTask) {
        this.dtData = dtData;
        this.sim = sim;
        this.sceneLoader = drivingTask.getSceneLoader();
        extractRoadSegments();
        extractRoadWaypoints();
        processSceneCar();
        extractTraffic();
        extractCameraFlight();
        extractConversionMatrices();
        extractFrictionMap();

        if (sim instanceof DriveAnalyzer)
            extractIdealLine();

        extractRoadInformation();
        //extractTrafficLights();
    }


    private void processSceneCar() {
        String driverCarRef = dtData.getValue(Layer.SCENARIO,
                "/scenario:scenario/scenario:driver/scenario:car/@ref", String.class);

        MapObject sceneCar = null;

        for (MapObject mapObject : sceneLoader.getMapObjects()) {
            if (mapObject.getName().equals(driverCarRef)) {
                sceneCar = mapObject;
                driverCarMass = sceneCar.getMass();
                driverCarStartLocation = sceneCar.getLocation();
                driverCarStartRotation = sceneCar.getRotation();
                driverCarModelPath = sceneCar.getModelPath();
            }
        }

        if (sceneCar != null)
            sceneLoader.getMapObjects().remove(sceneCar);

        extractResetPoints();
    }

    float[] xArray = new float[]{23.611113f, 58.663197f, 88.59374f, 136.09377f, 86.42361f, 184.14932f, 190.08684f, 42.22224f};
    float[] zArray = new float[]{-61.527794f, -161.2848f, -232.639f, -333.90637f, -353.64597f, -305.24313f, -193.21194f, -253.12515f};

    float[][] addPos = new float[][]{new float[]{90.0001f, -232.6389f}, new float[]{190.4168f, -191.94444f}, new float[]{187.36122f, -143.61111f}, new float[]{136.4758f, -334.16675f}, new float[]{86.44106f, -354.40982f}, new float[]{42.50008f, -254.32295f}, new float[]{200.67725f, -280.5556f}, new float[]{174.5314f, -416.1459f}, new float[]{122.63903f, -305.60773f}, new float[]{98.68071f, -254.566f}, new float[]{75.34737f, -194.98265f}, new float[]{70.76403f, -196.64932f}, new float[]{58.5939f, -159.84378f}, new float[]{97.23974f, -146.51045f}, new float[]{-5.676949f, -183.38547f}, new float[]{160.78142f, -81.71876f}, new float[]{85.57307f, -72.13543f}, new float[]{82.6564f, -62.760418f}, new float[]{-32.13528f, -108.38544f}, new float[]{-34.218613f, -100.26044f}, new float[]{-78.489456f, -166.04172f}, new float[]{-51.545013f, -248.48965f}, new float[]{-7.638747f, -269.04523f}, new float[]{140.55574f, -210.98964f}, new float[]{177.55226f, -311.71884f}, new float[]{204.44464f, -227.25702f}, new float[]{198.40297f, -228.22923f}, new float[]{62.014034f, -299.3057f}, new float[]{-64.91308f, -204.87857f}};


    Quaternion[] directions = new Quaternion[]{new Quaternion().fromAngles(-0.0025712692f, 0.03326704f, -5.86821E-5f),
            new Quaternion().fromAngles(-0.0025712654f, 1.5271299f, -5.864317E-5f),
            new Quaternion().fromAngles(-0.0025712722f, -3.1344445f, -5.8668284E-5f),
            new Quaternion().fromAngles(-0.0025654996f, -1.4783081f, -5.8666337E-5f)
    };

    private void extractResetPoints() {
        String path = "/scenario:scenario/scenario:driver/scenario:car/scenario:resetPoints/scenario:resetPoint";
        try {
            NodeList positionNodes = (NodeList) dtData.xPathQuery(Layer.SCENARIO,
                    path, XPathConstants.NODESET);

            for (int k = 1; k <= positionNodes.getLength(); k++) {
                ResetPosition resetPosition = createResetPosition(path + "[" + k + "]");

                String resetPositionRef = dtData.getValue(Layer.SCENARIO,
                        path + "[" + k + "]/@ref", String.class);

                Map<String, ResetPosition> resetPositionMap = sceneLoader.getResetPositionMap();

                if (resetPosition != null) {
                    sim.getResetPositionList().add(resetPosition);
                } else if ((resetPositionRef != null) && (resetPositionMap.containsKey(resetPositionRef))) {
                    ResetPosition refPosition = resetPositionMap.get(resetPositionRef);
                    sim.getResetPositionList().add(refPosition);
                }
            }

        } catch (Exception e) {
            e.printStackTrace();
        }

		/*
		for(int i = 0; i < xArray.length; ++i){
		    for(int q = 0; q < directions.length; ++q){
                ResetPosition resetPosition = new ResetPosition(new Vector3f(xArray[i],0.1f,zArray[i]), directions[q]);
                sim.getResetPositionList().add(resetPosition);
            }
		}

		for(int i = 0; i < addPos.length; ++i){
			float[] tmp = addPos[i];
            for(int q = 0; q < directions.length; ++q) {
                ResetPosition resetPosition = new ResetPosition(new Vector3f(tmp[0], 0.1f, tmp[1]), directions[q]);
                sim.getResetPositionList().add(resetPosition);
            }
		}*/
    }


    private void extractRoadInformation() {
        String path = "/scenario:scenario/scenario:road/scenario:lane";

        try {
            NodeList laneNodes = (NodeList) dtData.xPathQuery(Layer.SCENARIO,
                    path, XPathConstants.NODESET);

            for (int k = 1; k <= laneNodes.getLength(); k++) {
                String laneID = dtData.getValue(Layer.SCENARIO, path + "[" + k + "]/@id", String.class);

                Float xMin = dtData.getValue(Layer.SCENARIO, path + "[" + k + "]/scenario:xMin", Float.class);

                Float xMax = dtData.getValue(Layer.SCENARIO, path + "[" + k + "]/scenario:xMax", Float.class);


                if (laneID != null && !laneID.isEmpty() && xMin != null && xMax != null) {
                    LaneLimit laneLimit = new LaneLimit(xMin, xMax);
                    laneList.put(laneID, laneLimit);
                }
            }

        } catch (Exception e) {
            e.printStackTrace();
        }
    }


    private ResetPosition createResetPosition(String path) {
        String id = dtData.getValue(Layer.SCENARIO, path + "/@id", String.class);
        Vector3f translation = dtData.getVector3f(Layer.SCENARIO, path + "/scenario:translation");
        Quaternion rotation = dtData.getQuaternion(Layer.SCENARIO, path + "/scenario:rotation");

        if ((id != null) && (translation != null) && (rotation != null))
            return new ResetPosition(id, translation, rotation);

        return null;
    }


    private void extractCameraFlight() {
        List<Vector3f> cameraFlightWayPointList = new ArrayList<Vector3f>();

        try {

            Float cameraFlightSpeed = dtData.getValue(Layer.SCENARIO,
                    "/scenario:scenario/scenario:driver/scenario:cameraFlight/scenario:speed", Float.class);

            Boolean automaticStart = dtData.getValue(Layer.SCENARIO,
                    "/scenario:scenario/scenario:driver/scenario:cameraFlight/scenario:automaticStart", Boolean.class);

            Boolean automaticStop = dtData.getValue(Layer.SCENARIO,
                    "/scenario:scenario/scenario:driver/scenario:cameraFlight/scenario:automaticStop", Boolean.class);

            NodeList pointNodes = (NodeList) dtData.xPathQuery(Layer.SCENARIO,
                    "/scenario:scenario/scenario:driver/scenario:cameraFlight/scenario:track/scenario:point", XPathConstants.NODESET);

            for (int k = 1; k <= pointNodes.getLength(); k++) {
                Vector3f point = dtData.getVector3f(Layer.SCENARIO,
                        "/scenario:scenario/scenario:driver/scenario:cameraFlight/scenario:track/scenario:point[" + k + "]/scenario:translation");

                String pointRef = dtData.getValue(Layer.SCENARIO,
                        "/scenario:scenario/scenario:driver/scenario:cameraFlight/scenario:track/scenario:point[" + k + "]/@ref", String.class);

                Map<String, Vector3f> pointMap = sceneLoader.getPointMap();

                if (point != null) {
                    cameraFlightWayPointList.add(point);
                } else if ((pointRef != null) && (pointMap.containsKey(pointRef))) {
                    Vector3f translation = pointMap.get(pointRef);
                    cameraFlightWayPointList.add(translation);
                } else
                    throw new Exception("Error in camera flight way point list");
            }

            cameraFlightSettings = new CameraFlightSettings(cameraFlightSpeed, automaticStart,
                    automaticStop, cameraFlightWayPointList);

        } catch (Exception e) {
            e.printStackTrace();
        }
    }


    public CameraFlightSettings getCameraFlightSettings() {
        return cameraFlightSettings;
    }


    private void extractIdealLine() {
        try {

            NodeList idealTrackNodes = (NodeList) dtData.xPathQuery(Layer.SCENARIO,
                    "/scenario:scenario/scenario:driver/scenario:idealTracks/scenario:idealTrack", XPathConstants.NODESET);

            for (int i = 1; i <= idealTrackNodes.getLength(); i++) {
                String idealTrackID = dtData.getValue(Layer.SCENARIO,
                        "/scenario:scenario/scenario:driver/scenario:idealTracks/scenario:idealTrack[" + i + "]/@id", String.class);

                Float roadWidth = dtData.getValue(Layer.SCENARIO,
                        "/scenario:scenario/scenario:driver/scenario:idealTracks/scenario:idealTrack[" + i + "]/@roadWidth", Float.class);

                if (idealTrackID != null) {
                    NodeList pointNodes = (NodeList) dtData.xPathQuery(Layer.SCENARIO,
                            "/scenario:scenario/scenario:driver/scenario:idealTracks/scenario:idealTrack[" + i + "]/scenario:point", XPathConstants.NODESET);

                    List<Vector3f> idealPoint3fList = new ArrayList<Vector3f>();

                    for (int k = 1; k <= pointNodes.getLength(); k++) {
                        Vector3f point = dtData.getVector3f(Layer.SCENARIO,
                                "/scenario:scenario/scenario:driver/scenario:idealTracks/scenario:idealTrack[" + i + "]/scenario:point[" + k + "]/scenario:translation");

                        String pointRef = dtData.getValue(Layer.SCENARIO,
                                "/scenario:scenario/scenario:driver/scenario:idealTracks/scenario:idealTrack[" + i + "]/scenario:point[" + k + "]/@ref", String.class);

                        Map<String, Vector3f> pointMap = sceneLoader.getPointMap();

                        if (point != null) {
                            idealPoint3fList.add(point);
                        } else if ((pointRef != null) && (pointMap.containsKey(pointRef))) {
                            Vector3f translation = pointMap.get(pointRef);
                            idealPoint3fList.add(translation);
                        } else
                            throw new Exception("Error in ideal point list");
                    }

                    ArrayList<Vector2f> idealPoint2fList = new ArrayList<Vector2f>();
                    for (Vector3f idealPoint : idealPoint3fList) {
                        Vector2f idealPoint2f = new Vector2f(idealPoint.getX(), idealPoint.getZ());
                        //((DriveAnalyzer) sim).getDeviationComputer().addIdealPoint(idealPoint2f);

                        if (idealPoint2f != null) {
                            // If the distance between two ideal points is too large, add (an) additional
                            // ideal point(s) between them in order to allow a better approximation

                            // exclude first ideal point which has no predecessor
                            if (!idealPoint2fList.isEmpty()) {
                                // if distance to previous ideal point is too large, add (a) new ideal point(s)
                                Vector2f previousIdealPoint = idealPoint2fList.get(idealPoint2fList.size() - 1);
                                while (previousIdealPoint.distance(idealPoint2f) > MAX_DISTANCE_BETWEEN_TWO_IDEAL_POINTS) {
                                    previousIdealPoint = createIdealPoint(previousIdealPoint, idealPoint2f);
                                    idealPoint2fList.add(previousIdealPoint);
                                }
                            }

                            // add current ideal point to list
                            idealPoint2fList.add(idealPoint2f);
                        }
                    }

                    idealTrackMap.put(idealTrackID, new IdealTrackContainer(roadWidth, idealPoint2fList));
                }
            }

        } catch (Exception e) {
            e.printStackTrace();
        }
    }


    public Map<String, IdealTrackContainer> getIdealTrackMap() {
        return idealTrackMap;
    }


    private void extractFrictionMap() {
        try {

            NodeList frictionItemNodes = (NodeList) dtData.xPathQuery(Layer.SCENARIO,
                    "/scenario:scenario/scenario:frictionMap/scenario:frictionItem", XPathConstants.NODESET);

            for (int i = 1; i <= frictionItemNodes.getLength(); i++) {
                String geometryID = dtData.getValue(Layer.SCENARIO,
                        "/scenario:scenario/scenario:frictionMap/scenario:frictionItem[" + i + "]/@geometry", String.class);

                Float value = dtData.getValue(Layer.SCENARIO,
                        "/scenario:scenario/scenario:frictionMap/scenario:frictionItem[" + i + "]/@value", Float.class);

                if (geometryID != null && value != null) {
                    if (frictionMap.containsKey(geometryID))
                        System.err.println("Friction value of " + geometryID + " overwritten by " + value +
                                " (previous value: " + frictionMap.get(geometryID) + ")");

                    frictionMap.put(geometryID, value);
                }
            }

        } catch (Exception e) {
            e.printStackTrace();
        }
    }


    public HashMap<String, Float> getFrictionMap() {
        return frictionMap;
    }


    /**
     * Computes a point on the ideal line between previous and current
     * ideal point with distance "MAX_DISTANCE_BETWEEN_TWO_IDEAL_POINTS"
     * from previous ideal point towards current ideal point.
     *
     * @param previousIdealPoint Previous ideal point. The new point will have the distance
     *                           specified in "MAX_DISTANCE_BETWEEN_TWO_IDEAL_POINTS" from
     *                           this point.
     * @param currentIdealPoint  Current ideal point.
     * @return New point on ideal line (between previousIdealPoint and
     * currentIdealPoint) with distance MAX_DISTANCE_BETWEEN_TWO_IDEAL_POINTS
     * from previousIdealPoint.
     */
    private Vector2f createIdealPoint(Vector2f previousIdealPoint, Vector2f currentIdealPoint) {
        // difference in x- and y-coordinates between previous and current ideal point
        float diffX = currentIdealPoint.x - previousIdealPoint.x;
        float diffY = currentIdealPoint.y - previousIdealPoint.y;

        // square distance and difference values for lambda computation
        float distanceSquare = FastMath.sqr(MAX_DISTANCE_BETWEEN_TWO_IDEAL_POINTS);
        float diffXSquare = FastMath.sqr(diffX);
        float diffYSquare = FastMath.sqr(diffY);

        // lambda is the factor (between 0 and 1) indicating the new point's position
        // between previous and current ideal point:
        // lambda = 0   --> new point has same position as previous ideal point
        // lambda = 0.5 --> new point in the middle between previous and current ideal point
        // lambda = 1   --> new point has same position as current ideal point
        float lambda = FastMath.sqrt(distanceSquare / (diffXSquare + diffYSquare));

        // compute new point's x- and y-coordinates from the previous ideal point's coordinates
        Vector2f newIdealPoint = new Vector2f();
        newIdealPoint.x = lambda * diffX + previousIdealPoint.x;
        newIdealPoint.y = lambda * diffY + previousIdealPoint.y;

        return newIdealPoint;
    }


    private void extractConversionMatrices() {
        String modelToGeoPath = "/scenario:scenario/scenario:coordinateConversion/scenario:modelToGeo";
        modelToGeoMatrix = dtData.getMatrix(Layer.SCENARIO, modelToGeoPath);

        String geoToModelPath = "/scenario:scenario/scenario:coordinateConversion/scenario:geoToModel";
        geoToModelMatrix = dtData.getMatrix(Layer.SCENARIO, geoToModelPath);
    }


    public Matrix getModelToGeoMatrix() {
        return modelToGeoMatrix;
    }


    public Matrix getGeoToModelMatrix() {
        return geoToModelMatrix;
    }


    public WeatherSettings getWeatherSettings() {
        Float snowingPercentage = dtData.getValue(Layer.SCENARIO,
                "/scenario:scenario/scenario:environment/scenario:weather/scenario:snowingPercentage", Float.class);
        if (snowingPercentage == null)
            snowingPercentage = 0f;

        Float rainingPercentage = dtData.getValue(Layer.SCENARIO,
                "/scenario:scenario/scenario:environment/scenario:weather/scenario:rainingPercentage", Float.class);
        if (rainingPercentage == null)
            rainingPercentage = 0f;

        Float fogPercentage = dtData.getValue(Layer.SCENARIO,
                "/scenario:scenario/scenario:environment/scenario:weather/scenario:fogPercentage", Float.class);
        if (fogPercentage == null)
            fogPercentage = 0f;

        return new WeatherSettings(snowingPercentage, rainingPercentage, fogPercentage);
    }


    public boolean isBloomFilter() {
        Boolean useBloomFilter = dtData.getValue(Layer.SCENARIO,
                "/scenario:scenario/scenario:environment/scenario:useBloomFilter", Boolean.class);

        if (useBloomFilter == null)
            useBloomFilter = false;

        return useBloomFilter;
    }


    public boolean isShadowFilter() {
        Boolean useShadowFilter = dtData.getValue(Layer.SCENARIO,
                "/scenario:scenario/scenario:environment/scenario:useShadowFilter", Boolean.class);

        if (useShadowFilter == null)
            useShadowFilter = true;

        return useShadowFilter;
    }


    /**
     * Looks up the node "startPosition" for the initial location
     * of the car at the beginning of the simulation.
     *
     * @return Initial position of the car at the beginning of the simulation
     */
    public Vector3f getStartLocation() {
        return driverCarStartLocation;
    }


    /**
     * Looks up the node "startPosition" for the initial rotation
     * of the car at the beginning of the simulation.
     *
     * @return Initial rotation of the car at the beginning of the simulation
     */
    public Quaternion getStartRotation() {
        return driverCarStartRotation;
    }


    public float getChassisMass() {
        return driverCarMass;
    }


    public String getModelPath() {
        return driverCarModelPath;
    }


    public Map<String, LaneLimit> getLaneList() {
        return laneList;
    }


    /**
     * Looks up the sub node (specified in parameter name) of the given element node
     * and writes the data to the global variable with the same name. If this was
     * successful, the global variable "isSet_&lt;name&gt;" will be set to true.
     *
     * @param <T>          Type of property to look up.
     * @param carProperty  Property to look up.
     * @param defaultValue Default value (will be returned if no valid property could be found).
     * @return Value of the property.
     */
    public <T> T getCarProperty(CarProperty carProperty, T defaultValue) {
        try {
            Class<T> cast = (Class<T>) defaultValue.getClass();
            T returnValue = (T) dtData.getValue(Layer.SCENARIO, carProperty.getXPathQuery(), cast);

            if (returnValue != null)
                return returnValue;
            else
                return defaultValue;

        } catch (Exception e2) {
            dtData.reportInvalidValueError(carProperty.toString(), dtData.getScenarioPath());
        }

        return (T) defaultValue;
    }


    private void extractTraffic() {
        NodeList vehicleNodes = (NodeList) dtData.xPathQuery(Layer.SCENARIO,
                "/scenario:scenario/scenario:traffic/scenario:vehicle", XPathConstants.NODESET);

        for (int k = 1; k <= vehicleNodes.getLength(); k++) {
            Node currentNode = vehicleNodes.item(k - 1);

            //String name = dtData.getValue(Layer.SCENARIO,
            //		"/scenario:scenario/scenario:traffic/scenario:vehicle["+k+"]/@id", String.class);
            String name = currentNode.getAttributes().getNamedItem("id").getNodeValue();

            NodeList childnodes = currentNode.getChildNodes();

            Float mass = null;
            Float acceleration = null;
            Float decelerationBrake = null;
            Float decelerationFreeWheel = null;
            Boolean engineOn = null;
            String modelPath = null;
            Boolean isSpeedLimitedToSteeringCar = null;
            FollowBoxSettings followBoxSettings = extractFollowBoxSettings(currentNode);


            for (int j = 1; j <= childnodes.getLength(); j++) {
                Node currentChild = childnodes.item(j - 1);

                //Float mass = dtData.getValue(Layer.SCENARIO,
                //		"/scenario:scenario/scenario:traffic/scenario:vehicle["+k+"]/scenario:mass", Float.class);
                if (currentChild.getNodeName().equals("mass")) {
                    mass = Float.parseFloat(currentChild.getTextContent());
                }

                //Float acceleration = dtData.getValue(Layer.SCENARIO,
                //		"/scenario:scenario/scenario:traffic/scenario:vehicle["+k+"]/scenario:acceleration", Float.class);
                else if (currentChild.getNodeName().equals("acceleration")) {
                    acceleration = Float.parseFloat(currentChild.getTextContent());
                }

                //Float decelerationBrake = dtData.getValue(Layer.SCENARIO,
                //		"/scenario:scenario/scenario:traffic/scenario:vehicle["+k+"]/scenario:decelerationBrake", Float.class);
                else if (currentChild.getNodeName().equals("decelerationBrake")) {
                    decelerationBrake = Float.parseFloat(currentChild.getTextContent());
                }

                //Float decelerationFreeWheel = dtData.getValue(Layer.SCENARIO,
                //		"/scenario:scenario/scenario:traffic/scenario:vehicle["+k+"]/scenario:decelerationFreeWheel", Float.class);
                else if (currentChild.getNodeName().equals("decelerationFreeWheel")) {
                    decelerationFreeWheel = Float.parseFloat(currentChild.getTextContent());
                }

                //Boolean engineOn = dtData.getValue(Layer.SCENARIO,
                //		"/scenario:scenario/scenario:traffic/scenario:vehicle["+k+"]/scenario:engineOn", Boolean.class);
                else if (currentChild.getNodeName().equals("engineOn")) {
                    engineOn = Boolean.parseBoolean(currentChild.getTextContent());
                }

                //String modelPath = dtData.getValue(Layer.SCENARIO,
                //		"/scenario:scenario/scenario:traffic/scenario:vehicle["+k+"]/scenario:modelPath", String.class);
                else if (currentChild.getNodeName().equals("modelPath")) {
                    modelPath = currentChild.getTextContent();
                }

                //Boolean isSpeedLimitedToSteeringCar = dtData.getValue(Layer.SCENARIO,
                //		"/scenario:scenario/scenario:traffic/scenario:vehicle["+k+"]/scenario:neverFasterThanSteeringCar", Boolean.class);
                else if (currentChild.getNodeName().equals("neverFasterThanSteeringCar")) {
                    isSpeedLimitedToSteeringCar = Boolean.parseBoolean(currentChild.getTextContent());
                }
            }

            if (mass == null)
                mass = 0f;

            if (acceleration == null)
                acceleration = 3.3f;

            if (decelerationBrake == null)
                decelerationBrake = 8.7f;

            if (decelerationFreeWheel == null)
                decelerationFreeWheel = 2.0f;

            if (engineOn == null)
                engineOn = true;

            if (isSpeedLimitedToSteeringCar == null)
                isSpeedLimitedToSteeringCar = false;

            TrafficCarData trafficCarData = new TrafficCarData(name, mass, acceleration, decelerationBrake,
                    decelerationFreeWheel, engineOn, modelPath, followBoxSettings, isSpeedLimitedToSteeringCar);
            ((Simulator) sim).getPhysicalTraffic().getVehicleDataList().add(trafficCarData);
        }

        sim.getPhysicalTraffic().getPedestrianDataList().clear();

        NodeList pedestrianNodes = (NodeList) dtData.xPathQuery(Layer.SCENARIO,
                "/scenario:scenario/scenario:traffic/scenario:pedestrian", XPathConstants.NODESET);

        for (int k = 1; k <= pedestrianNodes.getLength(); k++) {
            Node currentNode = pedestrianNodes.item(k - 1);

            //String name = dtData.getValue(Layer.SCENARIO,
            //		"/scenario:scenario/scenario:traffic/scenario:pedestrian["+k+"]/@id", String.class);
            String name = currentNode.getAttributes().getNamedItem("id").getNodeValue();

            NodeList childnodes = currentNode.getChildNodes();

            Float mass = null;
            boolean enabled = true;
            String animationStand = null;
            String animationWalk = null;
            Float localScale = null;
            Vector3f localTranslation = null;
            Quaternion localRotation = null;
            String modelPath = null;
            FollowBoxSettings followBoxSettings = extractFollowBoxSettings(currentNode);

            for (int j = 1; j <= childnodes.getLength(); j++) {
                Node currentChild = childnodes.item(j - 1);

                //Float mass = dtData.getValue(Layer.SCENARIO,
                //		"/scenario:scenario/scenario:traffic/scenario:pedestrian["+k+"]/scenario:mass", Float.class);
                if (currentChild.getNodeName().equals("mass")) {
                    mass = Float.parseFloat(currentChild.getTextContent());
                }

                //Boolean enabled = dtData.getValue(Layer.SCENARIO,
                //		"/scenario:scenario/scenario:traffic/scenario:pedestrian["+k+"]/scenario:enabled", Boolean.class);
                else if (currentChild.getNodeName().equals("enabled")) {
                    enabled = Boolean.parseBoolean(currentChild.getTextContent());
                }

                //String animationStand = dtData.getValue(Layer.SCENARIO,
                //		"/scenario:scenario/scenario:traffic/scenario:pedestrian["+k+"]/scenario:animationStand", String.class);
                else if (currentChild.getNodeName().equals("animationStand")) {
                    animationStand = currentChild.getTextContent();
                }

                //String animationWalk = dtData.getValue(Layer.SCENARIO,
                //		"/scenario:scenario/scenario:traffic/scenario:pedestrian["+k+"]/scenario:animationWalk", String.class);
                else if (currentChild.getNodeName().equals("animationWalk")) {
                    animationWalk = currentChild.getTextContent();
                }

                //Float scale = dtData.getValue(Layer.SCENARIO,
                //		"/scenario:scenario/scenario:traffic/scenario:pedestrian["+k+"]/scenario:scale", Float.class);
                else if (currentChild.getNodeName().equals("scale")) {
                    localScale = Float.parseFloat(currentChild.getTextContent());
                } else if (currentChild.getNodeName().equals("localTranslation")) {
                    //localTranslation = getTranslation(currentChild);
                    localTranslation = dtData.getVector3f(Layer.SCENARIO, "/scenario:scenario/scenario:traffic/scenario:pedestrian[" +
                            k + "]/scenario:localTranslation");
                } else if (currentChild.getNodeName().equals("localRotation")) {
                    //localRotation = getRotation(currentChild); // method does not exist
                    localRotation = dtData.getQuaternion(Layer.SCENARIO, "/scenario:scenario/scenario:traffic/scenario:pedestrian[" +
                            k + "]/scenario:localRotation");
                }

                //String modelPath = dtData.getValue(Layer.SCENARIO,
                //		"/scenario:scenario/scenario:traffic/scenario:pedestrian["+k+"]/scenario:modelPath", String.class);
                else if (currentChild.getNodeName().equals("modelPath")) {
                    modelPath = currentChild.getTextContent();
                }
            }

            if (mass == null)
                mass = 50f;

            if (animationStand == null)
                animationStand = "Stand";

            if (animationWalk == null)
                animationWalk = "Walk";

            if (localScale == null)
                localScale = 1.0f;

            if (localTranslation == null)
                localTranslation = new Vector3f(0, 0, 0);

            if (localRotation == null)
                localRotation = new Quaternion();

            PedestrianData pedestrianData = new PedestrianData(name, enabled, mass, animationStand, animationWalk,
                    localScale, localTranslation, localRotation, modelPath, followBoxSettings);
            sim.getPhysicalTraffic().getPedestrianDataList().add(pedestrianData);
        }

    }


    private ArrayList<Segment> extractSegments(Node currentChild) {
        ArrayList<Segment> segmentList = new ArrayList<Segment>();

        try {
            NodeList segmentNodeList = currentChild.getChildNodes();
            for (int k = 1; k <= segmentNodeList.getLength(); k++) {
                Node currentSegmentNode = segmentNodeList.item(k - 1);

                if (currentSegmentNode.getNodeName().equals("segment")) {
                    if (currentSegmentNode.getAttributes().getNamedItem("ref") != null) {
                        String ref = currentSegmentNode.getAttributes().getNamedItem("ref").getNodeValue();
                        if (segmentsMap.containsKey(ref)) {
                            segmentList.add(segmentsMap.get(ref));
                            //System.err.println("ref: " + ref);
                        }
                    }
                }
            }

        } catch (Exception e) {
            e.printStackTrace();
        }

        return segmentList;
    }


    private void extractRoadSegments() {
        Node segmentsNode = (Node) dtData.xPathQuery(Layer.SCENARIO,
                "/scenario:scenario/scenario:road/scenario:segments", XPathConstants.NODE);

        if (segmentsNode != null && segmentsNode.hasChildNodes()) {
                if (segmentsNode.getAttributes().getNamedItem("debug") != null)
                    visualizeRoadSegments = Boolean.parseBoolean(segmentsNode.getAttributes().getNamedItem("debug").getNodeValue());

                if (segmentsNode.getAttributes().getNamedItem("isRightHandTraffic") != null)
                    isRightHandTraffic = Boolean.parseBoolean(segmentsNode.getAttributes().
                            getNamedItem("isRightHandTraffic").getNodeValue());

                NodeList segmentNodeList = segmentsNode.getChildNodes();
                for (int k = 1; k <= segmentNodeList.getLength(); k++) {
                    Node currentSegmentNode = segmentNodeList.item(k - 1);

                    if (currentSegmentNode.getNodeName().equals("segment")) {
                        //String id = getValue(layer, path + "/@id", String.class);
                        String id = null;
                        if (currentSegmentNode.getAttributes().getNamedItem("id") != null)
                            id = currentSegmentNode.getAttributes().getNamedItem("id").getNodeValue();
                        //System.err.println("id: " + id);

                        NodeList childnodes = currentSegmentNode.getChildNodes();

                        String from = null;
                        ArrayList<String> viaList = new ArrayList<String>();
                        String to = null;
                        String leftNeighbor = null;
                        String rightNeighbor = null;
                        Float speed = null;
                        Boolean jump = null;
                        Float probability = null;
                        ArrayList<String> prioritizedSegments = new ArrayList<String>();
                        Float curveTension = null;

                        for (int j = 1; j <= childnodes.getLength(); j++) {
                            Node currentChild = childnodes.item(j - 1);

                            //String from = getValue(layer, path + "/" + layer + ":from", String.class);
                            if (currentChild.getNodeName().equals("from")) {
                                from = currentChild.getTextContent();
                            }

                            if (currentChild.getNodeName().equals("via")) {
                                NodeList wpNodes = currentChild.getChildNodes();
                                for (int l = 1; l <= wpNodes.getLength(); l++) {
                                    Node wpNode = wpNodes.item(l - 1);

                                    if (wpNode.getNodeName().equals("wp"))
                                        viaList.add(wpNode.getTextContent());
                                }
                            }

                            //String to = getValue(layer, path + "/" + layer + ":to", String.class);
                            if (currentChild.getNodeName().equals("to")) {
                                to = currentChild.getTextContent();
                            }

                            //String leftNeighbor = getValue(layer, path + "/" + layer + ":leftNeighbor", String.class);
                            if (currentChild.getNodeName().equals("leftNeighbor")) {
                                leftNeighbor = currentChild.getTextContent();
                            }

                            //String rightNeighbor = getValue(layer, path + "/" + layer + ":rightNeighbor", String.class);
                            if (currentChild.getNodeName().equals("rightNeighbor")) {
                                rightNeighbor = currentChild.getTextContent();
                            }

                            //Float speed = getValue(layer, path + "/" + layer + ":speed", Float.class);
                            if (currentChild.getNodeName().equals("speed")) {
                                speed = Float.parseFloat(currentChild.getTextContent());
                            }

                            //Boolean jump = getValue(layer, path + "/" + layer + ":jump", Boolean.class);
                            if (currentChild.getNodeName().equals("jump")) {
                                jump = Boolean.parseBoolean(currentChild.getTextContent());
                            }

                            //Float probability = getValue(layer, path + "/" + layer + ":probability", Float.class);
                            if (currentChild.getNodeName().equals("probability")) {
                                probability = Float.parseFloat(currentChild.getTextContent());
                            }

                            //String giveWayTo = getValue(layer, path + "/" + layer + ":giveWayTo", Float.class);
                            if (currentChild.getNodeName().equals("giveWayTo")) {
                                String prioritizedSegmentString = currentChild.getTextContent();

                                if (prioritizedSegmentString != null && !prioritizedSegmentString.isEmpty()) {
                                    String[] prioritizedSegmentArray = prioritizedSegmentString.split(",");

                                    for (String prioritizedSegment : prioritizedSegmentArray)
                                        prioritizedSegments.add(prioritizedSegment.trim());
                                }
                            }

                            //Float curveTension = getValue(layer, path + "/" + layer + ":curveTension", Float.class);
                            if (currentChild.getNodeName().equals("curveTension")) {
                                curveTension = Float.parseFloat(currentChild.getTextContent());
                            }
                        }

                        if (speed == null)
                            speed = 50.0f;

                        if (jump == null)
                            jump = false;

                        if (probability == null)
                            probability = 0.5f;

                        if (curveTension == null)
                            curveTension = 0.05f;

                        if ((id != null) && (from != null) && (to != null)) {
                            if (segmentsMap.containsKey(id))
                                System.err.println("Segment " + id + " already contained in segment list");
                            else
                                segmentsMap.put(id, new Segment(id, from, viaList, to, leftNeighbor, rightNeighbor,
                                        speed, jump, probability, prioritizedSegments, curveTension));
                        } else
                            throw new RuntimeException("Error in segment list");
                    }
                }
        }
    }


    private void extractRoadWaypoints() {
        Node wayPointsNode = (Node) dtData.xPathQuery(Layer.SCENARIO,
                "/scenario:scenario/scenario:road/scenario:wayPoints", XPathConstants.NODE);

        if (wayPointsNode != null && wayPointsNode.hasChildNodes()) {

            if (wayPointsNode.getAttributes().getNamedItem("debug") != null)
                visualizeRoadWaypoints = Boolean.parseBoolean(wayPointsNode.getAttributes().getNamedItem("debug").getNodeValue());

            NodeList wayPointNodeList = wayPointsNode.getChildNodes();
            for (int k = 1; k <= wayPointNodeList.getLength(); k++) {
                Node currentWayPointNode = wayPointNodeList.item(k - 1);

                if (currentWayPointNode.getNodeName().equals("wayPoint")) {
                    //String id = getValue(layer, path + "/@id", String.class);
                    String id = null;
                    if (currentWayPointNode.getAttributes().getNamedItem("id") != null)
                        id = currentWayPointNode.getAttributes().getNamedItem("id").getNodeValue();
                    //System.err.println("id: " + id);

                    NodeList childnodes = currentWayPointNode.getChildNodes();

                    Vector3f translation = null;
                    String trafficLightID = null;
                    Float headLightIntensity = null;
                    String turnSignal = null;
                    Boolean brakeLightOn = null;
                    Integer waitingTime = null;

                    for (int j = 1; j <= childnodes.getLength(); j++) {
                        Node currentChild = childnodes.item(j - 1);

                        //translation = getVector3f(layer, path + "/" + layer + ":translation");
                        if (currentChild.getNodeName().equals("translation")) {
                            translation = getTranslation(currentChild);
                        }

                        //String trafficLightID = getValue(layer, path + "/" + layer + ":trafficLight", String.class);
                        if (currentChild.getNodeName().equals("trafficLight")) {
                            trafficLightID = currentChild.getTextContent();
                        }

                        //Float headLightIntensity = getValue(layer, path + "/" + layer + ":headLightIntensity", Float.class);
                        if (currentChild.getNodeName().equals("headLightIntensity")) {
                            headLightIntensity = Float.parseFloat(currentChild.getTextContent());
                        }

                        //String turnSignal = getValue(layer, path + "/" + layer + ":turnSignal", String.class);
                        if (currentChild.getNodeName().equals("turnSignal")) {
                            turnSignal = currentChild.getTextContent();
                        }

                        //Boolean brakeLightOn = getValue(layer, path + "/" + layer + ":brakeLightOn", String.class);
                        if (currentChild.getNodeName().equals("brakeLightOn")) {
                            brakeLightOn = Boolean.parseBoolean(currentChild.getTextContent());
                        }

                        //Integer waitingTime = getValue(layer, path + "/" + layer + ":waitingTime", Integer.class);
                        if (currentChild.getNodeName().equals("waitingTime")) {
                            waitingTime = Integer.parseInt(currentChild.getTextContent());
                        }
                    }

                    //String wayPointRef = dtData.getValue(Layer.SCENARIO, path + "["+k+"]/@ref", String.class);
                    String ref = null;
                    if (currentWayPointNode.getAttributes().getNamedItem("ref") != null)
                        ref = currentWayPointNode.getAttributes().getNamedItem("ref").getNodeValue();
                    //System.err.println("ref: " + ref);

                    Map<String, Vector3f> pointMap = sceneLoader.getPointMap();

                    if ((id != null) && (translation != null)) {
                        ArrayList<Segment> segmentList = getSegmentList(id);
                        waypointMap.put(id, new Waypoint(id, translation, trafficLightID, headLightIntensity, turnSignal, brakeLightOn, waitingTime, segmentList));
                    } else if ((ref != null) && (pointMap.containsKey(ref))) {
                        translation = pointMap.get(ref);

                        if ((translation != null)) {
                            ArrayList<Segment> segmentList = getSegmentList(ref);
                            waypointMap.put(ref, new Waypoint(ref, translation, trafficLightID, headLightIntensity, turnSignal, brakeLightOn, waitingTime, segmentList));
                        }
                    } else
                        throw new RuntimeException("Error in way point list");
                }
            }

        }
    }


    private ArrayList<Segment> getSegmentList(String fromWP) {
        ArrayList<Segment> subList = new ArrayList<Segment>();
        for (Segment segment : segmentsMap.values()) {
            if (segment.getFromWaypointString().equalsIgnoreCase(fromWP))
                subList.add(segment);
        }

        return subList;
    }


    private Vector3f getTranslation(Node translationNode) {
        NodeList vectorNodesList = translationNode.getChildNodes();
        for (int k = 1; k <= vectorNodesList.getLength(); k++) {
            Node currentVectorNode = vectorNodesList.item(k - 1);

            if (currentVectorNode.getNodeName().equals("vector")) {
                NodeList entryNodesList = currentVectorNode.getChildNodes();

                Float[] outVector = new Float[3];
                int outVectorIdx = 0;

                for (int i = 0; i < entryNodesList.getLength(); i++) {
                    if ((entryNodesList.item(i).getNodeType() == Node.ELEMENT_NODE)
                            && (entryNodesList.item(i).getNodeName().equals("entry")))
                        outVector[outVectorIdx++] = Float.parseFloat(entryNodesList.item(i).getTextContent());
                }

                if (outVector[0] != null && outVector[1] != null && outVector[2] != null) {
                    return new Vector3f(outVector[0], outVector[1], outVector[2]);
                }
            }
        }

        return null;
    }


    public boolean isAutomaticTransmission(boolean defaultValue) {
        Boolean isAutomatic = dtData.getValue(Layer.SCENARIO,
                "/scenario:scenario/scenario:driver/scenario:car/scenario:transmission/scenario:automatic", Boolean.class);

        if (isAutomatic != null)
            return isAutomatic;
        else
            return defaultValue;
    }


    public float getReverseGear(float defaultValue) {
        Float transmission = dtData.getValue(Layer.SCENARIO,
                "/scenario:scenario/scenario:driver/scenario:car/scenario:transmission/scenario:reverse", Float.class);

        if (transmission != null)
            return transmission;
        else
            return defaultValue;
    }


    public float getEngineSoundIntensity(float defaultValue) {
        Float soundIntensity = dtData.getValue(Layer.SCENARIO,
                "/scenario:scenario/scenario:driver/scenario:car/scenario:engine/scenario:soundIntensity", Float.class);

        if (soundIntensity != null)
            return soundIntensity;
        else
            return defaultValue;
    }


    public Float[] getForwardGears(Float[] defaultValue) {
        List<Float> transmission = dtData.getArray(Layer.SCENARIO,
                "/scenario:scenario/scenario:driver/scenario:car/scenario:transmission/scenario:forward", Float.class);

        Float[] transmissionArray = new Float[transmission.size()];

        for (int i = 0; i < transmission.size(); i++)
            transmissionArray[i] = transmission.get(i);

        if ((transmissionArray != null) && (transmissionArray.length >= 1))
            return transmissionArray;
        else
            return defaultValue;
    }


    public HashMap<String, Waypoint> getWaypointMap() {
        return waypointMap;
    }


    public HashMap<String, Segment> getSegmentMap() {
        return segmentsMap;
    }


    private void extractIntersections() {
        String path = "/scenario:scenario/scenario:road/scenario:intersection";

        try {
            NodeList intersectionNodes = (NodeList) dtData.xPathQuery(Layer.SCENARIO,
                    path, XPathConstants.NODESET);

            for (int i = 1; i <= intersectionNodes.getLength(); i++) {
                // get ID of intersection (traffic light group)
                String intersectionID = dtData.getValue(Layer.SCENARIO,
                        path + "[" + i + "]/@id", String.class);

                // get mode of traffic lights at this intersection
                String trafficLightMode = dtData.getValue(Layer.SCENARIO,
                        path + "[" + i + "]/@mode", String.class);

                LinkedList<TrafficLightPhase> trafficLightPhaseList = new LinkedList<TrafficLightPhase>();

                NodeList phaseNodes = (NodeList) dtData.xPathQuery(Layer.SCENARIO,
                        path + "[" + i + "]/scenario:phases/scenario:phase", XPathConstants.NODESET);

                for (int k = 1; k <= phaseNodes.getLength(); k++) {
                    // get ID of traffic light phase
                    String phaseID = dtData.getValue(Layer.SCENARIO,
                            path + "[" + i + "]/scenario:phases/scenario:phase[" + k + "]/@id", String.class);

                    // get duration of phase
                    Integer duration = dtData.getValue(Layer.SCENARIO,
                            path + "[" + i + "]/scenario:phases/scenario:phase[" + k + "]/@duration", Integer.class);

                    // get state of phase
                    String state = dtData.getValue(Layer.SCENARIO,
                            path + "[" + i + "]/scenario:phases/scenario:phase[" + k + "]/@state", String.class);

                    trafficLightPhaseList.add(new TrafficLightPhase(phaseID, duration, state));
                }

                if (trafficLightPhaseList.size() == 0)
                    trafficLightPhaseList = null;

                List<TrafficLight> localTrafficLightList = new ArrayList<TrafficLight>();

                NodeList trafficLightNodes = (NodeList) dtData.xPathQuery(Layer.SCENARIO,
                        path + "[" + i + "]/scenario:trafficLights/scenario:trafficLight", XPathConstants.NODESET);

                for (int k = 1; k <= trafficLightNodes.getLength(); k++) {
                    // get ID and spatial of traffic light
                    String trafficLightID = dtData.getValue(Layer.SCENARIO,
                            path + "[" + i + "]/scenario:trafficLights/scenario:trafficLight[" + k + "]/@id", String.class);

                    // get phase positon of traffic light
                    Integer phasePosition = dtData.getValue(Layer.SCENARIO,
                            path + "[" + i + "]/scenario:trafficLights/scenario:trafficLight[" + k + "]/@phasePosition", Integer.class);
                    if (phasePosition == null)
                        phasePosition = 1;

                    // get trigger of traffic light (might be null)
                    String trafficLightTriggerID = dtData.getValue(Layer.SCENARIO,
                            path + "[" + i + "]/scenario:trafficLights/scenario:trafficLight[" + k + "]/@trigger", String.class);

                    // get trigger of traffic light phase (might be null)
                    String trafficLightPhaseTriggerID = dtData.getValue(Layer.SCENARIO,
                            path + "[" + i + "]/scenario:trafficLights/scenario:trafficLight[" + k + "]/@phaseTrigger", String.class);

                    // get initial traffic light state
                    TrafficLightState initialState = TrafficLightState.RED;
                    String initialStateString = dtData.getValue(Layer.SCENARIO,
                            path + "[" + i + "]/scenario:trafficLights/scenario:trafficLight[" + k + "]/scenario:initialState", String.class);
                    if (!initialStateString.isEmpty())
                        initialState = TrafficLightState.valueOf(initialStateString.toUpperCase());

                    // get direction of arrow
                    TrafficLightDirection direction = TrafficLightDirection.NONE;
                    String directionString = dtData.getValue(Layer.SCENARIO,
                            path + "[" + i + "]/scenario:trafficLights/scenario:trafficLight[" + k + "]/scenario:direction", String.class);
                    if (!directionString.isEmpty())
                        direction = TrafficLightDirection.valueOf(directionString.toUpperCase());

                    // get conflicting traffic lights
                    String requiresRedString = dtData.getValue(Layer.SCENARIO,
                            path + "[" + i + "]/scenario:trafficLights/scenario:trafficLight[" + k + "]/scenario:requiresRed", String.class);
                    ArrayList<String> requiresRedList = new ArrayList<String>();
                    for (String reqiresRedTL : requiresRedString.split(","))
                        requiresRedList.add(reqiresRedTL);

                    // get position data (for SIM-TD traffic light phase assistant)
                    String roadID = dtData.getValue(Layer.SCENARIO,
                            path + "[" + i + "]/scenario:trafficLights/scenario:trafficLight[" + k + "]/scenario:positionData/@roadID", String.class);
                    Integer crossingType = dtData.getValue(Layer.SCENARIO,
                            path + "[" + i + "]/scenario:trafficLights/scenario:trafficLight[" + k + "]/scenario:positionData/@crossingType", Integer.class);
                    if (crossingType == null)
                        crossingType = 0;
                    Integer arrowType = dtData.getValue(Layer.SCENARIO,
                            path + "[" + i + "]/scenario:trafficLights/scenario:trafficLight[" + k + "]/scenario:positionData/@arrowType", Integer.class);
                    if (arrowType == null)
                        arrowType = 0;
                    Integer lane = dtData.getValue(Layer.SCENARIO,
                            path + "[" + i + "]/scenario:trafficLights/scenario:trafficLight[" + k + "]/scenario:positionData/@lane", Integer.class);
                    if (lane == null)
                        lane = 0;
                    TrafficLightPositionData positionData = new TrafficLightPositionData(roadID, crossingType, arrowType, lane);

                    // create new traffic light
                    TrafficLight trafficLight = new TrafficLight((Simulator) sim, trafficLightID, trafficLightTriggerID,
                            trafficLightPhaseTriggerID, intersectionID, initialState, direction,
                            phasePosition, requiresRedList, positionData);

                    // if not already contained in global traffic light list
                    if (!containedInList(trafficLightID)) {
                        globalTrafficLightList.add(trafficLight);
                        localTrafficLightList.add(trafficLight);
                    } else
                        System.err.println("Object '" + trafficLightID + "' has been assigned to more than one traffic light!");
                }

                intersectionList.add(new Intersection(intersectionID, trafficLightMode, trafficLightPhaseList, localTrafficLightList));
            }

        } catch (Exception e) {
            e.printStackTrace();
        }
    }


    private boolean containedInList(String trafficLightID) {
        for (TrafficLight t : globalTrafficLightList)
            if (t.getName().equals(trafficLightID))
                return true;

        return false;
    }


    public List<Intersection> getIntersections() {
        extractIntersections();
        return intersectionList;
    }


    public List<TrafficLight> getTrafficLights() {
        return globalTrafficLightList;
    }


    // extract FollowBox settings of driving car
    public FollowBoxSettings getAutoPilotFollowBoxSettings() {
        Node autopilotNode = (Node) dtData.xPathQuery(Layer.SCENARIO,
                "/scenario:scenario/scenario:driver/scenario:car/scenario:autoPilot", XPathConstants.NODE);
        return extractFollowBoxSettings(autopilotNode);
    }


    // extract FollowBox settings
    private FollowBoxSettings extractFollowBoxSettings(Node node) {
        Float minDistance = null;
        Float maxDistance = null;
        Float maxSpeed = null;
        String startWayPoint = null;
        Float giveWayDistance = null;
        Float intersectionObservationDistance = null;
        Float minIntersectionClearance = null;
        ArrayList<Segment> segments = new ArrayList<Segment>();

        try {

            NodeList childnodes = node.getChildNodes();

            for (int j = 1; j <= childnodes.getLength(); j++) {
                Node currentChild = childnodes.item(j - 1);

                if (currentChild.getNodeName().equals("minDistanceFromPath"))
                    minDistance = Float.parseFloat(currentChild.getTextContent());

                else if (currentChild.getNodeName().equals("maxDistanceFromPath"))
                    maxDistance = Float.parseFloat(currentChild.getTextContent());

                else if (currentChild.getNodeName().equals("maxSpeed"))
                    maxSpeed = Float.parseFloat(currentChild.getTextContent());

                else if (currentChild.getNodeName().equals("startWayPoint"))
                    startWayPoint = currentChild.getTextContent();

                else if (currentChild.getNodeName().equals("segments"))
                    segments = extractSegments(currentChild);

                else if (currentChild.getNodeName().equals("giveWayDistance"))
                    giveWayDistance = Float.parseFloat(currentChild.getTextContent());

                else if (currentChild.getNodeName().equals("intersectionObservationDistance"))
                    intersectionObservationDistance = Float.parseFloat(currentChild.getTextContent());

                else if (currentChild.getNodeName().equals("minIntersectionClearance"))
                    minIntersectionClearance = Float.parseFloat(currentChild.getTextContent());
            }

        } catch (Exception e) {
            //e.printStackTrace();
        }

        if (minDistance == null)
            minDistance = 1.0f;

        if (maxDistance == null)
            maxDistance = 10.0f;

        if (maxSpeed == null)
            maxSpeed = Float.MAX_VALUE;

        if (giveWayDistance == null)
            giveWayDistance = 15.0f;

        if (intersectionObservationDistance == null)
            intersectionObservationDistance = 15.0f;

        if (minIntersectionClearance == null)
            minIntersectionClearance = 5.0f;

        return new FollowBoxSettings(segments, minDistance, maxDistance, maxSpeed, startWayPoint,
                giveWayDistance, intersectionObservationDistance, minIntersectionClearance);
    }


    public Boolean isAutoPilot() {
        Boolean enabled = null;
        try {
            Node enabledNode = (Node) dtData.xPathQuery(Layer.SCENARIO,
                    "/scenario:scenario/scenario:driver/scenario:car/scenario:autoPilot/scenario:enabled", XPathConstants.NODE);

            if (enabledNode != null && enabledNode.getNodeName() != null && enabledNode.getNodeName().equals("enabled")) {
                enabled = Boolean.parseBoolean(enabledNode.getTextContent());
            }

        } catch (Exception e) {
            e.printStackTrace();
        }

        return enabled;
    }


    public boolean isVisualizeRoadSegments() {
        return visualizeRoadSegments;
    }


    public boolean isVisualizeRoadWaypoints() {
        return visualizeRoadWaypoints;
    }


    public boolean isRightHandTraffic() {
        return isRightHandTraffic;
    }

}
