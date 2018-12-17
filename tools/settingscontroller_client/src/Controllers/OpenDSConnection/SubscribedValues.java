package settingscontroller_client.src.Controllers.OpenDSConnection;

import org.w3c.dom.Element;
import org.w3c.dom.Node;
import settingscontroller_client.src.Parameters;
import settingscontroller_client.src.TrafficObject.Obstacle;
import settingscontroller_client.src.TrafficObject.Pedestrian;

import java.util.LinkedList;
import java.util.List;
import static settingscontroller_client.src.Parameters.mapResolution;
import static settingscontroller_client.src.Parameters.map_height;
import static settingscontroller_client.src.Parameters.map_width;

/** The state information sent by OpenDS **/
public class SubscribedValues {

    /**
     * Orientation of the car
     */
    public double orientation;

    /**
     * X-coordinate of the car
     */
    public double x;

    /**
     * Z-coordinate of the car
     */
    public double z;

    /**
     * Speed of the car
     */
    public double speed;

    /**
     * Pedestrians in the scene
     */
    public List<Pedestrian> pedestrians = null;

    /**
     * Obstacle in the scene
     */
    public Obstacle obstacle = null;

    /**
     * If the pedestrian is currently crossing
     */
    public boolean isCrossing = true;

    /** Creates the state objects manually from partial information **/
    public SubscribedValues(double orientation, double x, double z, double speed) {
        float adjustedAngle = (float) (orientation + 90) % 360;

        this.orientation = (float) Math.toRadians(adjustedAngle);
        this.x = x;
        this.z = z;
        this.speed = speed;
    }

    /** Creates the state objects manually from partial information **/
    public SubscribedValues(double orientation, double x, double z, double speed, List<Pedestrian> pedestrians) {
        this(orientation, x, z, speed);
        this.pedestrians = pedestrians;
    }

    /** Creates the state objects manually from partial information **/
    public SubscribedValues(double orientation, double x, double z, double speed, List<Pedestrian> pedestrians, Obstacle obstacle){
        this(orientation, x, z, speed, pedestrians);
        this.obstacle = obstacle;
    }

    /** Creates the state objects manually from partial information **/
    public SubscribedValues(double orientation, double x, double z, double speed, List<Pedestrian> pedestrians, Obstacle obstacle, boolean isCrossing){
        this(orientation, x, z, speed, pedestrians);
        this.obstacle = obstacle;
        this.isCrossing = isCrossing;
    }

    /**
     * Parses the state information sent by OpenDS as XML element
     */
    static SubscribedValues parseSubscribedValues(Element eventNode) {
        Element vehicleNode = (Element) eventNode.getElementsByTagName("thisVehicle").item(0);

        Element orientationElement = (Element) vehicleNode.getElementsByTagName("orientation").item(0);
        Element xElement = (Element) vehicleNode.getElementsByTagName("x").item(0);
        Element zElement = (Element) vehicleNode.getElementsByTagName("z").item(0);
        Element speedElement = (Element) vehicleNode.getElementsByTagName("speed").item(0);

        Element isCrossingElement = (Element) vehicleNode.getElementsByTagName("isCrossing").item(0);
        boolean isCrossing = Boolean.parseBoolean(isCrossingElement.getTextContent());

        double orientation = Double.parseDouble(orientationElement.getTextContent());

        double x = map_width - (Double.parseDouble(xElement.getTextContent())) / mapResolution;
        double z = map_height - (Double.parseDouble(zElement.getTextContent())) / mapResolution;
        double speed = Double.parseDouble(speedElement.getTextContent());

        List<Pedestrian> pedestrians = new LinkedList<>();
        Element pedestriansNode = (Element) eventNode.getElementsByTagName("pedestrians").item(0);

        int i = 1;
        if (Parameters.NUM_PEDESTRIAN > 0) {
            while (true) {
                Node pedestrianNode = pedestriansNode.getElementsByTagName("ped" + i).item(0);
                if (pedestrianNode == null) {
                    break;
                }

                pedestrians.add(new Pedestrian((Element) pedestrianNode));
                ++i;
            }
        }

        Element obstacleNodes = (Element) eventNode.getElementsByTagName("obstacle1").item(0);

        Obstacle obstacle = null;
        if(obstacleNodes != null){
            String obstacleContent = obstacleNodes.getTextContent();

            if(obstacleContent != null && (!obstacleContent.isEmpty())){
                obstacle = new Obstacle();
                obstacle.x = map_width - Float.parseFloat(obstacleContent.split(",")[0]) / mapResolution;
                obstacle.z = map_height - Float.parseFloat(obstacleContent.split(",")[1]) / mapResolution;

                float obstacleOrientation = Float.parseFloat(obstacleContent.split(",")[2]);
                float adjustedAngle = (obstacleOrientation + 90) % 360;

                obstacle.theta = (float) Math.toRadians(adjustedAngle);
            }
        }

        return new SubscribedValues(orientation, x, z, speed, pedestrians, obstacle, isCrossing);
    }

    @Override
    public String toString() {
        return x + ", " + z + ", " + orientation + ", " + speed;
    }
}