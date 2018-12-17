package settingscontroller_client.src.TrafficObject;

import org.w3c.dom.Element;

import java.util.Objects;

import static settingscontroller_client.src.Parameters.mapResolution;
import static settingscontroller_client.src.Parameters.map_height;
import static settingscontroller_client.src.Parameters.map_width;


/**
 * A pedestrian in the current scene
 */
public class Pedestrian {
    public double x;
    public double z;
    public double speed;

    /**
     * Creates a pedestrian from an XML element sent by OpenDS
     * @param pedestrianNode The XML element sent by OpenDS
     */
    public Pedestrian(Element pedestrianNode) {
        Element pedestrianPosition = (Element) pedestrianNode.getElementsByTagName("position").item(0);
        Element pedestrianSpeedElement = (Element) pedestrianNode.getElementsByTagName("speed").item(0);

        speed = Double.parseDouble(pedestrianSpeedElement.getTextContent());
        String[] posTmp = pedestrianPosition.getTextContent().split(",");

        if(Double.parseDouble(posTmp[0]) == 0){
            x = 0;
        }else{
            x = map_width - Double.parseDouble(posTmp[0])/mapResolution;
        }

        if(Double.parseDouble(posTmp[1]) == 0){
            z = 0;
        }else{
            z = map_height - Double.parseDouble(posTmp[1])/mapResolution;
        }
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        Pedestrian that = (Pedestrian) o;
        return Double.compare(that.x, x) == 0 &&
                Double.compare(that.z, z) == 0 &&
                Double.compare(that.speed, speed) == 0;
    }

    @Override
    public int hashCode() {

        return Objects.hash(x, z, speed);
    }
}