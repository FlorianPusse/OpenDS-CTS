package settingscontroller_client.src.Util;

import com.jme3.math.Vector2f;
import settingscontroller_client.src.Controllers.AbstractController;
import settingscontroller_client.src.Controllers.OpenDSConnection.SubscribedValues;
import settingscontroller_client.src.TrafficObject.Pedestrian;

import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.toRadians;
import static settingscontroller_client.src.Parameters.IN_FRONT_ANGLE_DEG;
import static settingscontroller_client.src.Parameters.mapResolution;
import static settingscontroller_client.src.PathPlanning.HybridAStarPhysical.rotatePosition;

/**
 * Math functions needed for OpenDS-CTS
 */
public class MathUtil {

    /**
     * Checks if the pedestrian ped is currently moving away from the car
     * based on the last state lastPed and the last state of the car
     * lastVal
     * @param currVal State of the car
     * @param lastVal Last state of the car
     * @param lastPed Last state of the ped
     * @param ped State of the ped
     * @return Whether ped is moving away from the car
     */
    public static boolean isMovingAway(SubscribedValues currVal, SubscribedValues lastVal, Pedestrian lastPed, Pedestrian ped) {
        Vector2f currPos = new Vector2f((float) currVal.x, (float) currVal.z);
        Vector2f lastPos = new Vector2f((float) lastVal.x, (float) lastVal.z);

        Vector2f currPedPos = new Vector2f((float) ped.x, (float) ped.z);
        Vector2f lastPedPos = new Vector2f((float) lastPed.x, (float) lastPed.z);

        Vector2f v = currPos.subtract(lastPos).subtract(currPedPos.subtract(lastPedPos));
        Vector2f p = currPos.subtract(currPedPos);

        return v.dot(p) > 0;
    }

    /**
     * Checks if ped is in front of the car
     * @param currVal Current state of the car
     * @param ped Pedestrian to check
     * @return Whether ped is in front of the car
     */
    public static boolean inFront(SubscribedValues currVal, Pedestrian ped) {
        double d0 = euclideanDistance(currVal.x, currVal.z, ped.x, ped.z);
        if(d0 <= 0.7) return true;

        float[] forward_pos = rotatePosition((float) currVal.x,(float) currVal.z,(float) currVal.orientation,(float) currVal.x + 10.0f,(float) currVal.z);
        double d1 = euclideanDistance(currVal.x, currVal.z, forward_pos[0], forward_pos[1]);
        if(d1<=0) return false;

        double dot = dotProduct(forward_pos[0] - currVal.x, forward_pos[1] - currVal.z,
                ped.x - currVal.x, ped.z - currVal.z);

        double cosa = dot / (d0 * d1);

        if(!(cosa <= 1.0 + 1E-8 && cosa >= -1.0 - 1E-8)){
            assert(cosa <= 1.0 + 1E-8 && cosa >= -1.0 - 1E-8);
        }

        double in_front_angle_cos = cos(IN_FRONT_ANGLE_DEG / 180.0 * Math.PI);

        return cosa > in_front_angle_cos;
    }

    /**
     * Minimum distance between an pedestrian and the car
     * @param parsedValue State of the POMDP
     * @param onlyInFront Whether only pedestrians are considered
     *                    that are in front of the car
     * @return Minimum distance between car and any ped
     */
    public static double minPedDistance(SubscribedValues parsedValue, boolean onlyInFront) {
        double minDist = Double.POSITIVE_INFINITY;
        if (parsedValue.pedestrians != null) {
            for (Pedestrian p : parsedValue.pedestrians) {
                double dist = Math.sqrt(Math.pow(parsedValue.x - p.x, 2) +
                        Math.pow(parsedValue.z - p.z, 2));

                if(onlyInFront){
                    if (inFront(parsedValue, p) && dist < minDist) {
                        minDist = dist;
                    }
                }else{
                    if (dist < minDist) {
                        minDist = dist;
                    }
                }
            }
        }
        return minDist * mapResolution;
    }

    /**
     * Mapping a number X from range A-B to C-D linearly
     */
    // https://stackoverflow.com/questions/345187/math-mapping-numbers
    public static float linMap(float A, float B, float C, float D, float X) {
        return (X - A) / (B - A) * (D - C) + C;
    }

    /**
     * Euclidean distance between (x1,y1) and (x2,y2)
     * @return The euclidean distance between (x1,y1) and (x2,y2)
     */
    public static double euclideanDistance(double x1, double y1, double x2, double y2){
        return Math.sqrt(Math.pow(x1 - x2,2) + Math.pow(y1 - y2,2));
    }

    /**
     * Dot product between (x1,y1) and (x2,y2)
     * @return Dot product between (x1,y1) and (x2,y2)
     */
    public static double dotProduct(double x1, double y1, double x2, double y2){
        return x1*x2 + y1*y2;
    }

    /**
     * Creates a vector from an angle and a length
     * @param angle Angle to use
     * @param length Length to use
     * @return Created vector representing the angle and the length
     */
    public static Vector2f toAngle(double angle, double length) {
        Vector2f dir = new Vector2f((float) cos(toRadians(angle)), (float) sin(toRadians(angle)));
        return dir.mult((float) length);
    }

}
