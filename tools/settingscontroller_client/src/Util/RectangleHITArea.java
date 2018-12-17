package settingscontroller_client.src.Util;

import com.jme3.math.Vector2f;
import settingscontroller_client.src.Evaluation.State;
import settingscontroller_client.src.PathPlanning.HybridAStarPhysical;
import settingscontroller_client.src.TrafficObject.Pedestrian;

import java.util.List;

/**
 * Defines the rectangle around the car used for
 * the near-miss area and the hit area
 */
public class RectangleHITArea {

    /** Checks weather a point M is in a rectangle
     *  defined by the corner points A,B,C,D
     * @return weather a point M is in a rectangle
     *  defined by the corner points A,B,C,D
     */
    static boolean pointInRectangle(Vector2f A, Vector2f B, Vector2f C, Vector2f D, Vector2f M) {
        Vector2f AB = B.subtract(A);
        Vector2f AM = M.subtract(A);
        Vector2f BC = C.subtract(B);
        Vector2f BM = M.subtract(B);
        float dotABAM = AB.dot(AM);
        float dotABAB = AB.dot(AB);
        float dotBCBM = BC.dot(BM);
        float dotBCBC = BC.dot(BC);
        return 0 <= dotABAM && dotABAM <= dotABAB && 0 <= dotBCBM && dotBCBM <= dotBCBC;
    }

    /**
     * Creates a 2D-vector from an array
     * @param arr The array to create the vector from
     * @return The created array
     */
    static Vector2f createVec(float[] arr){
        Vector2f v = new Vector2f();
        v.x = arr[0];
        v.y = arr[1];

        return v;
    }

    static Vector2f createVec(float x, float z){
        return new Vector2f(x,z);
    }

    /**
     * Checks if HIT there is a hit in the current state
     * with the pedestrian p
     * @param s State of the POMDP
     * @param p Pedestrian to check for
     * @return Whether there is a HIT
     */
    public static boolean isHIT(State s, Pedestrian p){
        return pedInArea(s.x, s.z, s.orientation, p, 0.5f, 0.5f);
    }

    /**
     * Checks if near-miss there is a hit in the current state
     * with the pedestrian p
     * @param s State of the POMDP
     * @param p Pedestrian to check for
     * @return Whether there is a HIT
     */
    public static boolean isNearMiss(State s, Pedestrian p){
        if(Math.abs(s.speed) <= 20){
            return pedInArea(s.x, s.z, s.orientation, p, 1, 0.75f);
        }else{
            return pedInArea(s.x, s.z, s.orientation, p, 1.5f, 1.0f);
        }
    }

    /**
     * Checks if the pedestrian p is in an area of the car
     * @param carX X-coordinate of the car
     * @param carZ Z-coordinate of the car
     * @param carTheta Orientation of the car
     * @param p Pedestrian to check
     * @param frontMargin Front-margin of the car
     * @param sideMargin Side-margin of the car
     * @return Whether p is in the area
     */
    public static boolean pedInArea(double carX, double carZ, double carTheta, Pedestrian p, float frontMargin, float sideMargin){
        List<float[]> cornerPositions = HybridAStarPhysical.getCornerPositions((float) carX, (float) carZ, (float) carTheta, frontMargin, sideMargin);
        Vector2f A = createVec(cornerPositions.get(0));
        Vector2f B = createVec(cornerPositions.get(1));
        Vector2f C = createVec(cornerPositions.get(3));
        Vector2f D = createVec(cornerPositions.get(2));

        return pointInRectangle(A,B,C,D, createVec((float) p.x, (float) p.z));
    }

}
