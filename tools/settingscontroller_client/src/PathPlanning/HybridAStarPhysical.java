package settingscontroller_client.src.PathPlanning;

import com.jme3.math.FastMath;
import eu.opends.main.Simulator;
import settingscontroller_client.src.Parameters;
import settingscontroller_client.src.TrafficObject.Pedestrian;
import settingscontroller_client.src.Util.SinCosLookupTable;
import settingscontroller_client.src.Util.Util;

import javax.imageio.ImageIO;
import javax.swing.*;
import java.awt.*;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.util.*;
import java.util.List;
import static settingscontroller_client.src.Parameters.*;

/**
 * Hybrid A* used for path planning
 */
@SuppressWarnings("Duplicates")
public class HybridAStarPhysical {
    /**
     * The star point of the search
     */
    private ContinuousSearchState start;

    /**
     * Endpoint of the search
     */
    private ContinuousSearchState goal;


    /**
     * Speed of the car
     */
    private float velocity = 1.05f;

    /**
     * Time to drive of the car
     */
    private float delta_t = 1;

    /**
     * The distance the car is driving for
     */
    float d;

    /**
     * Radiant of possible angles of the car
     */
    float[] radAngles;

    /**
     * Turning angle of the car given angles and distance
     */
    float[] turningAngles;

    /**
     * Costmap of the environment
     */
    public static short[][] costMap = null;

    /**
     * Non-holonomic without obstacles heuristic
     */
    public static short[][][] nonHolonomicShortestPaths = null;

    /**
     * Holonomic with obstacles heuristic
     */
    public static Dijkstra holonomicWithObstacles;

    /**
     * Lookup table for sin and cos function for faster computation
     */
    public static SinCosLookupTable lookupTable = new SinCosLookupTable();

    /**
     * If the car is allowed to drive backward in the search
     */
    private boolean allowBackward;

    private static final float TWO_PI = 2*FastMath.PI;

    /**
     * Roates a point (x,z) around a center point (centerX,centerZ) with orientation theta
     * @param centerX X-coordinate of center point
     * @param centerZ Z-coordinate of the center point
     * @param theta The orientation to rotate for
     * @param x X-coordinate of the point to rotate
     * @param z Z-coordinate of the point to rotate
     * @return
     */
    public static float[] rotatePosition(float centerX, float centerZ, float theta, float x, float z){
            float tempX = x - centerX;
            float tempZ = z - centerZ;

            float rotatedX = tempX * lookupTable.cos(theta) - tempZ * lookupTable.sin(theta);
            float rotatedY = tempX * lookupTable.sin(theta) + tempZ * lookupTable.cos(theta);

            return new float[]{rotatedX + centerX,rotatedY + centerZ};
    }

    /**
     * Forwards the car at point (x,z,theta) for distance meters with a turning angle beta
     * @param x X-coordinate of the car
     * @param z Z-coordinate of the car
     * @param theta Rotation of the car
     * @param turning_angle_beta Turning angle of the car
     * @param distance Forward distance of the car
     * @return
     */
    float[] step(float x, float z, float theta, float turning_angle_beta, float distance){
        float x_prime;
        float z_prime;
        float theta_prime;


        if (Math.abs(turning_angle_beta) < 0.0001) {
            x_prime = x + distance * lookupTable.cos(theta);
            z_prime = z + distance * lookupTable.sin(theta);
            theta_prime = (theta + turning_angle_beta) % TWO_PI;
            if (theta_prime < 0) {
                theta_prime = TWO_PI + theta_prime;
            }
        } else {
            float R = distance / turning_angle_beta;

            float cx = x - lookupTable.sin(theta) * R;
            float cz = z + lookupTable.cos(theta) * R;

            theta_prime = (theta + turning_angle_beta) % TWO_PI;
            if (theta_prime < 0) {
                theta_prime = TWO_PI + theta_prime;
            }

            x_prime = cx + lookupTable.sin(theta_prime) * R;
            z_prime = cz - lookupTable.cos(theta_prime) * R;
        }

        return new float[]{x_prime,z_prime,theta_prime};
    }

    /**
     * Points on the car frame with (x,z,theta)
     * @param x X-coordinate of the car
     * @param z Z-coordinate of the car
     * @param theta Rotation of the car
     * @return The points of the car frame
     */
    public static List<float[]> getCornerPositions(float x, float z, float theta) {
        List<float[]> cornerPositions = new LinkedList<>();
        cornerPositions.add(new float[]{x - (car_length/mapResolution) / 2.0f, z - (car_width/mapResolution) / 2.0f});
        cornerPositions.add(new float[]{x + (car_length/mapResolution) / 2.0f, z - (car_width/mapResolution) / 2.0f});
        cornerPositions.add(new float[]{x - (car_length/mapResolution) / 2.0f, z + (car_width/mapResolution) / 2.0f});
        cornerPositions.add(new float[]{x + (car_length/mapResolution) / 2.0f, z + (car_width/mapResolution) / 2.0f});

        cornerPositions.add(new float[]{x, z + (car_width/mapResolution) / 2.0f});
        cornerPositions.add(new float[]{x, z - (car_width/mapResolution) / 2.0f});
        cornerPositions.add(new float[]{x + (car_length/mapResolution) / 2.0f, z});
        cornerPositions.add(new float[]{x - (car_length/mapResolution) / 2.0f, z});

        cornerPositions.add(new float[]{x - (car_length/mapResolution) / 2.0f, z - (car_width/mapResolution) / 3.0f});
        cornerPositions.add(new float[]{x + (car_length/mapResolution) / 2.0f, z - (car_width/mapResolution) / 3.0f});
        cornerPositions.add(new float[]{x - (car_length/mapResolution) / 2.0f, z + (car_width/mapResolution) / 3.0f});
        cornerPositions.add(new float[]{x + (car_length/mapResolution) / 2.0f, z + (car_width/mapResolution) / 3.0f});

        cornerPositions.add(new float[]{x - (car_length/mapResolution) / 3.0f, z - (car_width/mapResolution) / 2.0f});
        cornerPositions.add(new float[]{x + (car_length/mapResolution) / 3.0f, z - (car_width/mapResolution) / 2.0f});
        cornerPositions.add(new float[]{x - (car_length/mapResolution) / 3.0f, z + (car_width/mapResolution) / 2.0f});
        cornerPositions.add(new float[]{x + (car_length/mapResolution) / 3.0f, z + (car_width/mapResolution) / 2.0f});

        for (float[] cornerPosition : cornerPositions) {
            float[] tmp = rotatePosition(x,z,theta,cornerPosition[0],cornerPosition[1]);
            cornerPosition[0] = tmp[0];
            cornerPosition[1] = tmp[1];
        }

        return cornerPositions;
    }

    /**
     * More car frame points with (x,z,theta)
     * @param x X-coordinate of the car
     * @param z Z-coordinate of the car
     * @param theta Rotation of the car
     * @return The points of the car frame
     */
    public static List<float[]> getAllCornerPositions(float x, float z, float theta) {
        List<float[]> cornerPositions = new LinkedList<>();
        cornerPositions.add(new float[]{x - (car_length/mapResolution) / 2.0f, z - (car_width/mapResolution) / 2.0f});
        cornerPositions.add(new float[]{x + (car_length/mapResolution) / 2.0f, z - (car_width/mapResolution) / 2.0f});
        cornerPositions.add(new float[]{x - (car_length/mapResolution) / 2.0f, z + (car_width/mapResolution) / 2.0f});
        cornerPositions.add(new float[]{x + (car_length/mapResolution) / 2.0f, z + (car_width/mapResolution) / 2.0f});

        cornerPositions.add(new float[]{x, z + (car_width/mapResolution) / 2.0f});
        cornerPositions.add(new float[]{x, z - (car_width/mapResolution) / 2.0f});
        cornerPositions.add(new float[]{x + (car_length/mapResolution) / 2.0f, z});
        cornerPositions.add(new float[]{x - (car_length/mapResolution) / 2.0f, z});

        cornerPositions.add(new float[]{x, z + (car_width/mapResolution) / 3.0f});
        cornerPositions.add(new float[]{x, z - (car_width/mapResolution) / 3.0f});
        cornerPositions.add(new float[]{x + (car_length/mapResolution) / 3.0f, z});
        cornerPositions.add(new float[]{x - (car_length/mapResolution) / 3.0f, z});

        cornerPositions.add(new float[]{x - (car_length/mapResolution) / 2.0f, z - (car_width/mapResolution) / 3.0f});
        cornerPositions.add(new float[]{x + (car_length/mapResolution) / 2.0f, z - (car_width/mapResolution) / 3.0f});
        cornerPositions.add(new float[]{x - (car_length/mapResolution) / 2.0f, z + (car_width/mapResolution) / 3.0f});
        cornerPositions.add(new float[]{x + (car_length/mapResolution) / 2.0f, z + (car_width/mapResolution) / 3.0f});

        cornerPositions.add(new float[]{x - (car_length/mapResolution) / 3.0f, z - (car_width/mapResolution) / 2.0f});
        cornerPositions.add(new float[]{x + (car_length/mapResolution) / 3.0f, z - (car_width/mapResolution) / 2.0f});
        cornerPositions.add(new float[]{x - (car_length/mapResolution) / 3.0f, z + (car_width/mapResolution) / 2.0f});
        cornerPositions.add(new float[]{x + (car_length/mapResolution) / 3.0f, z + (car_width/mapResolution) / 2.0f});

        cornerPositions.add(new float[]{x - (car_length/mapResolution) / 3.0f, z - (car_width/mapResolution) / 3.0f});
        cornerPositions.add(new float[]{x + (car_length/mapResolution) / 3.0f, z - (car_width/mapResolution) / 3.0f});
        cornerPositions.add(new float[]{x - (car_length/mapResolution) / 3.0f, z + (car_width/mapResolution) / 3.0f});
        cornerPositions.add(new float[]{x + (car_length/mapResolution) / 3.0f, z + (car_width/mapResolution) / 3.0f});

        cornerPositions.add(new float[]{x - (car_length/mapResolution) / 4.0f, z - (car_width/mapResolution) / 4.0f});
        cornerPositions.add(new float[]{x + (car_length/mapResolution) / 4.0f, z - (car_width/mapResolution) / 4.0f});
        cornerPositions.add(new float[]{x - (car_length/mapResolution) / 4.0f, z + (car_width/mapResolution) / 4.0f});
        cornerPositions.add(new float[]{x + (car_length/mapResolution) / 4.0f, z + (car_width/mapResolution) / 4.0f});

        cornerPositions.add(new float[]{x - (car_length/mapResolution) / 3.0f, z - (car_width/mapResolution) / 4.0f});
        cornerPositions.add(new float[]{x + (car_length/mapResolution) / 3.0f, z - (car_width/mapResolution) / 4.0f});
        cornerPositions.add(new float[]{x - (car_length/mapResolution) / 3.0f, z + (car_width/mapResolution) / 4.0f});
        cornerPositions.add(new float[]{x + (car_length/mapResolution) / 3.0f, z + (car_width/mapResolution) / 4.0f});

        cornerPositions.add(new float[]{x - (car_length/mapResolution) / 4.0f, z - (car_width/mapResolution) / 3.0f});
        cornerPositions.add(new float[]{x + (car_length/mapResolution) / 4.0f, z - (car_width/mapResolution) / 3.0f});
        cornerPositions.add(new float[]{x - (car_length/mapResolution) / 4.0f, z + (car_width/mapResolution) / 3.0f});
        cornerPositions.add(new float[]{x + (car_length/mapResolution) / 4.0f, z + (car_width/mapResolution) / 3.0f});

        for (float[] cornerPosition : cornerPositions) {
            float[] tmp = rotatePosition(x,z,theta,cornerPosition[0],cornerPosition[1]);
            cornerPosition[0] = tmp[0];
            cornerPosition[1] = tmp[1];
        }

        return cornerPositions;
    }

    /**
     * The four corner positions of the car (x,z,theta)
     * @param x X-coordinate of the car
     * @param z Z-coordinate of the car
     * @param theta Rotation of the car
     * @return The corner positions
     */
    public static List<float[]> getCornerPositions(float x, float z, float theta, float frontMargin, float sideMargin) {
        List<float[]> cornerPositions = new LinkedList<>();
        cornerPositions.add(new float[]{x - (car_length/mapResolution) / 2.0f, z - ((car_width + sideMargin)/mapResolution) / 2.0f});
        cornerPositions.add(new float[]{x + ((car_length + frontMargin)/mapResolution) / 2.0f, z - ((car_width + sideMargin)/mapResolution) / 2.0f});
        cornerPositions.add(new float[]{x - (car_length/mapResolution) / 2.0f, z + ((car_width + sideMargin)/mapResolution) / 2.0f});
        cornerPositions.add(new float[]{x + ((car_length + frontMargin)/mapResolution) / 2.0f, z + ((car_width + sideMargin)/mapResolution) / 2.0f});

        for (float[] cornerPosition : cornerPositions) {
            float[] tmp = rotatePosition(x,z,theta,cornerPosition[0],cornerPosition[1]);
            cornerPosition[0] = tmp[0];
            cornerPosition[1] = tmp[1];
        }

        return cornerPositions;
    }

    public static float[] rotateBack(float centerX, float centerZ, float theta, float length){
        return new float[]{centerX + length*lookupTable.cos(theta), centerZ + length*lookupTable.sin(theta)};
    }

    public static List<float[]> getCornerPositionsIncreased(float x, float z, float theta) {
        List<float[]> cornerPositions = new LinkedList<>();

        float car_length = Parameters.car_length + 1.0f;
        float car_width = Parameters.car_width + 0.4f;

        cornerPositions.add(new float[]{x - (car_length/mapResolution) / 2.0f, z - ((car_width)/mapResolution) / 2.0f});
        cornerPositions.add(new float[]{x + ((car_length)/mapResolution) / 2.0f, z - ((car_width)/mapResolution) / 2.0f});
        cornerPositions.add(new float[]{x - (car_length/mapResolution) / 2.0f, z + ((car_width)/mapResolution) / 2.0f});
        cornerPositions.add(new float[]{x + ((car_length)/mapResolution) / 2.0f, z + ((car_width)/mapResolution) / 2.0f});

        for (float[] cornerPosition : cornerPositions) {
            float[] tmp = rotatePosition(x,z,theta,cornerPosition[0],cornerPosition[1]);
            cornerPosition[0] = tmp[0];
            cornerPosition[1] = tmp[1];
        }

        return cornerPositions;
    }

    public List<float[]> getIncreasedCornerPositions(float x, float z, float theta) {
        float car_length = Parameters.car_length + 0.4f;
        float car_width = Parameters.car_width + 0.2f;

        List<float[]> cornerPositions = new LinkedList<>();
        cornerPositions.add(new float[]{x - (car_length/mapResolution) / 2.0f, z - (car_width/mapResolution) / 2.0f});
        cornerPositions.add(new float[]{x + (car_length/mapResolution) / 2.0f, z - (car_width/mapResolution) / 2.0f});
        cornerPositions.add(new float[]{x - (car_length/mapResolution) / 2.0f, z + (car_width/mapResolution) / 2.0f});
        cornerPositions.add(new float[]{x + (car_length/mapResolution) / 2.0f, z + (car_width/mapResolution) / 2.0f});

        cornerPositions.add(new float[]{x, z + (car_width/mapResolution) / 2.0f});
        cornerPositions.add(new float[]{x, z - (car_width/mapResolution) / 2.0f});
        cornerPositions.add(new float[]{x + (car_length/mapResolution) / 2.0f, z});
        cornerPositions.add(new float[]{x - (car_length/mapResolution) / 2.0f, z});

        cornerPositions.add(new float[]{x - (car_length/mapResolution) / 2.0f, z - (car_width/mapResolution) / 3.0f});
        cornerPositions.add(new float[]{x + (car_length/mapResolution) / 2.0f, z - (car_width/mapResolution) / 3.0f});
        cornerPositions.add(new float[]{x - (car_length/mapResolution) / 2.0f, z + (car_width/mapResolution) / 3.0f});
        cornerPositions.add(new float[]{x + (car_length/mapResolution) / 2.0f, z + (car_width/mapResolution) / 3.0f});

        cornerPositions.add(new float[]{x - (car_length/mapResolution) / 3.0f, z - (car_width/mapResolution) / 2.0f});
        cornerPositions.add(new float[]{x + (car_length/mapResolution) / 3.0f, z - (car_width/mapResolution) / 2.0f});
        cornerPositions.add(new float[]{x - (car_length/mapResolution) / 3.0f, z + (car_width/mapResolution) / 2.0f});
        cornerPositions.add(new float[]{x + (car_length/mapResolution) / 3.0f, z + (car_width/mapResolution) / 2.0f});

        for (float[] cornerPosition : cornerPositions) {
            float[] tmp = rotatePosition(x,z,theta,cornerPosition[0],cornerPosition[1]);
            cornerPosition[0] = tmp[0];
            cornerPosition[1] = tmp[1];
        }

        return cornerPositions;
    }

    public HybridAStarPhysical() {
        radAngles = new float[angles];
        turningAngles = new float[angles];

        for (int i = 0; i < angles; ++i) {
            radAngles[i] = (float) Math.toRadians(possibleSteeringAngles[i]);
        }
    }

    public void changeSettings(float velocity, float delta_t) {
        this.velocity = velocity;
        this.delta_t = delta_t;
    }

    /**
     * The discretized search state
     */
    class DiscreteSearchState implements Comparable<DiscreteSearchState> {
        float x; // discretized position in x space(g + h) - (s.g + s.h)
        float z; // discretized position in z space
        float theta;
        ContinuousSearchState state;

        public DiscreteSearchState(ContinuousSearchState state) {
            this.state = state;

            // calculate discretized states
            x = FastMath.floor(state.x / (discretization/mapResolution));
            z = FastMath.floor(state.z / (discretization/mapResolution));
            theta = FastMath.floor(state.theta / orientationDiscretization);
        }

        @Override
        public int compareTo(DiscreteSearchState s) {
            int t = (int) Math.signum((state.g + state.h) - (s.state.g + s.state.h));
            return t != 0 ? t : (int) Math.signum(s.state.g - state.g);
        }

        @Override
        public boolean equals(Object o) {
            if (this == o) return true;
            if (o == null || getClass() != o.getClass()) return false;

            DiscreteSearchState that = (DiscreteSearchState) o;

            if (Float.compare(that.x, x) != 0) return false;
            if (Float.compare(that.z, z) != 0) return false;
            return Float.compare(that.theta, theta) == 0;
        }

        @Override
        public int hashCode() {
            int result = (x != +0.0f ? Float.floatToIntBits(x) : 0);
            result = 31 * result + (z != +0.0f ? Float.floatToIntBits(z) : 0);
            result = 31 * result + (theta != +0.0f ? Float.floatToIntBits(theta) : 0);
            return result;
        }
    }

    /**
     * The continuous search state
     */
    public class ContinuousSearchState implements Comparable<ContinuousSearchState> {
        public float x; // discretized position in x space
        public float z; // discretized position in z space
        public float theta;

        float g; // accumulated total cost until to reach this state
        float h; // estimated rest cost

        ContinuousSearchState parent = null;
        public int parentAngle;
        public boolean reverse = false;

        public ContinuousSearchState(float x, float z, float theta) {
            this.x = x;
            this.z = z;
            this.theta = theta;
        }

        public ContinuousSearchState(float x, float z, float theta, float g) {
            this.x = x;
            this.z = z;
            this.theta = theta;
            this.g = g;
            this.h = h(goal);
        }

        private float actionCost(int action, boolean reverse, float x_prime, float z_prime, float theta_prime) {

            // find closest values
            int closestX = Math.round(x_prime);
            int closestZ = Math.round(z_prime);
            int lowestReward = 0;
            if(closestX >= 0 && closestZ >= 0 && closestX < map_width && closestZ < map_height){
                lowestReward = costMap[closestZ][closestX];

                if(beliefImage != null){
                    int tmpReward = new Color(beliefImage.getRGB(closestX, closestZ)).getRed();
                    if(tmpReward < lowestReward){
                        lowestReward = tmpReward;
                    }
                }
            }

            if(lowestReward > 0){
                List<float[]> cornerPositions = getIncreasedCornerPositions(x_prime, z_prime, theta_prime);
                cornerPositions.addAll(getCornerPositions(x_prime, z_prime, theta_prime));

                for (float[] cornerPosition : cornerPositions) {
                    closestX = (int) Math.floor(cornerPosition[0]);
                    closestZ = (int) Math.floor(cornerPosition[1]);

                    int reward = 0;
                    if(cornerPosition[0] >= 0 && cornerPosition[1] >= 0 && cornerPosition[0] < map_width && cornerPosition[1] < map_height){
                        reward = costMap[closestZ][closestX];

                        if(beliefImage != null){
                            int tmpReward = new Color(beliefImage.getRGB(closestX, closestZ)).getRed();
                            if(tmpReward < lowestReward){
                                lowestReward = tmpReward;
                            }
                        }
                    }

                    if (reward < lowestReward) {
                        lowestReward = reward;
                    }

                    if(lowestReward == 0){
                        break;
                    }
                }
            }

            int absAngle = Math.abs(action);
            float steeringCost;

            if(absAngle <= 30){
                steeringCost = absAngle / 1200.0f;
            }else{
                steeringCost = absAngle / 600.0f;
            }

            steeringCost += Math.abs(action - parentAngle) / 120.0;

            float reverseCost = reverse ? 25.0f : 0f;
            return (256 - lowestReward) + steeringCost + reverseCost;
        }

        @Override
        public int compareTo(ContinuousSearchState s) {
            int t = (int) Math.signum((g + h) - (s.g + s.h));
            return t != 0 ? t : (int) Math.signum(s.g - g);
        }

        private int nonHolonomicWithoutObstacles(ContinuousSearchState goal){

            int direction = (int) Math.floor(theta / Math.toRadians(non_holonomic_direction_discretization));
            if(direction == 72){
                direction = 0;
            }

            if(goal.x <= x && goal.z <= z){
                // goal top-left
                short distanceX = (short) Math.round(((x - goal.x)*mapResolution) / non_holonomic_discretization);
                short distanceZ = (short) Math.round(((z - goal.z)*mapResolution) / non_holonomic_discretization);
                return nonHolonomicShortestPaths[distanceZ][distanceX][direction];
            }else if(goal.x >= x && goal.z <= z){
                // goal top-right
                short distanceX = (short) Math.round(((goal.x - x)*mapResolution) / non_holonomic_discretization);
                short distanceZ = (short) Math.round(((z - goal.z)*mapResolution) / non_holonomic_discretization);
                return nonHolonomicShortestPaths[distanceZ][distanceX][direction];
            }
            else if(goal.x <= x && goal.z >= z){
                // goal down-left
                short distanceX = (short) Math.round(((x - goal.x)*mapResolution) / non_holonomic_discretization);
                short distanceZ = (short) Math.round(((goal.z - z)*mapResolution) / non_holonomic_discretization);
                return nonHolonomicShortestPaths[distanceZ][distanceX][direction];
            }else if(goal.x >= x && goal.z >= z){
                // goal down-right
                short distanceX = (short) Math.round(((goal.x - x)*mapResolution) / non_holonomic_discretization);
                short distanceZ = (short) Math.round(((goal.z - z)*mapResolution) / non_holonomic_discretization);
                return nonHolonomicShortestPaths[distanceZ][distanceX][direction];
            }

            throw new RuntimeException("Unknown position");
        }

        float h(ContinuousSearchState goal) {
            if(samePosition(goal)){
                return 0;
            }

            //holonomicWithObstacles.beliefImage = beliefImage;
            final float holonomicWithObstaclesHeuristic = (holonomicWithObstacles.search(x, z, (short) goal.x, (short) goal.z)*10);
            final float euclideanDistanceHeuristic =
                    FastMath.sqrt(FastMath.pow(x - goal.x, 2)
                            + FastMath.pow(z - goal.z, 2));

            return Math.max(holonomicWithObstaclesHeuristic, euclideanDistanceHeuristic);

            //final float nonHolonomicWithoutObstaclesCost = (nonHolonomicWithoutObstacles(goal)*10);
            //final float nonHolonomic =  Math.max(nonHolonomicWithoutObstaclesCost,euclideanDistanceHeuristic);
            //return Math.max(nonHolonomic,holonomicWithObstaclesHeuristic); // / 10.0f;
        }

        float interMediateCost(float x, float z, float theta, int angle, int angleIndex, int steps, Map<DiscreteSearchState, ContinuousSearchState> aggregatedResults){
            float cost = 0;

            for(float i = 1.0f; i <= steps; ++i) {
                float dist = (d*i) / (steps+1);
                float turningAngle = (dist / total_length) * FastMath.tan(radAngles[angleIndex]);

                float[] tmp = step(x, z, theta, turningAngle, dist);
                //TODO float[] rotatedCarPosition = rotatePosition(tmp[0], tmp[1], tmp[2], tmp[0] + (1.5218339f / mapResolution), tmp[1]);
                float[] rotatedCarPosition = rotateBack(tmp[0], tmp[1], tmp[2], (1.5218339f / mapResolution));

                float tmpCost = actionCost(angle, false, rotatedCarPosition[0], rotatedCarPosition[1], tmp[2]);
                cost = Math.max(cost, tmpCost);

                if (rotatedCarPosition[0] >= 0 && rotatedCarPosition[1] >= 0 && rotatedCarPosition[0] < map_width && rotatedCarPosition[1] < map_height) {
                    ContinuousSearchState cst = new ContinuousSearchState(rotatedCarPosition[0], rotatedCarPosition[1], tmp[2], g + tmpCost);
                    if (cst.samePosition(goal)) {
                        cst.parent = this;
                        cst.parentAngle = angle;
                        DiscreteSearchState discreteTmp = new DiscreteSearchState(cst);
                        if (!aggregatedResults.containsKey(discreteTmp)) {
                            aggregatedResults.put(discreteTmp, cst);
                        } else {
                            ContinuousSearchState stored = aggregatedResults.get(discreteTmp);
                            if (cst.g + cst.h < stored.g + stored.h) {
                                aggregatedResults.put(discreteTmp, cst);
                            }
                        }
                    }
                }
            }

            return cost;
        }

        /**
         * Using simple motion model of:
         * x' = x + v*\delta t * cos(theta)
         * y' = y + v*\delta t * sin(theta)
         * \theta' = \theta + w*\delta t
         * <p>
         * where \theta is the orientation, v is the velocity, w is the turing angle (in
         */
        Map<DiscreteSearchState, ContinuousSearchState> expand() {

            Map<DiscreteSearchState, ContinuousSearchState> aggregatedResults = new HashMap<>();

            // new coordinates that will be calculated for each successor
            float x_prime;
            float z_prime;
            float theta_prime;

            float[] rotatedCarPosition = rotatePosition(x,z,theta,x - (1.5218339f/mapResolution),z);
            float x = rotatedCarPosition[0];
            float z = rotatedCarPosition[1];

            // generate actual successors
            for (int i = 0; i < angles; ++i) {
                int angle = possibleSteeringAngles[i];
                float turningAngle = turningAngles[i];

                float[] s = step(x,z,theta,turningAngle,d);
                x_prime = s[0];
                z_prime = s[1];
                theta_prime = s[2];

                rotatedCarPosition = rotateBack(x_prime, z_prime, theta_prime, (1.5218339f/mapResolution));
                x_prime = rotatedCarPosition[0];
                z_prime = rotatedCarPosition[1];

                if (x_prime >= 0 && z_prime >= 0 && x_prime < map_width && z_prime < map_height) {
                    float cost = actionCost(angle, false, x_prime, z_prime, theta_prime);

                    if(d > 2){
                        float maxIntermediateCost = interMediateCost(x,z,theta,angle, i, 3, aggregatedResults);
                        cost = Math.max(cost, maxIntermediateCost);
                    }

                    ContinuousSearchState succ = new ContinuousSearchState(x_prime, z_prime, theta_prime, g + cost);
                    DiscreteSearchState discreteSucc = new DiscreteSearchState(succ);

                    if (!aggregatedResults.containsKey(discreteSucc)) {
                        succ.parent = this;
                        succ.parentAngle = angle;
                        aggregatedResults.put(discreteSucc, succ);
                    } else {
                        ContinuousSearchState stored = aggregatedResults.get(discreteSucc);
                        if (succ.g + succ.h < stored.g + stored.h) {
                            succ.parent = this;
                            succ.parentAngle = angle;
                            aggregatedResults.put(discreteSucc, succ);
                        }
                    }
                }
            }

            if(allowBackward){
                float d = -HybridAStarPhysical.this.d;

                for (int i = 0; i < angles; ++i) {
                    int angle = possibleSteeringAngles[i];
                    float turningAngle = turningAngles[i];

                    float[] s = step(x,z,theta,turningAngle ,d);
                    x_prime = s[0];
                    z_prime = s[1];
                    theta_prime = s[2];

                    rotatedCarPosition = rotatePosition(x_prime,z_prime,theta_prime,x_prime + (1.5218339f/mapResolution),z_prime);
                    x_prime = rotatedCarPosition[0];
                    z_prime = rotatedCarPosition[1];

                    if (x_prime >= 0 && z_prime >= 0 && x_prime < map_width && z_prime < map_height) {
                        float cost = actionCost(angle, true, x_prime, z_prime, theta_prime);
                        // Obstacle: don't go there
                        if(cost >= 200){
                            continue;
                        }

                        ContinuousSearchState succ =
                                new ContinuousSearchState(x_prime, z_prime, theta_prime, g + cost);
                        DiscreteSearchState discreteSucc = new DiscreteSearchState(succ);

                        if (!aggregatedResults.containsKey(discreteSucc)) {
                            succ.parent = this;
                            succ.parentAngle = angle;
                            succ.reverse = true;
                            aggregatedResults.put(discreteSucc, succ);
                        } else {
                            ContinuousSearchState stored = aggregatedResults.get(discreteSucc);
                            if (succ.g + succ.h < stored.g + stored.h) {
                                succ.parent = this;
                                succ.parentAngle = angle;
                                succ.reverse = true;
                                aggregatedResults.put(discreteSucc, succ);
                            }
                        }
                    }
                }
            }

            return aggregatedResults;
        }

        @Override
        public String toString() {
            return "(" + x + ", " + z + ", " + theta + "), g:" + g + ", h: " + h;
        }

        public float distance(ContinuousSearchState s) {
            return FastMath.sqrt(FastMath.pow(x - s.x, 2) + FastMath.pow(z - s.z, 2));
        }

        public boolean samePosition(ContinuousSearchState s) {
            return distance(s) <= (R_GOAL/mapResolution);
        }

        @Override
        public boolean equals(Object o) {
            if (this == o) return true;
            if (o == null || getClass() != o.getClass()) return false;

            ContinuousSearchState that = (ContinuousSearchState) o;

            if (Float.compare(that.x, x) != 0) return false;
            if (Float.compare(that.z, z) != 0) return false;
            return Float.compare(that.theta, theta) == 0;
        }

        @Override
        public int hashCode() {
            int result = (x != +0.0f ? Float.floatToIntBits(x) : 0);
            result = 31 * result + (z != +0.0f ? Float.floatToIntBits(z) : 0);
            result = 31 * result + (theta != +0.0f ? Float.floatToIntBits(theta) : 0);
            return result;
        }
    }

    public List<Float> simpleStep(float x, float z, float theta, int angleIndex, float d, int repetitions){
        float x_prime;
        float z_prime;
        float theta_prime;

        List<Float> res = new LinkedList<>();
        res.add(x*mapResolution);
        res.add(z*mapResolution);
        res.add(theta);

        float[] rotatedCarPosition = rotatePosition(x,z,theta,x - (1.5218339f/mapResolution),z);
        x = rotatedCarPosition[0];
        z = rotatedCarPosition[1];

        for(int i = 1; i < repetitions; i++){
            float turningAngle = ((i*d) / total_length) * FastMath.tan(radAngles[angleIndex]);

            float[] s = step(x,z,theta,turningAngle,i*d);
            x_prime = s[0];
            z_prime = s[1];
            theta_prime = s[2];

            float[] rotateBack = rotateBack(x_prime,z_prime,theta_prime,(1.5218339f/mapResolution));
            res.add(rotateBack[0]*mapResolution);
            res.add(rotateBack[1]*mapResolution);
            res.add(theta_prime);
        }

        return res;
    }

    BufferedImage beliefImage = null;
    public ContinuousSearchState search(float startX, float startZ, float startTheta, float goalX, float goalZ, float goalTheta, boolean allowBackward, BufferedImage beliefImage) {
        this.beliefImage = beliefImage;
        return search(startX, startZ, startTheta, goalX, goalZ, goalTheta, allowBackward);
    }

    public int getPathStep(){
        float val = (velocity * delta_t) / (mapResolution);
        return (int) Math.ceil(val / 2) * 2;
    }

    /*
     <entry key="frontAxlePos">-1.092812</entry>
     <entry key="backAxlePos">1.5218339</entry>
    */

    final float total_length = (1.5218339f - (-1.092812f))/mapResolution;

    /**
     * Searches a path from (startX,startZ,startTheta) to (goalX,goalZ,goalTheta).
     * If allowBackward, the path can also contain backward parts
     */
    public ContinuousSearchState search(float startX, float startZ, float startTheta, float goalX, float goalZ, float goalTheta, boolean allowBackward) {
        this.allowBackward = allowBackward;

        d = getPathStep();
        // System.out.println("Step length: " + d);

        for (int i = 0; i < angles; ++i) {
            turningAngles[i] = (d / total_length) * FastMath.tan(radAngles[i]);
        }

        goal = new ContinuousSearchState(goalX, goalZ, goalTheta);
        start = new ContinuousSearchState(startX, startZ, startTheta, 0);

        float bestSeen = 100000000;
        Map<PositionEntry,ContinuousSearchState> openSet = new HashMap<>();
        Set<PositionEntry> closedSet = new HashSet<>();
        PriorityQueue<ContinuousSearchState> openList = new PriorityQueue<>();
        openList.add(start);

        while (!openList.isEmpty() && closedSet.size() < 400000) {
            ContinuousSearchState current = openList.poll();
            PositionEntry currPosition = new PositionEntry(new DiscreteSearchState(current));
            openSet.remove(currPosition);
            closedSet.add(currPosition);

            if (current.samePosition(goal)) {
                return current;
            }

            float d = current.distance(goal);
            if (d < bestSeen) {
                bestSeen = d;
            }

            Map<DiscreteSearchState, ContinuousSearchState> expandResults = current.expand();

            expandResults.forEach((discreteState, result) -> {
                PositionEntry positionEntry = new PositionEntry(discreteState);

                if(!closedSet.contains(positionEntry)){
                    ContinuousSearchState storedState = null;

                    if(openSet.containsKey(positionEntry)){
                        storedState = openSet.get(positionEntry);
                    }

                    if((storedState == null || result.g + result.h  < storedState.g + storedState.h )){
                        //pane.addOpen(result);
                        if(storedState != null){
                            openList.remove(storedState);
                        }
                        openSet.put(positionEntry, result);
                        openList.add(result);
                    }
                }
            });
        }

        return null;
    }

    public List<ContinuousSearchState> getPath(ContinuousSearchState goal) {
        List<ContinuousSearchState> path = new LinkedList<>();

        ContinuousSearchState current = goal;

        while (current != start) {
            path.add(0, current);
            if(current.parent == null){
                return null;
            }
            current = current.parent;
        }

        path.add(0, start);

        return path;
    }
}
