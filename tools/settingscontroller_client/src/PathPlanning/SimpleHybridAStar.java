package settingscontroller_client.src.PathPlanning;

import com.jme3.math.FastMath;
import settingscontroller_client.src.TrafficObject.Pedestrian;

import java.io.Serializable;
import java.util.*;

import static settingscontroller_client.src.PathPlanning.HybridAStarPhysical.rotatePosition;
import static settingscontroller_client.src.Parameters.*;

/**
 * Represents the simplified hybrid A* used for the
 * non-holonomic without obstacles heuristic
 * -> no obstacles in path search
 */
@SuppressWarnings("Duplicates")
public class SimpleHybridAStar {

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
    private float delta_t = 1.0f;

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

    public SimpleHybridAStar() {
        radAngles = new float[angles];
        turningAngles = new float[angles];

        for (int i = 0; i < angles; ++i) {
            radAngles[i] = (float) Math.toRadians(possibleSteeringAngles[i]);
        }
    }

    public class PositionEntry implements Serializable {
        float x;
        float z;
        float theta;

        public PositionEntry(float x, float z, float theta) {
            this.x = x;
            this.z = z;
            this.theta = theta;
        }

        public PositionEntry(DiscreteSearchState d) {
            this.x = d.x;
            this.z = d.z;
            this.theta = d.theta;
        }

        @Override
        public boolean equals(Object o) {
            if (this == o) return true;
            if (o == null || getClass() != o.getClass()) return false;

            PositionEntry that = (PositionEntry) o;

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

    class DiscreteSearchState implements Comparable<DiscreteSearchState> {
        float x; // discretized position in x space
        float z; // discretized position in z space
        float theta;
        ContinuousSearchState state;

        public DiscreteSearchState(ContinuousSearchState state) {
            this.state = state;

            float discretization = 0.1f;
            float orientationDiscretization = 0.02f;

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

    class ContinuousSearchState implements Comparable<ContinuousSearchState> {
        float x; // discretized position in x space
        float z; // discretized position in z space
        float theta;

        float g; // accumulated total cost until to reach this state
        float h; // estimated rest cost

        ContinuousSearchState parent = null;
        int parentAngle;
        boolean reverse = false;

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
            return 1;
        }

        @Override
        public int compareTo(ContinuousSearchState s) {
            int t = (int) Math.signum((g + h) - (s.g + s.h));
            return t != 0 ? t : (int) Math.signum(s.g - g);
        }

        float h(ContinuousSearchState goal) {
            return FastMath.sqrt(FastMath.pow(x - goal.x, 2) + FastMath.pow(z - goal.z, 2)) - (R_GOAL / mapResolution);
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
                float turning_angle_beta = turningAngles[i];


                if (Math.abs(turning_angle_beta) < 0.0001) {
                    x_prime = x + d * FastMath.cos(theta);
                    z_prime = z + d * FastMath.sin(theta);
                    theta_prime = (theta + turning_angle_beta) % (2 * FastMath.PI);
                    if (theta_prime < 0) {
                        theta_prime = 2 * FastMath.PI + theta_prime;
                    }
                } else {
                    float R = d / turning_angle_beta;

                    float cx = x - FastMath.sin(theta) * R;
                    float cz = z + FastMath.cos(theta) * R;

                    theta_prime = (theta + turning_angle_beta) % (2 * FastMath.PI);
                    if (theta_prime < 0) {
                        theta_prime = 2 * FastMath.PI + theta_prime;
                    }

                    x_prime = cx + FastMath.sin(theta_prime) * R;
                    z_prime = cz - FastMath.cos(theta_prime) * R;
                }

                rotatedCarPosition = rotatePosition(x_prime,z_prime,theta_prime,x_prime + (1.5218339f/mapResolution),z_prime);
                x_prime = rotatedCarPosition[0];
                z_prime = rotatedCarPosition[1];

                if (x_prime >= 0 && z_prime >= 0) {
                    float cost = 1;

                    ContinuousSearchState succ =
                            new ContinuousSearchState(x_prime, z_prime, theta_prime, g + cost);
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

            if(velocity < 5){
                float d = -SimpleHybridAStar.this.d;

                for (int i = 0; i < angles; ++i) {
                    int angle = possibleSteeringAngles[i];
                    float turning_angle_beta = turningAngles[i];


                    if (Math.abs(turning_angle_beta) < 0.0001) {
                        x_prime = x + d * FastMath.cos(theta);
                        z_prime = z + d * FastMath.sin(theta);
                        theta_prime = (theta + turning_angle_beta) % (2 * FastMath.PI);
                        if (theta_prime < 0) {
                            theta_prime = 2 * FastMath.PI + theta_prime;
                        }
                    } else {
                        float R = d / turning_angle_beta;

                        float cx = x - FastMath.sin(theta) * R;
                        float cz = z + FastMath.cos(theta) * R;

                        theta_prime = (theta + turning_angle_beta) % (2 * FastMath.PI);
                        if (theta_prime < 0) {
                            theta_prime = 2 * FastMath.PI + theta_prime;
                        }

                        x_prime = cx + FastMath.sin(theta_prime) * R;
                        z_prime = cz - FastMath.cos(theta_prime) * R;
                    }

                    rotatedCarPosition = rotatePosition(x_prime,z_prime,theta_prime,x_prime + (1.5218339f/mapResolution),z_prime);
                    x_prime = rotatedCarPosition[0];
                    z_prime = rotatedCarPosition[1];

                    if (x_prime >= 0 && z_prime >= 0) {
                        float cost = 1;
                        // Obstacle: don't go there

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
            return distance(s) < (R_GOAL/ mapResolution);//&& Math.abs(theta - s.theta) < 0.1;
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

    ContinuousSearchState search(float startX, float startZ, float startTheta, float goalX, float goalZ, float goalTheta, List<Pedestrian> pedestrians) {
        //simpleAStar = new SimpleAStar();
        goal = new ContinuousSearchState(goalX, goalZ, goalTheta);
        start = new ContinuousSearchState(startX, startZ, startTheta, 0);

       /*
            <entry key="frontAxlePos">-1.092812</entry>
            <entry key="backAxlePos">1.5218339</entry>
        */

        float car_length = (1.5218339f - (-1.092812f))/mapResolution;

        d = (velocity * delta_t) / mapResolution;
        for (int i = 0; i < angles; ++i) {
            turningAngles[i] = (d / car_length) * FastMath.tan(radAngles[i]);
        }

        Map<PositionEntry,ContinuousSearchState> openSet = new HashMap<>();
        Set<PositionEntry> closedSet = new HashSet<>();
        PriorityQueue<ContinuousSearchState> openList = new PriorityQueue<>();
        openList.add(start);

        float bestSeen = 100000000;

        while (!openList.isEmpty()) {
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

    List<ContinuousSearchState> getPath(ContinuousSearchState goal) {
        List<ContinuousSearchState> path = new LinkedList<>();

        ContinuousSearchState current = goal;

        while (current != start) {
            path.add(0, current);
            current = current.parent;
        }

        path.add(0, start);

        return path;
    }
}
