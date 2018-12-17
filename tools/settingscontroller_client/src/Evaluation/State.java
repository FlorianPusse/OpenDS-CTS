package settingscontroller_client.src.Evaluation;

import settingscontroller_client.src.TrafficObject.Pedestrian;

import java.util.List;
import java.util.Objects;

/**
 * The state of the POMDP
 */
public class State {
    public double orientation;
    public double x;
    public double z;
    public double speed;
    public double targetSpeed;
    public List<Pedestrian> pedestrians;

    public State(double orientation, double x, double z, double speed, double targetSpeed, List<Pedestrian> pedestrians) {
        this.orientation = orientation;
        this.x = x;
        this.z = z;
        this.speed = speed;
        this.targetSpeed = targetSpeed;
        this.pedestrians = pedestrians;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        State state = (State) o;
        return Double.compare(state.orientation, orientation) == 0 &&
                Double.compare(state.x, x) == 0 &&
                Double.compare(state.z, z) == 0 &&
                Double.compare(state.speed, speed) == 0 &&
                Objects.equals(pedestrians, state.pedestrians);
    }

    @Override
    public int hashCode() {
        return Objects.hash(orientation, x, z, speed, pedestrians);
    }
}