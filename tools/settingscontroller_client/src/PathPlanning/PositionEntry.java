package settingscontroller_client.src.PathPlanning;

import java.io.Serializable;

/**
 * A position entry used for the discretized state
 * of the path planner
 */
public class PositionEntry implements Serializable{
    float x;
    float z;
    float theta;

    public PositionEntry(HybridAStarPhysical.DiscreteSearchState d) {
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

