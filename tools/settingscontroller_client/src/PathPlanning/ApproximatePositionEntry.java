package settingscontroller_client.src.PathPlanning;

import com.jme3.math.FastMath;

import java.io.Serializable;

/**
 * Position entry used during path planning.
 */
public class ApproximatePositionEntry implements Serializable{
    /**
     * X-coordinate of the car
     */
    float x;

    /**
     * Z-coordinate of the car
     */
    float z;

    /**
     * Roation of the car
     */
    float theta;

    public ApproximatePositionEntry(float x, float z, float theta) {
        this.x = x;
        this.z = z;
        this.theta = theta;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;

        ApproximatePositionEntry that = (ApproximatePositionEntry) o;

        if (FastMath.abs(that.x - x) > 1) return false;
        if (FastMath.abs(that.z - z) > 1) return false;
        return (FastMath.abs(that.theta - theta) < 0.2);
    }

    @Override
    public int hashCode() {
        int result = (x != +0.0f ? Float.floatToIntBits(x) : 0);
        result = 31 * result + (z != +0.0f ? Float.floatToIntBits(z) : 0);
        result = 31 * result + (theta != +0.0f ? Float.floatToIntBits(theta) : 0);
        return result;
    }
}

