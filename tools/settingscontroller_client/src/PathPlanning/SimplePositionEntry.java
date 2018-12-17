package settingscontroller_client.src.PathPlanning;

import java.io.Serializable;
import java.util.Objects;

/**
 * Simplified position entry WIHTOUT orientation
 */
public class SimplePositionEntry implements Serializable{
    short x;
    short z;

    public SimplePositionEntry(short x, short z) {
        this.x = x;
        this.z = z;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        SimplePositionEntry that = (SimplePositionEntry) o;
        return x == that.x &&
                z == that.z;
    }

    @Override
    public int hashCode() {
        return Objects.hash(x, z);
    }
}

