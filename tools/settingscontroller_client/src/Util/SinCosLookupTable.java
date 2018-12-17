package settingscontroller_client.src.Util;

/**
 * A lookup table for sin and cos functions
 * for faster evaluation
 */
public class SinCosLookupTable {

    /**
     * Discretization of values to use
     */
    private double discretization = 0.001;

    /**
     * Stored cos values
     */
    private float[] cos = new float[(int) Math.ceil((2 * Math.PI) / discretization)];

    /**
     * Stored sin values
     */
    private float[] sin = new float[(int) Math.ceil((2 * Math.PI) / discretization)];

    /**
     * Static instance that can be used by all threads
     * asynchronously with ONLY reads
     */
    public static SinCosLookupTable table = new SinCosLookupTable();

    /**
     * Creates a lookup table with a discreatization between
     * values of discreatization
     */
    public SinCosLookupTable() {
        for (int i = 0; i*discretization < 2 * Math.PI; i++) {
            cos[i] = (float) Math.cos(i*discretization);
            sin[i] = (float) Math.sin(i*discretization);
        }
    }

    /**
     * Looks up sin(radAngle)
     * @param radAngle
     * @return sin(radAngle)
     */
    public float sin(double radAngle) {
        return sin[(int) (radAngle / discretization)];
    }

    /**
     * Looks up cos(radAngle)
     * @param radAngle
     * @return cos(radAngle)
     */
    public float cos(double radAngle) {
        return cos[(int) (radAngle / discretization)];
    }

}
