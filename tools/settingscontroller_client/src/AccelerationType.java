package settingscontroller_client.src;

/**
 * Possible acceleration types in the POMDP
 */
public enum  AccelerationType {
    /**
     * Accelerate the car by 5km/h
     */
    ACCELERATE,

    /**
     * Maintain the current speed
     */
    MAINTAIN,

    /**
     * Decelerate the car by 5km/h
     */
    DECELLERATE
}
