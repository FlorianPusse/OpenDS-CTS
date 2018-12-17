package settingscontroller_client.src.Actions;

import settingscontroller_client.src.AccelerationType;

/**
 * Represents an action that is used by the car controllers.
 * Can be implemented and use any representation of controlling
 * as long as it is possible to get/set an acceleration action and get
 * a steering angle.
 */
public interface AbstractAction {

    /**
     * Returns the acceleration action
     * @return The acceleration action
     */
    AccelerationType getAccelerationAction();

    /**
     * Sets the acceleration action
     * @param accelerationAction Sets an acceleration action
     */
    void setAccelerationAction(AccelerationType accelerationAction);

    /**
     * Returns the steering angle represented by this action
     * @return The steering angle
     */
    float getSteeringAngle();
}
