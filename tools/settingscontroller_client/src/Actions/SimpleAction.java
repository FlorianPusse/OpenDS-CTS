package settingscontroller_client.src.Actions;

import settingscontroller_client.src.AccelerationType;

/**
 * A simple action.
 */
public class SimpleAction implements AbstractAction {

    public float angle;
    public AccelerationType acc;

    /**
     * Creates a simple action from a steering angle and an acceleration action
     * @param angle The steering angle to use
     * @param acc The acceleration action to use
     */
    public SimpleAction(float angle, AccelerationType acc){
        this.angle = angle;
        this.acc = acc;
    }

    /**
     * Assumes that s contains a comma separated steering angle
     * and an acceleration action. Creates a simple action from that
     * @param s Comma separated steering angle and acceleration action
     */
    public SimpleAction(String s){
        String[] tmp = s.split(",");
        this.angle = Float.parseFloat(tmp[0]);
        this.acc = AccelerationType.values()[Integer.parseInt(tmp[1])];
    }

    @Override
    public AccelerationType getAccelerationAction() {
        return acc;
    }

    @Override
    public void setAccelerationAction(AccelerationType accelerationAction) {
        this.acc = accelerationAction;
    }

    @Override
    public float getSteeringAngle() {
        return angle;
    }

    @Override
    public String toString() {
        return "Action{" +
                "angle=" + angle +
                ", acc=" + acc +
                '}';
    }
}
