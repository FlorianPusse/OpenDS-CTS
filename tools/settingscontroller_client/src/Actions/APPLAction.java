package settingscontroller_client.src.Actions;

import settingscontroller_client.src.AccelerationType;

/**
 * The actions used for APPL based controllers -> IS-DESPOT-p, HyLEAP
 */
public class APPLAction implements AbstractAction{

    public float angle;
    public AccelerationType acc;

    /**
     * The action uses the output message s of the planner as acceleration action
     * Steering angle is used as given by the planner
     * @param s Output message of the planner
     * @param angle Steering angle as generated by the planner
     */
    public APPLAction(String s, float angle) {
        int tmp = Integer.parseInt(s.replace("\n",""));
        acc = AccelerationType.values()[tmp % 3];

        this.angle = angle;
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