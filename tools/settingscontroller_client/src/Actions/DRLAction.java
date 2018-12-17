package settingscontroller_client.src.Actions;

import settingscontroller_client.src.AccelerationType;

/**
 * The actions used for DRL based controllers -> NavA3C-p, ADRQN-p
 */
public class DRLAction implements AbstractAction{
    public enum SteeringType { SIMPLE, COMPLEX }

    public double drlAngle;
    public float angle;
    public AccelerationType acc;
    public static SteeringType type = SteeringType.COMPLEX;


    /**
     * The action uses the output message s of the planner as acceleration action
     * Steering angle is used as given by the planner to compute the difference
     * between planned angle and steering angle chosen by the neural network
     * @param s Output message of the neural network
     * @param plannedAngle Steering angle as generated by the planner
     */
    public DRLAction(String s, int plannedAngle) {
        int tmp = Integer.parseInt(s.replace("\n",""));

        int angleIndex = tmp / 3;
        int[] possibleAngles;
        if (type == SteeringType.COMPLEX){
            possibleAngles = new int[]{-20,-10,0,10,20};
        }else{
            possibleAngles = new int[]{-20,0,20};
        }

        int maxAngle = 40;

        drlAngle = possibleAngles[angleIndex];
        double totalAngle = drlAngle + plannedAngle;
        totalAngle = Math.max(Math.min(totalAngle,maxAngle),-maxAngle);

        angle = (float) totalAngle;
        acc = AccelerationType.values()[tmp % 3];
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
        return this.angle;
    }

    @Override
    public String toString() {
        return "Action{" +
                "angle=" + angle +
                ", acc=" + acc +
                '}';
    }
}
