package settingscontroller_client.src.Evaluation;

import settingscontroller_client.src.AccelerationType;

/**
 * The action of the POMDP
 */
public class Action {
    double angle;
    AccelerationType accelerationType;

    public Action(double angle, AccelerationType accelerationType) {
        this.angle = angle;
        this.accelerationType = accelerationType;
    }
}
