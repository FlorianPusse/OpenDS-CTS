package settingscontroller_client.src.Evaluation;

import java.util.LinkedList;
import java.util.List;

/**
 * The config of the experiments
 */
public class Config {

    /**
     * The type of experiment to run
     */
    public enum ExperimentType {
        /** Pedestrian walking in ZigZag line **/
        ZIGZAG_FOLLOW,

        /** Multiple pedestrians **/
        MULTIPLE_PEDESTRIANS,

        /** One pedestrian **/
        SPEED_DISTANCE
    }

    /**
     * Mode of simulation. Cars can be trained or tested
     */
    public enum SimulationMode {
        /** Car is running in training mode **/
        TRAINING,

        /** Car is running in testing mode **/
        TESTING
    }

    /**
     * Creates the speeds used for experiments based on the mode
     * @param mode Mode to run
     * @return Speeds to use
     */
    public static List<Float> sampleMarginSpeeds(SimulationMode mode) {
        final double mean = 1.34;
        final double sd = 0.37;
        final double speedStepSize = 0.1f;

        List<Float> speedList = new LinkedList<>();

        if (mode == SimulationMode.TRAINING) {
            for (double i = mean - 2 * sd + speedStepSize; i < mean + 2 * sd; i += speedStepSize) {
                speedList.add((float) i * 3.6f);
            }
        }
        if (mode == SimulationMode.TESTING) {
            for (double i = mean - 3 * sd; i <= mean - 2 * sd; i += speedStepSize) {
                speedList.add((float) i * 3.6f);
            }

            for (double i = mean - 2 * sd + (speedStepSize / 2); i < mean + 2 * sd; i += speedStepSize) {
                speedList.add((float) i * 3.6f);
            }

            for (double i = mean + 2 * sd; i <= mean + 4 * sd; i += speedStepSize) {
                speedList.add((float) i * 3.6f);
            }
        }

        return speedList;
    }

    /**
     * Creates the distances used for experiments based on the mode
     * @param mode Mode to run
     * @return Distances to use
     */
    public static List<Float> sampleDistance(SimulationMode mode) {
        final int distanceStepSize = 1;

        List<Float> distanceList = new LinkedList<>();
        if (mode == SimulationMode.TRAINING) {
            for (float i = 5; i < 50; i += distanceStepSize) {
                distanceList.add(i);
            }
        }

        if(mode == SimulationMode.TESTING){
            for (float i = 5; i < 50; i += distanceStepSize) {
                distanceList.add(i + ((float) distanceStepSize) / 2.0f);
            }
        }

        return distanceList;
    }
}
