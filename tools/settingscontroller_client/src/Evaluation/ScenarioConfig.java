package settingscontroller_client.src.Evaluation;

/**
 * The scenario to simulate, represented by an integer
 */
public class ScenarioConfig {

    /**
     * Selected the scenario to simulate, represented by an integer,
     * @param side The side the pedestrian is crossing from
     * @param intersection The intersection type in the scenario
     * @param carTurning Whether the car is turning or not. If the car is turning,
     *                   the other parameters are ignored.
     */
    public static int toTrainingSet(PedestrianSide side, Intersection intersection,
                                    boolean carTurning){

        if(carTurning){
            return 14;
        }

        if(side == PedestrianSide.LEFT && intersection == Intersection.NONE){
            return 13;
        }

        if(side == PedestrianSide.LEFT && intersection == Intersection.APPROACHING){
            return 18;
        }

        if(side == PedestrianSide.RIGHT && intersection == Intersection.APPROACHING){
            return 19;
        }

        if(side == PedestrianSide.RIGHT && intersection == Intersection.LEAVING){
            return 20;
        }

        if(side == PedestrianSide.RIGHT && intersection == Intersection.NONE){
            return 0;
        }

        return 0;
    }

    /**
     * Returns the set number for multiple pedestrian crossing scenarios
     * @param sameSideCrossing Whether pedestrians should cross from the same side
     * @return The set number representing the chosen scenario
     */
    public static int toTrainingSetMultiplePedestrian(boolean sameSideCrossing){
        if(sameSideCrossing){
            return 10;
        }else{
            return 17;
        }
    }

    /**
     * Returns the set number of the three pedestrian crossing scenario
     * @return The set number representing the chosen scenario
     */
    public static int threePedestrianScenario(){
        return 21;
    }

    /**
     * Returns the set number of the four pedestrian crossing scenario
     * @return The set number representing the chosen scenario
     */
    public static int fourPedestrianScenario(){
        return 22;
    }

    /**
     * Possible locations of the obstacle in the scenario
     */
    public enum ObstaclePositions {
        /** No obstacle **/
        NONE,
        /** Obstacle on pavement **/
        ON_PAVEMENT,
        /** Obstalce on street **/
        ON_STREET
    }

    /**
     * Side the pedestrian is crossing from
     */
    public enum PedestrianSide {
        /** Pedestrian crossing from the left side **/
        LEFT,
        /** Pedestrians crossing from the right side **/
        RIGHT
    }

    /**
     * Intersection type in the scenario
     */
    public enum Intersection {
        /** Car approaching an intersection **/
        APPROACHING,
        /** Car leaving an intersection **/
        LEAVING,
        /** No intersection at all **/
        NONE
    }
}