package settingscontroller_client.src.Evaluation;
import settingscontroller_client.src.AccelerationType;
import settingscontroller_client.src.TrafficObject.Pedestrian;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.*;


import static settingscontroller_client.src.AccelerationType.ACCELERATE;
import static settingscontroller_client.src.AccelerationType.DECELLERATE;
import static settingscontroller_client.src.AccelerationType.MAINTAIN;
import static settingscontroller_client.src.Parameters.*;
import static settingscontroller_client.src.Util.RectangleHITArea.isHIT;
import static settingscontroller_client.src.Util.RectangleHITArea.isNearMiss;

/**
 * Represents the sequence of the POMDP created by the
 * car controllers in one scene. Captures relevant information of the
 * states and the actions.
 */
public class SimulationSequence {
    /**
     * Sequence of states of the POMDP created by the controller
     */
    List<State> stateSequence = new LinkedList<>();

    /**
     * Sequence of actions of the POMDP created by the controller
     */
    List<Action> actionSequence = new LinkedList<>();

    /**
     * A message to be added when evaluation the simulation sequnce
     */
    String message = null;

    /**
     * Returns whether the state sequence has already been filled
     * to a significant amount
     * @return Whether the sequence has been filled enough or not
     */
    public boolean emptySequence(){
        return stateSequence.size() <= 10;
    }

    /**
     * Resets the sequence, i.e. deletes saved states and actions
     */
    public void reset(){
        stateSequence = new LinkedList<>();
        actionSequence = new LinkedList<>();
    }

    /**
     * Creates a new state by manually adding the relevant state information to the simulation sequence
     * @param orientation
     * @param x
     * @param z
     * @param speed
     * @param targetSpeed
     * @param pedestrians
     */
    public void add(double orientation, double x, double z, double speed, double targetSpeed, List<Pedestrian> pedestrians){
        stateSequence.add(new State(orientation,x,z,speed,targetSpeed,pedestrians));
    }

    /**
     * Adds an action to the simulation sequence
     * @param angle
     * @param accType
     */
    public void addAction(double angle, int accType){
        actionSequence.add(new Action(angle, AccelerationType.values()[accType]));
    }

    Double crossingDistance = null;

    /**
     * Evaluates the simulation sequence with the given message.
     * @param message The message to print in front of the ouput.
     */
    public void evaluate(String message){
        this.message = message;
        evaluate();
    }

    /**
     * Evaluates the simulation sequence for the given evaluation measures.
     */
    public void evaluate(){

        // time taken to reach goal (or crash)
        int timeTaken = stateSequence.size();

        int accelerations = 0;
        long totalAngle = 0;

        for(int i = 0; i < Math.min(stateSequence.size(),actionSequence.size()); ++i){
            State s = stateSequence.get(i);
            Action a  = actionSequence.get(i);

            if(a.accelerationType == ACCELERATE && s.targetSpeed < MAX_SPEED
                    || a.accelerationType == DECELLERATE && s.speed > 0){
                ++accelerations;
            }

            totalAngle += Math.abs(a.angle);
        }

        boolean[] nearMisses = new boolean[stateSequence.get(0).pedestrians.size()];
        boolean[] crashes = new boolean[stateSequence.get(0).pedestrians.size()];

        double maxImpactSpeed = 0;

        for(State s : stateSequence){
            double speed = s.speed;

            if(speed > 0.5){
                int i = 0;
                for(Pedestrian p : s.pedestrians){
                    if(isHIT(s,p)){
                        crashes[i] = true;
                        if(speed > maxImpactSpeed){
                            maxImpactSpeed = speed;
                        }
                    }
                    if(isNearMiss(s,p)){
                        nearMisses[i] = true;
                    }
                    ++i;
                }
            }
        }

        int nrNearMisses = 0;
        for(boolean b : nearMisses){
            if(b){
                nrNearMisses++;
            }
        }
        int nrCrashes = 0;
        for(boolean b : crashes){
            if(b){
                nrCrashes++;
            }
        }

        if(message != null){
            System.out.println(message + "\tCrashes: " + nrCrashes + "\tNear misses: " + nrNearMisses + "\tAccelerations: " + accelerations + "\tTotal angle: "  + totalAngle +  "\tTime taken: " + timeTaken + "\tImpact Speed: " + maxImpactSpeed);
        }else if(crossingDistance == null){
            System.out.println("Crashes: " + nrCrashes + "\tNear misses: " + nrNearMisses + "\tAccelerations: " + accelerations + "\tTotal angle: "  + totalAngle +  "\tTime taken: " + timeTaken + "\tImpact Speed: " + maxImpactSpeed);
        }else{
            System.out.println("Crossing Distance: " + crossingDistance + "\tCrashes: " + nrCrashes + "\tNear misses: " + nrNearMisses + "\tAccelerations: " + accelerations + "\tTotal angle: "  + totalAngle +  "\tTime taken: " + timeTaken + "\tImpact Speed: " + maxImpactSpeed);
        }
    }
}
