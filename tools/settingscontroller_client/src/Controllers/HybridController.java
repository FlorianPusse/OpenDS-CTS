package settingscontroller_client.src.Controllers;/*
 *  This file is part of OpenDS (Open Source Driving Simulator).
 *  Copyright (C) 2016 Rafael Math
 *
 *  OpenDS is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  OpenDS is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with OpenDS. If not, see <http://www.gnu.org/licenses/>.
 */

import settingscontroller_client.src.Actions.APPLAction;
import settingscontroller_client.src.Actions.AbstractAction;
import settingscontroller_client.src.Controllers.OpenDSConnection.SubscribedValues;
import settingscontroller_client.src.TrafficObject.Pedestrian;

import java.io.IOException;
import java.util.*;

import static java.lang.Math.pow;
import static settingscontroller_client.src.AccelerationType.ACCELERATE;
import static settingscontroller_client.src.Evaluation.Config.SimulationMode.TRAINING;
import static settingscontroller_client.src.Parameters.R_GOAL;
import static settingscontroller_client.src.Parameters.mapResolution;
import static settingscontroller_client.src.Util.RectangleHITArea.pedInArea;


@SuppressWarnings("ALL")
public class HybridController extends AbstractController {

    public HybridController(){
        usePath = true;
        observationDelimiter = ";";
        MAX_TRIALS = 8;
    }

    /**
     * Overrides observation function to include the obstacle in the scene
     */
    public double[] observation(SubscribedValues subscribedValues, float targetSpeed) {
        double[] obs = new double[6 + (subscribedValues.pedestrians != null ? subscribedValues.pedestrians.size() * 2 : 0)];
        obs[0] = subscribedValues.x * mapResolution;
        obs[1] = subscribedValues.z * mapResolution;
        obs[2] = targetSpeed * 0.2777778;

        if (subscribedValues.pedestrians != null) {
            int i = 0;
            for (Pedestrian p : subscribedValues.pedestrians) {
                obs[3 + i * 2] = p.x * mapResolution;
                obs[3 + i * 2 + 1] = p.z * mapResolution;
                ++i;
            }

        }

        if(subscribedValues.obstacle != null){
            obs[3 + (2*subscribedValues.pedestrians.size())] = subscribedValues.obstacle.x * mapResolution;
            obs[3 + (2*subscribedValues.pedestrians.size()) + 1] = subscribedValues.obstacle.z * mapResolution;
            obs[3 + (2*subscribedValues.pedestrians.size()) + 2] = subscribedValues.obstacle.theta;
        }else{
            obs[3 + (2*subscribedValues.pedestrians.size())] = 0;
            obs[3 + (2*subscribedValues.pedestrians.size()) + 1] = 0;
            obs[3 + (2*subscribedValues.pedestrians.size()) + 2] = 0;
        }

        return obs;
    }

    public static void main(String[] args) throws InterruptedException, IOException {
        new HybridController().initController();

        while (true) {
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    /**
     * Overrides reward function to work exactly as the APPL reward
     */
    @Override
    public Reward calculateReward(SubscribedValues currentState, AbstractAction lastRelativeAction_, SubscribedValues lastState) {
        APPLAction lastRelativeAction = (APPLAction) lastRelativeAction_;

        Reward r = new Reward();
        r.reward = 0;

        List<float[]> cornerPositions = planner.getCornerPositions((float) currentState.x, (float) currentState.z, (float) currentState.orientation);
        double totalMindist = Double.POSITIVE_INFINITY;
        if (currentState.speed > 0.3) {
            Set<Pedestrian> pedestriansHit = new HashSet<>();
            Set<Pedestrian> pedestriansAlmostHit = new HashSet<>();

            for (Pedestrian ped : currentState.pedestrians) {
                // check for collisions

                double minDist = Math.sqrt(Math.pow(currentState.x - ped.x, 2) + Math.pow(currentState.z - ped.z, 2)) * mapResolution;

                for (float[] position : cornerPositions) {
                    double dist = Math.sqrt(Math.pow(position[0] - ped.x, 2) + Math.pow(position[1] - ped.z, 2)) * mapResolution;
                    if (dist < minDist) {
                        minDist = dist;
                    }
                    if (dist < totalMindist) {
                        totalMindist = dist;
                    }
                }

                if (minDist == 0) {
                    minDist = 0.01;
                }

                double absSpeed = Math.abs(currentState.speed);

                // set front margin between 1-2m and side margin between 0.5-1m
                boolean pedHit = pedInArea(currentState.x, currentState.z, currentState.orientation, ped, 2, 1.2f);

                if (pedHit) {
                    //System.out.println("[Crash situation] Car speed: " + speed);

                    float pedCollisionReward = -0.2f + (float) (-1 * pow(0.5 + currentState.speed / MAX_SPEED, 1.4));

                    if (mode == TRAINING) {
                        r.terminal = true;
                    }

                    r.reward += pedCollisionReward;
                }
            }
        }

        double goalDist = Math.sqrt(Math.abs(Math.pow(currentState.x - goalX, 2) + Math.pow(currentState.z - goalZ, 2))) * mapResolution;
        // double maxDistance = 4935;
        // r.reward -= Math.pow(goalDist / maxDistance, 0.8) / 4.0;

        // Reward the car for reaching the goal
        if (goalDist <= R_GOAL + 3) {
            r.reward += 1;
            r.terminal = true;
        }

        // Penalize braking/accelleration actions to get a smoother ride
        if (lastRelativeAction != null && lastRelativeAction.acc == ACCELERATE) {
            r.reward -= 0.01;
        }

        float minSpeed = Math.min(targetSpeed, MAX_SPEED);
        r.reward += (0.5 * (targetSpeed - MAX_SPEED) / MAX_SPEED) / 100.0;

        return r;
    }

    public class ISDESPOTAnswer {
        String accelleration;
        Map<Integer,List<Double>> pedBeliefs = new HashMap<>();

        public ISDESPOTAnswer(String message){
            message = message.replace("\n","");
            String[] tmp = message.split(";");
            accelleration = tmp[0];

            try{
                for(int i = 1; i < tmp.length; ++i){
                    String pedString = tmp[i];
                    String[] splitPedString = pedString.split(",");

                    List<Double> beliefs = new LinkedList<>();
                    for(int j = 1; j < splitPedString.length; ++j){
                        beliefs.add(new Double(splitPedString[j]));
                    }

                    pedBeliefs.put(new Integer(splitPedString[0]),beliefs);
                }
            }catch (NumberFormatException nfe){
                pedBeliefs = null;
            }

        }
    }

    @Override
    public AbstractAction getAction(String answer, float plannedAngle) {
        ISDESPOTAnswer parsedAnswer = new ISDESPOTAnswer(answer);
        currentBelief = parsedAnswer.pedBeliefs;

        APPLAction a = new APPLAction(parsedAnswer.accelleration, plannedAngle);
        return a;
    }
}
