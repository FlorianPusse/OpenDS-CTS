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

import settingscontroller_client.src.Actions.AbstractAction;
import settingscontroller_client.src.Actions.DRLAction;
import settingscontroller_client.src.Controllers.OpenDSConnection.SubscribedValues;
import settingscontroller_client.src.TrafficObject.Pedestrian;

import java.io.IOException;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

import static settingscontroller_client.src.AccelerationType.ACCELERATE;
import static settingscontroller_client.src.AccelerationType.DECELLERATE;
import static settingscontroller_client.src.Evaluation.Config.SimulationMode.TRAINING;
import static settingscontroller_client.src.Parameters.*;
import static settingscontroller_client.src.Util.MathUtil.linMap;
import static settingscontroller_client.src.Util.RectangleHITArea.pedInArea;

/**
 * Controllers used by DRL cars
 */
@SuppressWarnings("ALL")
public class DiscretizedController extends AbstractController {

    public DiscretizedController(){
        useCarIntention = true;
        planAroundPedestrian = false;
        MAX_TRIALS = 1;
    }

    public static void main(String[] args) throws InterruptedException, IOException {
        new DiscretizedController().init();

        while (true) {
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    /**
     * Overrides observation function to include the orientation and speed of the car
     */
    @Override
    public double[] observation(SubscribedValues subscribedValues, float targetSpeed) {
        double[] obs = new double[4 + (subscribedValues.pedestrians != null ? subscribedValues.pedestrians.size() * 2 : 0)];
        obs[0] = subscribedValues.orientation / (2 * Math.PI);
        obs[1] = subscribedValues.x / map_width;
        obs[2] = subscribedValues.z / map_height;
        obs[3] = subscribedValues.speed / MAX_SPEED;

        if (subscribedValues.pedestrians != null) {
            int i = 0;
            for (Pedestrian p : subscribedValues.pedestrians) {
                obs[4 + i * 2] = (((subscribedValues.x - p.x) / map_width) + 1) / 2;
                obs[4 + i * 2 + 1] = (((subscribedValues.z - p.z) / map_height) + 1) / 2;
                ++i;
            }

        }

        return obs;
    }

    /**
     * Overrides the reward function to penalize steering actions
     */
    @Override
    public Reward calculateReward(SubscribedValues currentState, AbstractAction lastRelativeAction_, SubscribedValues lastState) {
        DRLAction lastRelativeAction = (DRLAction) lastRelativeAction_;

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
                boolean pedHit;
                if (absSpeed <= 20) {
                    pedHit = pedInArea(currentState.x, currentState.z, currentState.orientation, ped, 1, 0.75f);
                } else {
                    pedHit = pedInArea(currentState.x, currentState.z, currentState.orientation, ped, 2, 1.2f);
                }

                if (pedHit) {
                    // scale penalty by impact speed
                    double speedScale = linMap(0, MAX_SPEED, 0, 1, (float) Math.min(absSpeed, MAX_SPEED));
                    double pedCollisionReward = HIT_PENALTY * (speedScale + 0.1);

                    if (pedCollisionReward >= 700 && mode == TRAINING) {
                        r.terminal = true;
                    }

                    r.reward -= pedCollisionReward;
                }
            }
        }

        double goalDist = Math.sqrt(Math.abs(Math.pow(currentState.x - goalX, 2) + Math.pow(currentState.z - goalZ, 2))) * mapResolution;
        double maxDistance = 4935;
        r.reward -= Math.pow(goalDist / maxDistance, 0.8) * 1.2;

        if (stuckSteps >= STUCK_MAX) {
            stuckSteps = 0;
            r.terminal = true;
            r.reward -= 500;
        }

        // Penalize for hitting an obstacle
        double obstacleCost = obstacleCost(currentState.x, currentState.z, currentState.orientation);

        if (obstacleCost <= 100) {
            r.reward -= (obstacleCost / 20.0);
        } else if (obstacleCost <= 150) {
            r.reward -= (obstacleCost / 15.0);
        } else if (obstacleCost <= 200) {
            r.reward -= (obstacleCost / 10.0);
        } else {
            r.reward -= (obstacleCost / 0.22);
        }

        // "Heavily" penalize braking if you are already standing still
        if (lastRelativeAction != null && lastState.speed < 0.2 && lastRelativeAction.acc == DECELLERATE) {
            r.reward -= 1;
        }

        // Penalize braking/accelleration actions to get a smoother ride
        if (lastRelativeAction != null &&
                (lastRelativeAction.acc == ACCELERATE ||
                        lastRelativeAction.acc == DECELLERATE)) {
            r.reward -= 0.05;
        }

        // Penalize steering. The larger the angle the higher the penalty
        if (lastRelativeAction != null) {
            double originalAbsAngle = Math.abs(lastRelativeAction.drlAngle);
            r.reward -= Math.pow(originalAbsAngle, 1.3) / 2.0;
        }

        if (goalDist <= R_GOAL + 3) {
            // Reward the car for reaching the goal
            r.reward += GOAL_REWARD;
            r.terminal = true;

        }

        // "Normalize reward"
        r.reward /= 1000;

        return r;
    }

    @Override
    public AbstractAction getAction(String answer, float plannedAngle) {
        return new DRLAction(answer, (int) plannedAngle);
    }
}
