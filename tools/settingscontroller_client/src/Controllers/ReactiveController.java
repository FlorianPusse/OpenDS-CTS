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
import settingscontroller_client.src.Parameters;
import settingscontroller_client.src.TrafficObject.Pedestrian;

import java.io.IOException;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;

import static settingscontroller_client.src.Util.MathUtil.euclideanDistance;
import static settingscontroller_client.src.Util.MathUtil.isMovingAway;
import static settingscontroller_client.src.Util.MathUtil.minPedDistance;

@SuppressWarnings("ALL")
public class ReactiveController extends AbstractController {

    public ReactiveController() {
        usePath = true;
        observationDelimiter = ";";
        MAX_TRIALS = 1;
    }

    public static void main(String[] args) throws InterruptedException, IOException {
        new ReactiveController().initController();

        while (true) {
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    /** Forces the reactive controller to drive if no ped is close enough **/
    boolean forceSpeedUp(SubscribedValues thisValue, SubscribedValues lastState) {
        if (!thisValue.isCrossing && targetSpeed < Parameters.MAX_SPEED) {
            return true;
        }

        if (minPedDistance(thisValue, false) > 3) {
            if (Math.abs(targetSpeed) < 10 && minPedDistance(thisValue, true) >= 10) {
                //System.out.println("Force speed up; No Ped in front");
                return true;
            } else if (Math.abs(targetSpeed) < 15 && minPedDistance(thisValue, true) >= 20) {
                //System.out.println("Force speed up; No Ped in front");
                return true;
            } else if (Math.abs(targetSpeed) < 30 && minPedDistance(thisValue, true) >= 25) {
                //System.out.println("Force speed up; No Ped in front");
                return true;
            }

            if (minPedDistance(thisValue, false) > 4 && lastState != null && targetSpeed < 10) {
                for (int i = 0; i < thisValue.pedestrians.size(); ++i) {
                    Pedestrian ped = thisValue.pedestrians.get(i);
                    if (euclideanDistance(thisValue.x, thisValue.z, ped.x, ped.z) < 10 && (!isMovingAway(thisValue, lastState, ped, lastState.pedestrians.get(i)))) {
                        return false;
                    }
                }
                return true;
            }
        }

        if (minPedDistance(thisValue, false) > 2 && targetSpeed == 0 && minPedDistance(thisValue, true) >= 10) {
            for (int i = 0; i < thisValue.pedestrians.size(); ++i) {
                Pedestrian ped = thisValue.pedestrians.get(i);
                Pedestrian pedLast = lastState.pedestrians.get(i);
                if (euclideanDistance(thisValue.x, thisValue.z, ped.x, ped.z) < 10 && (euclideanDistance(ped.x, ped.z, pedLast.x, pedLast.z) > 0.1)) {
                    return false;
                }
            }
            return true;
        }

        return false;
    }

    @Override
    public Reward calculateReward(SubscribedValues currentState, AbstractAction lastAction_, SubscribedValues lastState) {
        return new Reward();
    }

    /** The answer sent by the planner containing an action and a belief **/
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
