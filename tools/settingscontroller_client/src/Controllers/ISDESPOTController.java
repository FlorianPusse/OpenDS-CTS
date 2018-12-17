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

import static settingscontroller_client.src.AccelerationType.ACCELERATE;
import static settingscontroller_client.src.AccelerationType.DECELLERATE;
import static settingscontroller_client.src.Evaluation.Config.SimulationMode.TRAINING;
import static settingscontroller_client.src.Parameters.*;
import static settingscontroller_client.src.Util.MathUtil.linMap;
import static settingscontroller_client.src.Util.RectangleHITArea.pedInArea;

@SuppressWarnings("ALL")
public class ISDESPOTController extends AbstractController {

    public ISDESPOTController(){
        usePath = true;
        observationDelimiter = ";";
        MAX_TRIALS = 4;
    }

    public static void main(String[] args) throws InterruptedException, IOException {
        new ISDESPOTController().initController();

        while (true) {
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    @Override
    public Reward calculateReward(SubscribedValues currentState, AbstractAction lastRelativeAction_, SubscribedValues lastState) {
        APPLAction lastRelativeAction = (APPLAction) lastRelativeAction_;

        Reward r = new Reward();

        double goalDist = Math.sqrt(Math.abs(Math.pow(currentState.x - goalX, 2) + Math.pow(currentState.z - goalZ, 2))) * mapResolution;
        // Reward the car for reaching the goal
        if (goalDist <= R_GOAL + 3) {
            r.reward += GOAL_REWARD;
            r.terminal = true;
        }

        return r;
    }

    /** Parses the action sent by the planner **/
    @Override
    public AbstractAction getAction(String answer, float plannedAngle) {
        ISDESPOTAnswer parsedAnswer = new ISDESPOTAnswer(answer);
        currentBelief = parsedAnswer.pedBeliefs;

        APPLAction a = new APPLAction(parsedAnswer.accelleration, plannedAngle);
        return a;
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

}
