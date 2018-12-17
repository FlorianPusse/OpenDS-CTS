/*
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

package eu.opends.reactionCenter;

import com.jme3.input.InputManager;
import com.jme3.input.controls.JoyAxisTrigger;

import eu.opends.drivingTask.settings.SettingsLoader;
import eu.opends.drivingTask.settings.SettingsLoader.Setting;
import eu.opends.jasperReport.ReactionLogger;
import eu.opends.main.Simulator;

/**
 * 
 * @author Rafael Math
 */
public class SteeringReactionTimer extends ReactionTimer
{
	private InputManager inputManager;
	private ReactionListener reactionListener;
	
	
	public SteeringReactionTimer(Simulator sim, InputManager inputManager, ReactionListener reactionListener, 
			ReactionLogger reactionlogger, long experimentStartTime, String timerID, int index)
	{
		super(sim, reactionlogger, experimentStartTime, timerID, index);
		
		this.inputManager = inputManager;
		this.reactionListener = reactionListener;
	}

	
	public void setup(String newReactionGroupID, String newComment)
	{
		super.setup(newReactionGroupID, newComment);

        SettingsLoader settingsLoader = sim.getSettingsLoader();
        int steeringControllerID = settingsLoader.getSetting(Setting.Joystick_steeringControllerID, 0);
        int steeringAxis = settingsLoader.getSetting(Setting.Joystick_steeringAxis, 1);
        boolean invertSteeringAxis = settingsLoader.getSetting(Setting.Joystick_invertSteeringAxis, false);
        inputManager.addMapping("reaction_group_" + index, new JoyAxisTrigger(steeringControllerID, steeringAxis, invertSteeringAxis));
    	inputManager.addMapping("reaction_group_" + index, new JoyAxisTrigger(steeringControllerID, steeringAxis, !invertSteeringAxis));
    	
    	inputManager.addListener(reactionListener, "reaction_group_" + index, "failure_group_" + index);
        
		timerIsActive = true;
	}


	public void reportMissedReaction()
	{
		super.reportMissedReaction();
		
		inputManager.deleteMapping("reaction_group_" + index);
		inputManager.deleteMapping("failure_group_" + index);
	}
	

	public void update()
	{		
		super.update();
		
		if(timerIsActive && (correctReactionReported || failureReactionReported))
		{
			inputManager.deleteMapping("reaction_group_" + index);
			inputManager.deleteMapping("failure_group_" + index);
		}
	}

}
