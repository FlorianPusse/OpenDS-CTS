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

package eu.opends.trigger;

import eu.opends.basics.SimulationBasics;
import eu.opends.tools.PanelCenter;


/**
 * This class represents a WarningFrame trigger action. Whenever a collision
 * with a related trigger was detected, the warning frame will be flashing in 
 * the specified way.
 * 
 * @author Rafael Math
 */
public class WarningFrameTriggerAction extends TriggerAction 
{
	private int flashingInterval;
	private int duration;
	
	
	/**
	 * Creates a new WarningFrame trigger action instance, providing maximum
	 * number of repetitions, interval of flashing and duration. 
	 * 
	 * @param sim
	 * 			Simulator
	 * 
	 * @param delay
	 * 			Amount of seconds (float) to wait before the TriggerAction will be executed.
	 * 
	 * @param maxRepeat
	 * 			Maximum number how often the trigger can be hit.
	 * 
	 * @param flashingInterval
	 * 			Interval (ms) of flashing.
	 * 
	 * @param duration
	 * 			Duration (ms) of flashing.
	 */
	public WarningFrameTriggerAction(SimulationBasics sim, float delay, int maxRepeat, int flashingInterval, int duration) 
	{
		super(delay, maxRepeat);
		this.flashingInterval = flashingInterval;
		this.duration = duration;
	}

	
	/**
	 * Starts the flashing of the warning frame 
	 */
	@Override
	protected void execute() 
	{
		if(!isExceeded())
		{
			PanelCenter.showWarningFrame(true, flashingInterval, duration);			
			updateCounter();
		}
	}

}
