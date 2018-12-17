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

package eu.opends.traffic;


/**
 * 
 * @author Rafael Math
 */
public class TrafficCarData 
{
	private String name;
	private float mass;
	private float acceleration;
	private float decelerationBrake;
	private float decelerationFreeWheel;
	private boolean engineOn;
	private String modelPath;
	private FollowBoxSettings followBoxSettings;
	private boolean isSpeedLimitedToSteeringCar;
	
	
	public TrafficCarData(String name, float mass, float acceleration,	float decelerationBrake, 
			float decelerationFreeWheel, boolean engineOn, String modelPath, FollowBoxSettings followBoxSettings,
			boolean isSpeedLimitedToSteeringCar) 
	{
		this.name = name;
		this.mass = mass;
		this.acceleration = acceleration;
		this.decelerationBrake = decelerationBrake;
		this.decelerationFreeWheel = decelerationFreeWheel;
		this.engineOn = engineOn;
		this.modelPath = modelPath;
		this.followBoxSettings = followBoxSettings;
		this.isSpeedLimitedToSteeringCar = isSpeedLimitedToSteeringCar;
	}


	public String getName() {
		return name;
	}
	

	public float getMass() {
		return mass;
	}
	
	
	public void setMass(float mass)	{
		this.mass = mass;
	}


	public float getAcceleration() {
		return acceleration;
	}

	
	public void setAcceleration(float acceleration)	{
		this.acceleration = acceleration;
	}
	

	public float getDecelerationBrake() {
		return decelerationBrake;
	}

	
	public void setDecelerationBrake(float decelerationBrake) {
		this.decelerationBrake = decelerationBrake;
	}
	
	
	public float getDecelerationFreeWheel() {
		return decelerationFreeWheel;
	}

	
	public void setDecelerationFreeWheel(float decelerationFreeWheel) {
		this.decelerationFreeWheel = decelerationFreeWheel;
	}
	

	public boolean isEngineOn() {
		return engineOn;
	}

	
	public void setEngineOn(boolean engineOn) {
		this.engineOn = engineOn;
	}
	
	
	public String getModelPath() {
		return modelPath;
	}

	
	public void setModelPath(String modelPath) {
		this.modelPath = modelPath;
	}
	
	
	public FollowBoxSettings getFollowBoxSettings() {
		return followBoxSettings;
	}

	
	public boolean isSpeedLimitedToSteeringCar() {
		return isSpeedLimitedToSteeringCar;
	}
	

	public void setSpeedLimitedToSteeringCar(boolean isSpeedLimitedToSteeringCar) {
		this.isSpeedLimitedToSteeringCar = isSpeedLimitedToSteeringCar;
	}


	public String toXML()
	{
		
		String preferredSegments = "";
		
		if(!followBoxSettings.getPreferredSegmentsStringList().isEmpty())
		{
			preferredSegments = "\t\t\t<segments>";
			for(String segmentString : followBoxSettings.getPreferredSegmentsStringList())
				preferredSegments += "<segment ref=\"" + segmentString + "\"/>";
			preferredSegments += "</segments>\n";
		}
		
		return "\t\t<vehicle id=\"" + name + "\">\n" +
			   "\t\t\t<modelPath>" + modelPath + "</modelPath>\n" + 
			   "\t\t\t<mass>"+ mass + "</mass>\n" + 
			   "\t\t\t<acceleration>"+ acceleration + "</acceleration>\n" + 
			   "\t\t\t<decelerationBrake>"+ decelerationBrake + "</decelerationBrake>\n" + 
			   "\t\t\t<decelerationFreeWheel>"+ decelerationFreeWheel + "</decelerationFreeWheel>\n" + 
			   "\t\t\t<maxSpeed>"+ followBoxSettings.getMaxSpeed() + "</maxSpeed>\n" + 							  
			   "\t\t\t<giveWayDistance>"+ followBoxSettings.getGiveWayDistance() + "</giveWayDistance>\n" + 
			   "\t\t\t<intersectionObservationDistance>"+ followBoxSettings.getIntersectionObservationDistance() + "</intersectionObservationDistance>\n" + 
			   "\t\t\t<minIntersectionClearance>"+ followBoxSettings.getMinIntersectionClearance() + "</minIntersectionClearance>\n" + 
			   "\t\t\t<engineOn>"+ engineOn + "</engineOn>\n" + 
			   "\t\t\t<minDistanceFromPath>"+ followBoxSettings.getMinDistance() + "</minDistanceFromPath>\n" + 
			   "\t\t\t<maxDistanceFromPath>"+ followBoxSettings.getMaxDistance() + "</maxDistanceFromPath>\n" + 
			   "\t\t\t<neverFasterThanSteeringCar>"+ isSpeedLimitedToSteeringCar + "</neverFasterThanSteeringCar>\n" + 
			   "\t\t\t<startWayPoint>"+ followBoxSettings.getStartWayPointID() + "</startWayPoint>\n" + 
			   preferredSegments + 
			   "\t\t</vehicle>";	
	}
	

}
