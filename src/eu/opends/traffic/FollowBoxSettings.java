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

import java.util.ArrayList;

import eu.opends.infrastructure.Segment;

/**
 * 
 * @author Rafael Math
 */
public class FollowBoxSettings 
{
	private ArrayList<Segment> preferredSegments;
	private ArrayList<String> preferredSegmentsStringList = new ArrayList<String>();
	private float minDistance;
	private float maxDistance;
	private float maxSpeed;
	private String startWayPointID;
	private float giveWayDistance;
	private float intersectionObservationDistance;
	private float minIntersectionClearance;
	public float originalSpeed;
	
	
	public FollowBoxSettings(ArrayList<Segment> preferredSegments, float minDistance, float maxDistance, 
			float maxSpeed, String startWayPointID, float giveWayDistance, float intersectionObservationDistance,
			float minIntersectionClearance) 
	{
		this.preferredSegments = preferredSegments;
		this.minDistance = minDistance;
		this.maxDistance = maxDistance;
		this.maxSpeed = maxSpeed;
		this.originalSpeed = maxSpeed;
		this.startWayPointID = startWayPointID;
		this.giveWayDistance = giveWayDistance;
		this.intersectionObservationDistance = intersectionObservationDistance;
		this.minIntersectionClearance = minIntersectionClearance;
		
		for(Segment segment : preferredSegments)
			preferredSegmentsStringList.add(segment.getName());
	}


	/**
	 * @return the preferred segments of this follow box
	 */
	public ArrayList<Segment> getPreferredSegments() 
	{
		return preferredSegments;
	}

	
	public ArrayList<String> getPreferredSegmentsStringList() 
	{
		return preferredSegmentsStringList;
	}
	
	
	public String getStartWayPointID() 
	{
		return startWayPointID;
	}
	
	
	public void setStartWayPointID(String startWayPointID)
	{
		this.startWayPointID = startWayPointID;
	}
	

	public int getStartWayPointIndex() 
	{
		for(int i=0; i<preferredSegments.size(); i++)
			if(preferredSegments.get(i).getName().equals(startWayPointID))
				return i;
		
		return -1;
	}


	public float getMinDistance() 
	{
		return minDistance;
	}
	
	
	public void setMinDistance(float minDistance)
	{
		this.minDistance = minDistance;
	}
	

	public float getMaxDistance()
	{
		return maxDistance;
	}

	
	public void setMaxDistance(float maxDistance)
	{
		this.maxDistance = maxDistance;
	}
	
	
	public float getMaxSpeed()
	{
		return maxSpeed;
	}
	
	
	public void setMaxSpeed(float maxSpeed)
	{
		this.maxSpeed = maxSpeed;
	}
	

	public float getGiveWayDistance() 
	{
		return giveWayDistance;
	}

	
	public void setGiveWayDistance(float giveWayDistance)
	{
		this.giveWayDistance = giveWayDistance;
	}
	

	public float getIntersectionObservationDistance()
	{
		return intersectionObservationDistance;
	}

	
	public void setIntersectionObservationDistance(float intersectionObservationDistance)
	{
		this.intersectionObservationDistance = intersectionObservationDistance;
	}
	

	public float getMinIntersectionClearance()
	{
		return minIntersectionClearance;
	}
	

	public void setMinIntersectionClearance(float minIntersectionClearance)
	{
		this.minIntersectionClearance = minIntersectionClearance;
	}

}
