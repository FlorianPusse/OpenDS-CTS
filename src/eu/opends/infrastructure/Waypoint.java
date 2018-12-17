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

package eu.opends.infrastructure;

import java.util.ArrayList;

import com.jme3.math.FastMath;
import com.jme3.math.Vector3f;

/**
 * This class represents a way point for traffic participants. Each
 * way point consists of a name, position, and a list of segments 
 * leading to further way points. Optionally, a way point can have 
 * information about traffic lights, head light intensity, brake light
 * state, and whether there is a period of time the car has to wait.  
 * 
 * @author Rafael Math
 */
public class Waypoint
{
	private String name;
	private Vector3f position;
	private String trafficLightID;
	private Float headLightIntensity;
	private String turnSignal;
	private Boolean brakeLightOn;
	private Integer waitingTime;
	public ArrayList<Segment> outgoingSegmentList;
	
	// parent segment list contains all segments of which this way point is a via way point
	private ArrayList<Segment> parentSegmentList = new ArrayList<Segment>();
	
	
	/**
	 * Creates a new way point.
	 * 
	 * @param name
	 * 			Name of the way point.
	 * 
	 * @param position
	 * 			Position of the way point.
	 * 
	 * @param trafficLightID
	 * 			ID of related traffic light (if available, else null)
	 * 
	 * @param headLightIntensity
	 * 			Intensity change of head light (if available, else null)
	 * 
	 * @param turnSignal 
	 * 			State change of turn signal (if available, else null)
	 * 
	 * @param brakeLightOn
	 * 			State change of brake light (if available, else null)
	 * 	 
	 * @param waitingTime
	 * 			Amount of milliseconds to wait at this way point (if available, else null)
	 * 
	 * @param outgoingSegmentList
	 * 			List containing segments from this way point to other way points
	 */
	public Waypoint(String name, Vector3f position, String trafficLightID, Float headLightIntensity, String turnSignal, 
			Boolean brakeLightOn, Integer waitingTime, ArrayList<Segment> outgoingSegmentList) 
	{
		this.name = name;
		this.position = position;
		this.trafficLightID = trafficLightID;
		this.headLightIntensity = headLightIntensity;
		this.turnSignal = turnSignal;
		this.brakeLightOn = brakeLightOn;
		this.waitingTime = waitingTime;
		this.outgoingSegmentList = outgoingSegmentList;
	}

	
	/**
	 * Selects the next segment randomly from the way point's segment list 
	 * while taking into account the probability values of each segment.
	 *  	
	 * @param preferredSegments
	 * 			The list of segments which will be preferred when selecting
	 * 			the next segment at a branch. 
	 * 
	 * @return
	 * 			A randomly selected segment from the way point's segment list.
	 * 			null, if no segment available (way point == end point)
	 */
	public Segment getNextSegment(ArrayList<Segment> preferredSegments)
	{
		if(isEndPoint())
			return null;
		else
		{
			float sum = 0;
			for(Segment segment : outgoingSegmentList)
			{
				// check if one of the preferred segments is contained in outgoing segment list
				if(preferredSegments.contains(segment))
					return segment;
				
				// otherwise: get sum of all outgoing segments's probability (for normalizing)
				sum += segment.getProbability();
			}
			
			// select segment according to probability, e.g.:
			// segment1: p=0.2 -->  rdm == [0.0 - 0.2[
			// segment2: p=0.5 -->  rdm == [0.2 - 0.7[
			// segment3: p=0.3 -->  rdm == [0.7 - 1.0[
			float probability = 0;
			float rdm = FastMath.rand.nextFloat();
			for(Segment segment : outgoingSegmentList)
			{
				// sum up normalized probabilities 
				probability += segment.getProbability()/sum;

				// if summed up normalized probabilities exceed random, return current segment
				if(probability>rdm)
					return segment;
			}
			
			// the following code can only be reached in case of rounding imprecision
			// in this unlikely case: return last segment in the list
			return outgoingSegmentList.get(outgoingSegmentList.size()-1);
		}
	}
	

	/**
	 * Getter method for the name of the way point
	 * 
	 * @return
	 * 			Name of the way point
	 */
	public String getName() 
	{
		return name;
	}

	
	/**
	 * Getter method for the position of the way point
	 * 
	 * @return
	 * 			Position of the way point
	 */
	public Vector3f getPosition() 
	{
		return position;
	}

	public void setPosition(Vector3f position)
	{
		this.position.x = position.x;
		this.position.y = position.y;
		this.position.z = position.z;
	}
	

	/**
	 * Getter method for the ID of the related traffic light
	 * 
	 * @return
	 * 			ID of the related traffic light
	 */
	public String getTrafficLightID() 
	{
		return trafficLightID;
	}
	
	
	/**
	 * Getter method for the intensity of head light
	 * 
	 * @return
	 * 			intensity of head light
	 */
	public Float getHeadLightIntensity() 
	{
		return headLightIntensity;
	}
	
	
	/**
	 * Getter method for the state of turn signal
	 * 
	 * @return
	 * 			state of turn signal
	 */
	public String getTurnSignal() 
	{
		return turnSignal;
	}
	
	
	/**
	 * Getter method for the state of brake light
	 * 
	 * @return
	 * 			true, if brake light on
	 */
	public Boolean isBrakeLightOn()
	{
		return brakeLightOn;
	}

	
	/**
	 * Getter method for the rest time at this way point
	 * 
	 * @return
	 * 			amount of milliseconds to wait at this waypoint
	 */
	public Integer getWaitingTime()
	{
		return waitingTime;
	}
	
	
	/**
	 * Getter method for end point check
	 * 
	 * @return
	 * 			true, if waypoint has no outgoing segments
	 */
	public Boolean isEndPoint()
	{
		return outgoingSegmentList.isEmpty();
	}
	
	
	/**
	 * Setter method for parent segment. Having a parent segment 
	 * makes this way point a via way point.
	 *  
	 * @param parentSegment
	 * 			The parent segment.
	 */
	public void addParentSegment(Segment parentSegment)
	{
		parentSegmentList.add(parentSegment);		
	}
	

	/**
	 * Getter method for via point check
	 * 
	 * @return
	 * 			true, if way point is a via point (= has parent segment)
	 */
	public boolean isViaWP()
	{
		return !parentSegmentList.isEmpty();
	}
	
	
	/**
	 * Getter method for parent segment list
	 * 
	 * @return
	 * 			List of this way point's parent segments
	 */
	public ArrayList<Segment> getParentSegmentList()
	{
		return parentSegmentList;
	}
	
	
	/**
	 * Getter method for random parent segment of this way point
	 * 
	 * @return
	 * 			Random parent segment of this way point
	 */
	public Segment getRandomParentSegment()
	{
		int rand = FastMath.nextRandomInt(0,parentSegmentList.size()-1);
		return parentSegmentList.get(rand);
	}
	
	
	/**
	 * Getter method for the outgoing segment list
	 * 
	 * @return
	 * 			outgoing segment list
	 */
	public ArrayList<Segment> getOutgoingSegmentList()
	{
		return outgoingSegmentList;
	}
	
	
	/**
	 * String representation of a way point
	 * 
	 * @return
	 * 			String consisting of "name: position"
	 */
	@Override
	public String toString()
	{
		return name + ": " + position;
	}



}
