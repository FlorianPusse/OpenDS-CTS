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
import java.util.HashMap;
import java.util.List;

import com.jme3.math.FastMath;
import com.jme3.math.Spline;
import com.jme3.math.Vector3f;

import eu.opends.car.SteeringCar;
import eu.opends.main.Simulator;
import eu.opends.traffic.PhysicalTraffic;
import eu.opends.traffic.TrafficObject;

import com.jme3.math.Spline.SplineType;

/**
 * This class represents a segment of a traffic participant's pathway.
 * Each segment consists of a starting way point, an ending way point,
 * a speed value that the traffic object is trying to reach (by 
 * accelerating/braking) on this segment, and a probability value for
 * selecting this segment at the starting way point.
 * 
 * @author Rafael Math
 */
public class Segment 
{
	private Simulator sim;
	private boolean isInitialized = false;
	private String name;
	private String fromWaypointString;
	private ArrayList<String> viaWaypointListString;
	private String toWaypointString;
	private String leftNeighborString;
	private String rightNeighborString;
	private Waypoint fromWaypoint = null;
	private Waypoint toWaypoint = null;
	private Segment leftNeighbor = null;
	private Segment rightNeighbor = null;
	private float speed;
	private boolean isJump;
	private float probability;
	private Spline spline = new Spline();
	private ArrayList<String> prioritizedSegments;
	private ArrayList<Waypoint> viaWaypointList = new ArrayList<Waypoint>();
	private float curveTension;
	private HashMap<TrafficObject,Float> traveledDistanceMap = new HashMap<TrafficObject,Float>();
	
	
	/**
	 * Creates a new segment. The segment needs to be initialized with starting
	 * way point, ending way point, and a spline before it can be used. 
	 * 
	 * @param name
	 * 			The name of the segment

	 * @param fromWaypointString
	 * 			The string representation of the starting way point
	 * 
	 * @param viaWaypointListString
	 * 			The string representation of all way points between start and 
	 * 			end way point
	 * 
	 * @param toWaypointString
	 * 			The string representation of the ending way point
	 * 
	 * @param leftNeighborString
	 * 			The string representation of the neighbor segment to the left
	 * 
	 * @param rightNeighborString
	 * 			The string representation of the neighbor segment to the right
	 * 
	 * @param speed
	 * 			The speed of the car
	 * 
	 * @param isJump
	 * 			Indicates whether the segment will be "jumped" or moved along
	 * 
	 * @param probability
	 * 			The probability for selecting this segment
	 * 
	 * @param prioritizedSegments
	 * 			The segments with higher priority than this segment
	 * 
	 * @param curveTension
	 * 			The curve tension value of the segment (only used if no of way
	 * 			points is greater than 2)
	 */
	public Segment(String name, String fromWaypointString, ArrayList<String> viaWaypointListString, 
			String toWaypointString, String leftNeighborString, String rightNeighborString, float speed, 
			boolean isJump, float probability, ArrayList<String> prioritizedSegments, float curveTension) 
	{
		this.name = name;
		this.fromWaypointString = fromWaypointString;
		this.viaWaypointListString = viaWaypointListString;
		this.toWaypointString = toWaypointString;
		this.leftNeighborString = leftNeighborString;
		this.rightNeighborString = rightNeighborString;
		this.speed = speed;
		this.isJump = isJump;
		this.probability = probability;
		this.prioritizedSegments = prioritizedSegments;
		this.curveTension = curveTension;
	}

	
	/**
	 * Returns whether segment has been initialized.
	 * 
	 * @return
	 * 			true, if segment has been initialized.
	 */
	public boolean isInitialized()
	{
		return isInitialized;
	}
	
	
	/**
	 * Initializes a segment: add start way point, end way point, and spline
	 * 
	 * @param waypointMap
	 * 			The map of all available way points
	 * 
	 * @param sim
	 * 			The simulator
	 * 
	 * @return
	 * 			Returns whether the initialization was successful.
	 */
	public void init(HashMap<String, Waypoint> waypointMap, HashMap<String, Segment> segmentMap, Simulator sim)
	{
		if(true || !isInitialized)
		{
			this.sim = sim;
			this.spline = new Spline();

			// add start way point if exists
			if(waypointMap.containsKey(fromWaypointString))
				fromWaypoint = waypointMap.get(fromWaypointString);
	
			// add end way point if exists
			if(waypointMap.containsKey(toWaypointString))
				toWaypoint = waypointMap.get(toWaypointString);
			
			// add left neighbor segment if exists
			if(segmentMap.containsKey(leftNeighborString))
				leftNeighbor = segmentMap.get(leftNeighborString);
			
			// add right neighbor segment if exists
			if(segmentMap.containsKey(rightNeighborString))
				rightNeighbor = segmentMap.get(rightNeighborString);
			
			// add spline if start and end way points exist
			if(fromWaypoint != null && toWaypoint != null)
			{		
				// add start way point
				Vector3f startPos = fromWaypoint.getPosition();
				spline.addControlPoint(startPos);
	
				// Validates if strings represent actual way points
				for(String item : viaWaypointListString)
				{
					if(waypointMap.containsKey(item))
						viaWaypointList.add(waypointMap.get(item));
				}
				
				if(viaWaypointList.isEmpty())
				{
					// if no via way points --> connect start and end way point only
					spline.setType(SplineType.Linear);
				}
				else
				{
					// if via way points available...
					for(Waypoint viaWP : viaWaypointList)
					{
						// ...assign parent segment (=this) to all via way points
						viaWP.addParentSegment(this);
						
						// ...add all via way points to spline
						spline.addControlPoint(viaWP.getPosition());
					}
					
					spline.setType(SplineType.CatmullRom);
					spline.setCurveTension(curveTension);
				}
				
				// add end way point
				Vector3f endPos = toWaypoint.getPosition();
				spline.addControlPoint(endPos);
			}
			
			isInitialized = true;
		}		
	}
	
	
    /**
     * Interpolates the path giving the progress (0.0 - 1.0)    
     * 
     * @param progress 
     * 			The progress (0.0 - 1.0)  
     * 
     * @return
     * 			The position on the segment
     */
    public Vector3f interpolate(float progress)
    {
    	if(isInitialized)
    	{
	    	float sum = 0;
	    	float currentPos = progress * spline.getTotalLength();
	    	
	    	List<Float> segmentsLengthList = spline.getSegmentsLength();
	    	for(int i=0; i<segmentsLengthList.size(); i++)
	    	{
	    		float segmentLength = segmentsLengthList.get(i);
	    		if(sum + segmentLength >= currentPos)
	    		{
	    			float p = (currentPos - sum)/segmentLength;
	    			return spline.interpolate(p, i, null);
	    		}
	    		sum += segmentLength;
	    	}
    	}
    	return new Vector3f();
        //return spline.interpolate(progress, 0, null);
    }
    
    
    /**
     * Computes the heading at the given position of the segment in 2D space.
     * 
     * @param traveledDistance
     * 			Position to compute the heading at.
     * 
     * @return
     * 			Heading in radians.
     */
	public float getHeading(float traveledDistance) 
	{	
		float heading = 0;

		// if segment has been initialized, compute heading towards next way point
		if(isInitialized)
		{
			// compute heading by looking towards next way point from current position 
			Vector3f fromPosition = getFromViaWaypoint(traveledDistance).getPosition().clone();
			fromPosition.setY(0);
			
			Vector3f toPosition = getToViaWaypoint(traveledDistance).getPosition().clone();
			toPosition.setY(0);
			
			Vector3f drivingDirection = toPosition.subtract(fromPosition).normalize();

			// compute heading (orientation) from driving direction vector for
			// angle between driving direction and heading "0"
			float angle0  = drivingDirection.angleBetween(new Vector3f(0,0,-1));
			// angle between driving direction and heading "90"
			float angle90 = drivingDirection.angleBetween(new Vector3f(1,0,0));
			
			// get all candidates for heading
			// find the value from {heading1,heading2} which matches with one of {heading3,heading4}
			float heading1 = (2.0f * FastMath.PI + angle0)  % FastMath.TWO_PI;
			float heading2 = (2.0f * FastMath.PI - angle0)  % FastMath.TWO_PI;
			float heading3 = (2.5f * FastMath.PI + angle90) % FastMath.TWO_PI;
			float heading4 = (2.5f * FastMath.PI - angle90) % FastMath.TWO_PI;
			
			float diff_1_3 = FastMath.abs(heading1-heading3);
			float diff_1_4 = FastMath.abs(heading1-heading4);
			float diff_2_3 = FastMath.abs(heading2-heading3);
			float diff_2_4 = FastMath.abs(heading2-heading4);
			
			if((diff_1_3 < diff_1_4 && diff_1_3 < diff_2_3 && diff_1_3 < diff_2_4) ||
				(diff_1_4 < diff_1_3 && diff_1_4 < diff_2_3 && diff_1_4 < diff_2_4))
			{
				// if diff_1_3 or diff_1_4 are smallest --> the correct heading is heading1
				heading = heading1;
			}
			else
			{
				// if diff_2_3 or diff_2_4 are smallest --> the correct heading is heading2
				heading = heading2;
			}
		}
		
		return heading;
	}

	
	/**
	 * Getter method for the name of the segment
	 * 
	 * @return 
	 * 			The name
	 */
	public String getName()
	{
		return name;
	}


	/**
	 * Getter method for the string representation of the starting 
	 * way point of the segment
	 * 
	 * @return 
	 * 			The string representation of the starting way point
	 */
	public String getFromWaypointString()
	{
		return fromWaypointString;
	}


	/**
	 * Getter method for the string representation of the ending 
	 * way point of the segment
	 * 
	 * @return 
	 * 			The string representation of the ending way point
	 */
	public String getToWaypointString()
	{
		return toWaypointString;
	}


	/**
	 * Getter method for the target speed in this segment
	 * 
	 * @return
	 * 			The speed
	 */
	public float getSpeed() 
	{
		return speed;
	}


	/**
	 * Getter method indicating whether this segment is a jump
	 * 
	 * @return 
	 * 			Whether this segment is a jump
	 */
	public boolean isJump() 
	{
		return isJump;
	}
	

	/**
	 * Getter method for the selection probability of the segment
	 * 
	 * @return 
	 * 			The probability
	 */
	public float getProbability()
	{
		return probability;
	}

	
	/**
	 * Getter method for the spline
	 * 
	 * @return 
	 * 			The spline
	 */
	public Spline getSpline()
	{
		return spline;
	}


	/**
	 * Getter method for the length of the segment
	 * 
	 * @return 
	 * 			The length of the segment
	 */
	public float getLength() 
	{
		return spline.getTotalLength();
	}

	
	/**
	 * Getter method for the the from way point if available.
	 * 
	 * @return 
	 * 			The from way point if available, otherwise null.
	 */
	public Waypoint getFromWaypoint() 
	{
		return fromWaypoint;		
	}

	
	/**
	 * Getter method for the to way point if available.
	 * 
	 * @return 
	 * 			The to way point if available, otherwise null.
	 */
	public Waypoint getToWaypoint() 
	{
		return toWaypoint;		
	}
	

	/**
	 * Returns the left neighbor segment of this segment if available.
	 * 
	 * @return
	 * 			Left neighbor if available, otherwise null.
	 */
	public Segment getLeftNeighbor()
	{
		return leftNeighbor;
	}


	/**
	 * Returns the right neighbor segment of this segment if available.
	 * 
	 * @return
	 * 			Right neighbor if available, otherwise null.
	 */
	public Segment getRightNeighbor()
	{
		return rightNeighbor;
	}


	/**
	 * Getter method for list of segments that have higher 
	 * priority than this segment.
	 * 
	 * @return 
	 * 			List of segments.
	 */
	public ArrayList<String> getPriorityList() 
	{
		return prioritizedSegments;
	}


	/**
	 * Returns the number of curve segments (for visualization)
	 * 
	 * @return
	 * 			0 if segment is Linear, 10 if segment is CatmullRom
	 */
	public int getNoOfCurveSegments()
	{
		if(viaWaypointList.isEmpty())
			return 0;
		else
			return 10;
	}

	
	/**
	 * Returns whether the segment has via way points
	 * 
	 * @return
	 * 			true, if the segment has via way points
	 */
	public boolean hasViaWaypoints()
	{
		return !viaWaypointList.isEmpty();
	}

	
	/**
	 * Getter method for the previous via way point (if available)
	 * before or exactly at the given distance
	 * 
	 * @return
	 * 			The previous via way point (fromWaypoint if not available)
	 * 			before or exactly at the given distance
	 */
	private Waypoint getFromViaWaypoint(float traveledDistance)
	{
		// Iterate over segment's via way points till the given distance has been exceeded. 
		// Return the via way point at the previous position (index-1) or return fromWaypoint 
		// if previous via way point is fromWaypoint.
		float totalLenght = 0;
		for(int i=0 ; i<viaWaypointList.size(); i++)
		{
			totalLenght += spline.getSegmentsLength().get(i);
			if(totalLenght>traveledDistance)
			{
				if(i>0)
					return viaWaypointList.get(i-1);
				else
					return fromWaypoint;							
			}
		}
		
		return fromWaypoint;
	}
	
	
	/**
	 * Getter method for the next via way point (if available)
	 * beyond the given distance
	 * 
	 * @return
	 * 			The next via way point (toWaypoint if not available)
	 * 			beyond the given distance
	 */
	private Waypoint getToViaWaypoint(float traveledDistance)
	{
		// Iterate over segment's via way points till the given distance has been exceeded. 
		// Return the via way point at this position or return toWaypoint if last via way point has been passed.
		float totalLenght = 0;
		for(int i=0 ; i<viaWaypointList.size(); i++)
		{
			totalLenght += spline.getSegmentsLength().get(i);
			if(totalLenght>traveledDistance)
				return viaWaypointList.get(i);
		}
		
		return toWaypoint;
	}
	

	/**
	 * Setter method for traveled distance of a given traffic object, i.e. 
	 * the distance (in meters) the traffic object has traveled since the 
	 * FromWaypoint. This could be used if the segment will be entered 
	 * by the traffic object somewhere between its FromWaypoint and ToWaypoint.
	 * 
	 * @param traveledDistance
	 * 			The distance (in meters) traveled by the traffic object since 
	 * 			the FromWaypoint.
	 */
	public void setTraveledDistance(float traveledDistance, TrafficObject trafficObject) 
	{
		traveledDistanceMap.put(trafficObject, traveledDistance);
	}
	
	
	/**
	 * Getter method for traveled distance of a given traffic object, i.e.
	 * the distance (in meters) the traffic object has traveled since the 
	 * FromWaypoint. This could be used if the segment will be entered 
	 * by the traffic object somewhere between its FromWaypoint and ToWaypoint.
	 * After requesting this value, it will be reset to 0.
	 * 
	 * @return
	 * 			The distance (in meters) traveled the traffic object since
	 * 			the FromWaypoint.
	 */
	public float getTraveledDistance(TrafficObject trafficObject) 
	{
		float returnValue = 0;
				
		if(traveledDistanceMap.containsKey(trafficObject))
		{
			returnValue = traveledDistanceMap.get(trafficObject);
			
			// reset traveled distance (for next request)
			traveledDistanceMap.remove(trafficObject);
		}
		
		return returnValue;
	}


	/**
	 * Computes the closest distance from the given position to a point on the segment.
	 * Points on the segment to be considered must be located between minTraveledDistance 
	 * and maxTraveledDistance and have a distance of 1 meter to their neighbors.
	 * 
	 * @param position
	 * 			Position to compute distance to the segment
	 * 
	 * @param minTraveledDistance
	 * 			First point on the segment to be considered
	 * 			
	 * @param maxTraveledDistance
	 * 			Last point on the segment to be considered
	 * 
	 * @return
	 * 			Distance to the closest point on the segment
	 */
	public float distance(Vector3f position, float minTraveledDistance, float maxTraveledDistance)
	{
		if(minTraveledDistance > maxTraveledDistance)
		{
			float swap = maxTraveledDistance;
			maxTraveledDistance = minTraveledDistance;
			minTraveledDistance = swap;
		}
		
		float closestDistance = Float.POSITIVE_INFINITY;
		
		// position of the first point on the segment to be considered 
		float lowerBound = Math.max(0, Math.min(getLength(), minTraveledDistance));
		
		// position of the last point on the segment to be considered
		float upperBound = Math.max(0, Math.min(getLength(), maxTraveledDistance));
		
		// approach lower bound in steps of 1 meter towards upper bound
		while(lowerBound <= upperBound)
		{
			// compute the distance between position and the current point on the segment
			float distance = interpolate(lowerBound/getLength()).distance(position);
			if(distance < closestDistance)
				closestDistance = distance;
			
			// add 1 meter to the lower bound
			lowerBound += 1;
		}
		
		return closestDistance;
	}
	
	
	/**
	 * Checks whether the segment is clear of all traffic objects (cars and pedestrians)
	 * and the steering car (auto-pilot mode and human-control mode).
	 * 
	 * @param requestingTrafficObject
	 * 			Traffic object requesting this information (in order to prevent that own
	 * 			presence will lead to occupied segment)
	 * 
	 * @param traveledDistance
	 * 			Relative position on the segment (area before and after 30 meters will be 
	 * 			inspected only)
	 * 
	 * @return
	 * 			true, if segment is clear of all traffic objects and the steering car in
	 * 			the vicinity of the given position.
	 */
    public boolean isClear(TrafficObject requestingTrafficObject, float traveledDistance)
    {
    	if(!isInitialized)
    		return false;
    		
    	// check if any traffic object (car or pedestrian) is blocking the target segment
    	for(TrafficObject otherTrafficObject : sim.getPhysicalTraffic().getTrafficObjectList())
    	{
    		// exclude requesting traffic object
    		if(!otherTrafficObject.equals(requestingTrafficObject))
    		{
	    		Segment otherTrafficSegment = otherTrafficObject.getCurrentSegment();
	    		if(this.equals(otherTrafficSegment))
	    		{
	    			// check if traffic object is in a range of 40 meters before or 30 meters ahead of the 
	    			// entry point to the target segment
	    			float otherTraveledDistance = otherTrafficObject.getTraveledDistance();    			
	    			if(traveledDistance-10 < otherTraveledDistance && otherTraveledDistance < traveledDistance+60)
	    				return false;
	    		}
    		}
    	}
    	
		// check if steering car is blocking the target segment
    	SteeringCar car = sim.getCar();
		if(!car.equals(requestingTrafficObject))
		{
			if(car.isAutoPilot())
			{
	    		Segment otherTrafficSegment = car.getCurrentSegment();
	    		if(this.equals(otherTrafficSegment))
	    		{
	    			// check if follow box of steering car is in a range of 40 meters before or 30 meters ahead of the 
	    			// entry point to the target segment (if auto-pilot on)
	    			float otherTraveledDistance = car.getTraveledDistance();    			
	    			if(traveledDistance-10 < otherTraveledDistance && otherTraveledDistance < traveledDistance+60)
	    				return false;
	    		}
			}
			else
			{
				// check if human-controlled steering car is in a range of 40 meters before or 30 meters ahead of the 
    			// entry point to the target segment (if auto-pilot off)
				if(distance(car.getPosition(), traveledDistance-10, traveledDistance+60) < 1.5f)
					return false;
			}
		}

		return true;
	}


    /**
     * Computes distance between from way point and given via way point of this segment.
     *  
     * @param waypoint
     * 			Via way point
     * 
     * @return
     * 			Distance between from way point and given via way point of this segment.
     * 			If way point is not a via way point of this segment, 0 will be returned.
     */
	public float getViaWPPosition(Waypoint waypoint)
	{
		float sum = 0;
		
		if(viaWaypointList.contains(waypoint))
			for(int i=0 ; i<=viaWaypointList.lastIndexOf(waypoint); i++)
				sum += spline.getSegmentsLength().get(i);
			
		return sum;
	}

	
	public Float getCurveTension()
	{
		return curveTension;
	}


	public ArrayList<String> getViaWaypointStringList()
	{
		return viaWaypointListString;
	}


	public String getLeftNeighborString()
	{
		return leftNeighborString;
	}


	public String getRightNeighborString()
	{
		return rightNeighborString;
	}
}
