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
import java.util.LinkedList;

import com.jme3.material.Material;
import com.jme3.math.ColorRGBA;
import com.jme3.math.FastMath;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.scene.Geometry;
import com.jme3.scene.shape.Box;

import eu.opends.car.SteeringCar;
import eu.opends.infrastructure.Segment;
import eu.opends.infrastructure.Waypoint;
import eu.opends.main.Simulator;
import eu.opends.tools.Util;
import org.lwjgl.Sys;


public class FollowBox 
{
    // Distance where traffic objects of this FollowBox (ownSafetyDistanceToIntersection) 
    // will start slowing down to full stop if priority road ahead with traffic in less 
    // than their safety distance (othersSafetyDistanceToIntersection).
    private float ownSafetyDistanceToIntersection = 15; 
    private float othersSafetyDistanceToIntersection = 15;

    // Minimum time clearance between two traffic objects meeting at the same intersection.
    // If minIntersectionClearance = 5, a traffic object must wait for 5 seconds to enter 
    // an intersection which has been entered by a higher prioritized traffic object before.
    private float minIntersectionClearance = 5;
    
    
	private Simulator sim;
	public FollowBoxSettings settings;
	private TrafficObject trafficObject;
	private boolean trafficObjectInitialized = false;
    private Geometry followBox;
    private Waypoint currentFromWaypoint;
    private Segment currentSegment = null;
    private Segment nextSegment = null;
    private float traveledDistance = 0;
    private Waypoint resetWaypoint = null;
    private ArrayList<Segment> preferredSegments = new ArrayList<Segment>();
    private float distanceToNextWP = Float.MAX_VALUE;
    private int helperSegmentCounter = 1;
    private boolean obstacleInTheWay = false;
	public Waypoint startWayPoint;


	public FollowBox(Simulator sim, TrafficObject trafficObject, FollowBoxSettings settings, boolean setToStartWayPoint)
	{		
		this.sim = sim;
		this.settings = settings;
		this.trafficObject = trafficObject;
		preferredSegments = settings.getPreferredSegments();
		ownSafetyDistanceToIntersection = settings.getGiveWayDistance();
		othersSafetyDistanceToIntersection = settings.getIntersectionObservationDistance();
		minIntersectionClearance = settings.getMinIntersectionClearance();

        createFollowBoxGeometry();
		
        String startWaypointID = settings.getStartWayPointID();
		startWayPoint = sim.getRoadNetwork().getWaypoint(startWaypointID);
        
        // check if startWaypointID exists
        if(startWayPoint != null)
        {
        	boolean success = setFollowBoxToWP(startWayPoint);

	        if(success && setToStartWayPoint)
	        	setToWayPoint(startWaypointID);
        }
        else
        	System.err.println("Could not set " + trafficObject.getName() + " to non-existing way point " + startWaypointID);
	}
	

	private void createFollowBoxGeometry()
	{
		Material materialGreen = new Material(sim.getAssetManager(), "Common/MatDefs/Misc/Unshaded.j3md");
        materialGreen.getAdditionalRenderState().setWireframe(true);
        materialGreen.setColor("Color", ColorRGBA.Green);
        
        followBox = new Geometry("followBox", new Box(0.3f, 0.3f, 0.3f));
        followBox.setMaterial(materialGreen);
        sim.getRoadNetwork().getDebugNode().attachChild(followBox);
	}


	public void update(float tpf, Vector3f trafficObjectCenterPos)
	{
		// skip update during pause
		if(sim.isPause() || currentSegment==null)
			return;

		computeDistanceToNextWP();
		
		// if traffic object has crashed / got stuck (i.e. not moved significantly for a longer time)
		if(hasCrashed())
			setToWayPoint(settings.getStartWayPointID());
		
		// if new WP to set traffic object available
		if(resetWaypoint != null)
		{
			//System.err.println("RESET");
			
			// set traffic object to new position
	        performWayPointChange(resetWaypoint);
	        resetWaypoint = null;
		}
		
		// wait one frame for traffic object to be placed, then start follow box
		if(!trafficObjectInitialized)
		{
			trafficObjectInitialized = true;
			return;
		}
	

		stopTrafficObject = false;
		float minDistance = settings.getMinDistance();
		float maxDistance = settings.getMaxDistance();
		float currentDistance = getCurrentDistance(trafficObjectCenterPos);
		boolean hasLostSteeringCar = hasLostSteeringCar(currentDistance);
		
		if(hasLostSteeringCar)
		{
			// move followBox not more than "maxDistance" meters when SteeringCar is lost in order to 
			// prevent missing the steering car (auto-pilot only!)
			traveledDistance += maxDistance;
		}
		else
		{
			// set limits
			currentDistance = Math.max(Math.min(maxDistance, currentDistance), minDistance);

			//maxDistance --> 0.0
			//minDistance --> 2.0 (speed of follow box may be up to two times faster than speed of car)
			float factor = (1.0f - ((currentDistance-minDistance)/(maxDistance-minDistance)))*2.0f;
			
			float speed = currentSegment.getSpeed()/3.6f;
			traveledDistance += speed*tpf*factor;
		}
		
		float progress = Math.max(0, Math.min(1, traveledDistance/currentSegment.getLength()));
		

		followBox.setLocalTranslation(currentSegment.interpolate(progress));


		boolean isRightHandTraffic = sim.getRoadNetwork().isRightHandTraffic();

		// if obstacle in the segment (lane) try to change to passing segment (lane)
		Segment passingSegment = getPassingSegment(isRightHandTraffic);
		if(obstacleInTheWay && passingSegment!=null && passingSegment.isClear(trafficObject, traveledDistance))
			changeToNeighborSegment(progress, passingSegment);
		
		// if regular segment (lane) is clear (again) try to change to regular segment (lane)
		Segment regularSegment = getRegularSegment(isRightHandTraffic);
		if(regularSegment!=null && regularSegment.isClear(trafficObject, traveledDistance))
			changeToNeighborSegment(progress, regularSegment);

		
		// way point reached
		if(progress >= 1.0f)
		{
			if(hasLostSteeringCar)
			{
				// keep follow box close to steering car when auto-pilot is switched off
				currentFromWaypoint = sim.getRoadNetwork().getRandomNearbyWaypoint(trafficObject.getPosition());
				currentSegment = currentFromWaypoint.getNextSegment(preferredSegments);
				nextSegment = currentSegment.getToWaypoint().getNextSegment(preferredSegments);
				traveledDistance = 0;
			}
			else
			{
				// usual handling when way point reached
				Waypoint currentToWaypoint = currentSegment.getToWaypoint();
				if(waitAtWP(currentToWaypoint))
				{
					stopTrafficObject = true;
				}
				else
				{
					//System.err.println("finished: " + currentSegment.getFromWaypointString() + 
					//		" --(" + currentSegment.getName() + ")--> " + currentSegment.getToWaypointString());
					
					if(nextSegment == null)
					{
						// stop traffic object at current position forever
						stopTrafficObject = true;
					}
					else
					{
						if(nextSegment.isJump())
						{
							//System.err.println("finished: " + nextSegment.getFromWaypointString() + 
							//		" --(" + nextSegment.getName() + ")--> " + nextSegment.getToWaypointString() + " (jump)");
							
							performWayPointChange(nextSegment.getToWaypoint());
						}
						else
						{
							// last WP not yet reached --> move on to next WP
							currentFromWaypoint = currentToWaypoint;
							currentSegment = nextSegment;
							nextSegment = nextSegment.getToWaypoint().getNextSegment(preferredSegments);
							
							// usually 0 - unless a segment is begun somewhere between FromWaypoint and 
							// ToWaypoint (e.g. changeToNeighborSegment)
							traveledDistance = currentSegment.getTraveledDistance(trafficObject);
						}
					}
				}
			}
		}
	}


	private Segment getRegularSegment(boolean rightHandTraffic)
	{
		Segment regularSegment;
		if(rightHandTraffic)
			regularSegment = currentSegment.getRightNeighbor();
		else
			regularSegment = currentSegment.getLeftNeighbor();
		return regularSegment;
	}


	private Segment getPassingSegment(boolean isRightHandTraffic)
	{
		Segment passingSegment;
		if(isRightHandTraffic)
			passingSegment = currentSegment.getLeftNeighbor();
		else
			passingSegment = currentSegment.getRightNeighbor();
		return passingSegment;
	}

	
	public void setObstacleInTheWay(boolean obstacleInTheWay)
    {
    	this.obstacleInTheWay = obstacleInTheWay;
    }
    
	
	private void changeToNeighborSegment(float progress, Segment neighborSegment)
	{
		float progressOnNeighborSegment = progress + (30/neighborSegment.getLength());
		if(neighborSegment.getLength()>30 && progressOnNeighborSegment<=1.0)
		{
			// generate names for new way points and segment
			String helperStartWPName = trafficObject.getName() + "_helperStartWP" + helperSegmentCounter;
			String helperSegmentName = trafficObject.getName() + "_helperSegment" + helperSegmentCounter;
			String helperTargetWPName = trafficObject.getName() + "_helperTargetWP" + helperSegmentCounter;
			helperSegmentCounter++;
			
			// helper target way point (on neighbor segment)
			// needs to be added before helperStartWP as it is referenced by the outgoing segment of helperStartWP
			Vector3f targetPos = neighborSegment.interpolate(progressOnNeighborSegment);
			ArrayList<Segment> segmentList2 = new ArrayList<Segment>();
			segmentList2.add(neighborSegment);
			Waypoint helperTargetWP = new Waypoint(helperTargetWPName, targetPos, null, null, null, null, null, segmentList2);
			sim.getRoadNetwork().addWaypoint(helperTargetWP);
			
			// helper segment connecting start and target segment	
			Segment helperSegment = new Segment(helperSegmentName, helperStartWPName, new ArrayList<String>(), 
					helperTargetWPName, null, null, neighborSegment.getSpeed(), false, 1, new ArrayList<String>(), 0.05f);
			
			// helper start way point (= current position)
			Vector3f curPos = followBox.getLocalTranslation();
			ArrayList<Segment> segmentList1 = new ArrayList<Segment>();
			segmentList1.add(helperSegment);
			Waypoint helperStartWP = new Waypoint(helperStartWPName, curPos, null, null, null, null, null, segmentList1);
			sim.getRoadNetwork().addWaypoint(helperStartWP);
	
			// make a note of the starting position ("traveledDistance") of the follow box when reaching the neighbor segment
			neighborSegment.setTraveledDistance(progressOnNeighborSegment*neighborSegment.getLength(), trafficObject);
			
			// update follow box values
			currentFromWaypoint = helperStartWP;				
			currentSegment = helperSegment;
			nextSegment = neighborSegment;
			traveledDistance = 0;
			
			//System.err.println(trafficObject.getName() + " changed");
		}
		//else
		//	System.err.println("Segment '" + neighborSegment.getName() + "' is too short (<30 meters)");
	}


	private boolean hasLostSteeringCar(float currentDistance) 
	{
		if(trafficObject instanceof SteeringCar)
		{
			// true if follow box is behind traffic object (angle > 270 degrees)
			Vector3f carFrontPos = ((SteeringCar)trafficObject).getFrontGeometry().getWorldTranslation();
			Vector3f carCenterPos = ((SteeringCar)trafficObject).getCenterGeometry().getWorldTranslation();
			Vector3f followBoxPos = followBox.getWorldTranslation();
			float angle = Util.getAngleBetweenPoints(carFrontPos, carCenterPos, followBoxPos, true);
			//System.err.println("carFrontPos: " + carFrontPos + ", carCenterPos: " + carCenterPos + 
			//		", followBoxPos: " + followBoxPos + ", angle: " + angle*FastMath.RAD_TO_DEG);
			boolean boxBehindCar = FastMath.abs(angle)>(0.75f*FastMath.PI);
			
			// true if follow box is more than maxDistance+1 ahead
			boolean tooFarAhead = !boxBehindCar && currentDistance > settings.getMaxDistance() + 1;
			
			return boxBehindCar || tooFarAhead;
		}
		else
			return false;
	}


	public float CRASH_THRESHOLD = 1.0f;

	/**
	 * Checks whether the traffic object gets stuck for at least 30 seconds.
	 * If movement is less than 1 meter for the last 30 seconds, true will
	 * be returned. Check not applied to traffic objects of type SteeringCar.
	 * 
	 * @return
	 * 			true, if traffic object stuck.
	 */
	private boolean hasCrashed()
	{
		// exclude steering car from this check (e.g. auto-pilot) 
		if(trafficObject instanceof SteeringCar)
			return false;
		
		// check every 3 seconds
		if(System.currentTimeMillis() - lastCrashCheck > 1000)
		{
			
			//add traveled distance on current segment to storage
			if (trafficObject instanceof Pedestrian){
				Pedestrian ped = (Pedestrian) trafficObject;
				if(ped.visible){
					distanceStorage.addLast(ped.getPosition().clone());
				}
				/*
				if(ped.getName().equals("ped2")){
					System.out.println(ped.getPosition() + ", " + ped.personNode.getWorldTranslation());
				}
				*/
			}
			
			// if maximum size (10) has been reached...
			if(distanceStorage.size() > 6)
			{
				// ...remove oldest value
	        	distanceStorage.removeFirst();
				
				// ...compute total distance the traffic object moved during the last 30 seconds
				float totalDist = 0;
		        for (int i=0; i< distanceStorage.size()-1; i++) {
					Vector3f newVal = distanceStorage.get(i + 1);
					Vector3f oldVal = distanceStorage.get(i);

					if(newVal != null && oldVal != null){
						totalDist += newVal.distance(oldVal);
					}
				}

		        float CRASH_THRESHOLD = 0.3f;

		        if (this.CRASH_THRESHOLD == 0){
		        	CRASH_THRESHOLD = 0;
				}

		        // if total distance less than one meter --> crash
				if(totalDist < CRASH_THRESHOLD)
				{
					/*
					if(trafficObject.getName().equals("ped2")){
						System.err.println("RESET: " + totalDist);
						for(Vector3f v : distanceStorage){
							System.out.println("Entry: " + v);
						}
					}*/

					distanceStorage.clear();
					traveledDistance = 0;
					return true;
				}
			}

			lastCrashCheck = System.currentTimeMillis();
		}

		return false;
	}
	private long lastCrashCheck = 0;
	public LinkedList<Vector3f> distanceStorage = new LinkedList<Vector3f>();


	boolean isSetWaitTimer = false;
	long waitTimer = 0;
	private boolean waitAtWP(Waypoint wayPoint)
	{
		// get waiting time at way point (if available)
		Integer waitingTime = wayPoint.getWaitingTime();
		
		if(waitingTime == null || waitingTime <= 0)
		{
			// no (or invalid waiting time) --> do not wait
			return false;
		}
		else
		{
			// valid waiting time available
			if(!isSetWaitTimer)
			{
				// waiting timer not yet set --> set timer to current time stamp and wait
				waitTimer = System.currentTimeMillis();
				isSetWaitTimer = true;
				return true;
			}
			else
			{
				// waiting timer already set --> check if elapsed
				if(System.currentTimeMillis()-waitTimer > waitingTime)
				{
					// waiting timer elapsed --> stop waiting and resume motion					
					isSetWaitTimer = false;
					return false;
				}
				else 
				{
					// waiting timer not elapsed --> wait
					return true;
				}
			}
		}
	}
	
	
	private void performWayPointChange(Waypoint waypoint)
	{
		// set parameters of follow box
    	boolean success = setFollowBoxToWP(waypoint);
		
		// set position and rotation to traffic object
		// do not set position and heading to SteeringCar with auto-pilot switched off
		if(success && (!(trafficObject instanceof SteeringCar) || ((SteeringCar)trafficObject).isAutoPilot()))
		{	
			// set position to traffic object
			trafficObject.setPosition(waypoint.getPosition());
			
			// set heading to traffic object
			if(currentSegment != null)
			{
				float heading = currentSegment.getHeading(traveledDistance);
				Quaternion quaternion = new Quaternion().fromAngles(0, heading, 0);
				trafficObject.setRotation(quaternion);
			}
		}
	}


	private boolean setFollowBoxToWP(Waypoint waypoint)
	{
		if(!waypoint.isEndPoint())
    	{
    		currentSegment = waypoint.getNextSegment(preferredSegments);
    		currentFromWaypoint = waypoint;
    		traveledDistance = 0;
    	}
    	else if(waypoint.isViaWP())
    	{
    		// in case of the way point has no outgoing segment, however, is a viaWayPoint, 
    		// then select outgoing segment of its parent instead
    		currentSegment = waypoint.getRandomParentSegment();
    		currentFromWaypoint = currentSegment.getFromWaypoint();
    		traveledDistance = currentSegment.getViaWPPosition(waypoint);
    	}
    	else
    	{
    		System.err.println("Could not set " + trafficObject.getName() + " to way point " + waypoint.getName() + " (end point)");
			System.err.println("[DEBUG: ]" + sim.TRAINING_SET);
    		return false;
    	}
		
		nextSegment = currentSegment.getToWaypoint().getNextSegment(preferredSegments);
		return true;
	}


	public void setToWayPoint(String name)
	{
		if(sim.getRoadNetwork().getWaypoint(name) != null)
			resetWaypoint = sim.getRoadNetwork().getWaypoint(name);
		else
			System.err.println("Way point " + name + " does not exist");
	}


	public Vector3f getPosition()
	{
		return followBox.getWorldTranslation();
	}
	
	
	private float getCurrentDistance(Vector3f trafficObjectPos) 
	{
		// get box's position on xz-plane (ignore y component)
		Vector3f followBoxPosition = getPosition().clone();
		followBoxPosition.setY(0);
		
		// get traffic object's position on xz-plane (ignore y component)
		Vector3f trafficObjectPosition = trafficObjectPos.clone();
		trafficObjectPosition.setY(0);
		
		// distance between box and trafficObject
		float currentDistance = followBoxPosition.distance(trafficObjectPosition);
		return currentDistance;
	}


	private boolean stopTrafficObject = false;
	public float getSpeed() 
	{
		if(stopTrafficObject || currentSegment==null)
			return 0;
		else
		{
			// take care of maximum speed for current segment and global maximum speed of follow box
			float regularSpeed = Math.max(Math.min(currentSegment.getSpeed(), settings.getMaxSpeed()),0);

			// reduced speed to reach next speed limit in time
			float reducedSpeed = getReducedSpeed();
			
			// return minimum of regularSpeed and reducedSpeed
			return Math.max(Math.min(regularSpeed, reducedSpeed),0);
		}
	}
 

	public float getReducedSpeed()
	{
		// return a temporarily reduced speed for the traffic car
		// in order to reach next (lower) speed limit in time
		float reducedSpeedInKmh = Float.POSITIVE_INFINITY;
		
		// if next segment with lower speed comes closer --> reduce speed
		// not for Pedestrian (can stop immediately)
		if(nextSegment != null && !(trafficObject instanceof Pedestrian))
		{
			// speed at current segment
			float currentSpeedInKmh = currentSegment.getSpeed();
			float currentSpeed = currentSpeedInKmh / 3.6f;
			
			// speed at next segment
			float targetSpeedInKmh = nextSegment.getSpeed();
			float targetSpeed = targetSpeedInKmh / 3.6f;
			
			// if speed at the next segment is lower than at the current segment --> brake traffic object
			if(targetSpeed < currentSpeed)
			{
				// speed difference in m/s between current segment's speed and next segment's speed
				float speedDifference = currentSpeed - targetSpeed;
				
				// compute the distance in front of the next WP at what the traffic object has to start 
				// braking with 50% brake force in order to reach the next WP's (lower) speed in time.
				float deceleration50Percent = 50f * trafficObject.getMaxBrakeForce()/trafficObject.getMass();
				
				// time in seconds needed for braking process
				float time = speedDifference / deceleration50Percent;
				
				// distance covered during braking process
				float coveredDistance = 0.5f * -deceleration50Percent * time * time + currentSpeed * time;

				// start braking in x meters
				float distanceToBrakingPoint = distanceToNextWP - coveredDistance;
				
				if(distanceToBrakingPoint < 0)
				{
					// reduce speed linearly beginning from braking point
					
					// % of traveled distance between braking point and next way point
					float speedPercentage = -distanceToBrakingPoint/coveredDistance;
					
					//   0% traveled: reduced speed = currentSpeed
					//  50% traveled: reduced speed = (currentSpeed+targetSpeed)/2
					// 100% traveled: reduced speed = targetSpeed
					float reducedSpeed = currentSpeed - (speedPercentage * speedDifference);
					reducedSpeedInKmh = reducedSpeed * 3.6f;
					
					/*
					if(trafficObject.getName().equals("car5"))
					{
						float trafficObjectSpeedInKmh = ((TrafficCar)trafficObject).getCurrentSpeedKmh();
						System.out.println(nextSegment.getToWaypointString() + " : " + speedPercentage + " : " + 
								reducedSpeedInKmh + " : " + trafficObjectSpeedInKmh + " : " + targetSpeedInKmh);
					}
					*/
				}
			}
		}
		
		// if intersection with higher prioritized traffic ahead --> brake and hold position	
		if(prioritizedTrafficAhead())
		{
			//System.err.println("GIVE WAY");
			reducedSpeedInKmh = 0;
		}

		return reducedSpeedInKmh;
	}

	
	private boolean prioritizedTrafficAhead()
	{
		boolean prioritizedTrafficAhead = false;
		
		// get list of segments that are higher prioritized than the current segment
		ArrayList<String> prioritizedSegments = currentSegment.getPriorityList();

		// if list not empty and end of the current segment (< 15 m to go) has been reached
		if(!prioritizedSegments.isEmpty() && distanceToNextWP < ownSafetyDistanceToIntersection)
		{
			// check if any traffic object (car + pedestrian) is using a prioritized segment (except oneself)
			for(TrafficObject otherTrafficObject : sim.getPhysicalTraffic().getTrafficObjectList())
				if(hasPriority(otherTrafficObject))
					prioritizedTrafficAhead = true;
				
			// check if human-controlled car is using a prioritized segment (except oneself)
			if(hasPriority(sim.getCar()))
				prioritizedTrafficAhead = true;
		}
		
		return prioritizedTrafficAhead;
	}


	private boolean hasPriority(TrafficObject otherTrafficObject)
	{
		// get list of segments that are higher prioritized than the current segment
		ArrayList<String> prioritizedSegments = currentSegment.getPriorityList();
		
		// current segment of the other traffic object
		Segment segment = otherTrafficObject.getCurrentSegment();
		
		// distance of the other traffic object from the next way point (meeting point)
		float dist = otherTrafficObject.getDistanceToNextWP();
		
		// Other traffic object must have a current segment, this segment must have 
		// priority to this traffic object's segment, this and other traffic object
		// must not be the same, other traffic object must be close to the meeting
		// point, and the arrival of this and the other traffic object must be at the 
		// same time (+/- 5 seconds)
		// if all true --> other traffic object has priority
		return (segment!=null && 
				prioritizedSegments.contains(segment.getName()) && 
				!trafficObject.equals(otherTrafficObject) && 
				dist<othersSafetyDistanceToIntersection && 
				hasSimultaneousArrival(otherTrafficObject));
	}
	

	private boolean hasSimultaneousArrival(TrafficObject otherTrafficObject) 
	{
		// distance of the other traffic object from the next way point (meeting point)
		float dist = otherTrafficObject.getDistanceToNextWP();
		
		// time needed by other traffic object to reach the next way point (meeting point)
		float arrivalOtherTrafficObject = 3.6f * dist/otherTrafficObject.getCurrentSpeedKmh();
		
		// time needed by this traffic object to reach the next way point (meeting point)
		float arrivalThisTrafficObject = 3.6f * distanceToNextWP/trafficObject.getCurrentSpeedKmh();
		
		// true, if difference of arrival time is less than minIntersectionClearance (5 seconds by default)
		return FastMath.abs(arrivalOtherTrafficObject-arrivalThisTrafficObject)<minIntersectionClearance;
	}


	private void computeDistanceToNextWP() 
	{
		// distance between followBox and next way point
		Vector3f followBoxPos = followBox.getWorldTranslation().clone();
		followBoxPos.setY(0);
		Vector3f nextPos = currentSegment.getToWaypoint().getPosition().clone();
		nextPos.setY(0);
		distanceToNextWP = followBoxPos.distance(nextPos);
	}
	
		
	public float getDistanceToNextWP() 
	{
		return distanceToNextWP;
	}

	
	public Waypoint getCurrentWayPoint()
	{
		if(currentSegment != null)
			return currentSegment.getFromWaypoint();
		else
			return null;
	}
	

	public Waypoint getNextWayPoint()
	{
		if(currentSegment != null)
			return currentSegment.getToWaypoint();
		else
			return null;
	}


	public Segment getCurrentSegment()
	{
		return currentSegment;
	}


	public float getTraveledDistance()
	{
		return traveledDistance;
	}
}
