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

import java.util.HashMap;
import java.util.Iterator;
import java.util.TreeMap;

import com.jme3.material.Material;
import com.jme3.math.ColorRGBA;
import com.jme3.math.FastMath;
import com.jme3.math.Vector3f;
import com.jme3.scene.Geometry;
import com.jme3.scene.Node;
import com.jme3.scene.shape.Box;
import com.jme3.scene.shape.Curve;

import eu.opends.drivingTask.scenario.ScenarioLoader;
import eu.opends.main.Simulator;

/**
 * This class represents the road network of a scene consisting of way points and 
 * segments. A segment connects two way points allowing to build a graph of possible
 * path ways for traffic, pedestrians, and auto-pilot.
 * 
 * @author Rafael Math
 */
public class RoadNetwork 
{
	private Simulator sim;
	private Material blueMaterial;
	private Material redMaterial;
	private HashMap<String, Waypoint> waypointMap = new HashMap<String, Waypoint>();
	private HashMap<String, Segment> segmentMap = new HashMap<String, Segment>();
	private boolean isRightHandTraffic = true;
    private Node debugNode = new Node();
    private boolean drawWaypoints;
    private boolean drawSegments;
    
    
    /**
     * Creates a new instance of the road network by looking up all available way
     * points and their referenced segments. Furthermore, segments will be referenced 
     * to way points again.
     * 
     * @param sim
     * 			The simulator
     */
	public RoadNetwork(Simulator sim)
	{
		this.sim = sim;
		ScenarioLoader scenarioLoader = sim.getDrivingTask().getScenarioLoader();
		
		// set up whether right hand traffic is applicable
		isRightHandTraffic = scenarioLoader.isRightHandTraffic();
		
		// initialize blue and red material for debug visualization of the way points and segments 
        blueMaterial = new Material(sim.getAssetManager(), "Common/MatDefs/Misc/Unshaded.j3md");
        blueMaterial.getAdditionalRenderState().setWireframe(true);
        blueMaterial.setColor("Color", ColorRGBA.Blue);
        redMaterial = sim.getAssetManager().loadMaterial("Common/Materials/RedColor.j3m");

		// fill waypointMap and segmentMap from scenario.xml
		waypointMap = scenarioLoader.getWaypointMap();
		segmentMap = scenarioLoader.getSegmentMap();
		
		// set up visualization of way points and segments
	    //drawWaypoints = scenarioLoader.isVisualizeRoadWaypoints();
	    //drawSegments = scenarioLoader.isVisualizeRoadSegments();

		drawWaypoints = false;
		drawSegments = false;

		// initialize all way points and their outgoing segments
		for(Waypoint waypoint : waypointMap.values())
			initWaypoint(waypoint);
	    
		if(drawWaypoints || drawSegments)
			sim.getSceneNode().attachChild(debugNode);
	}

	
	/**
	 * Adds a way point to the way point map and visual debug node after 
	 * the simulation has already been started. Furthermore, all outgoing 
	 * segments will be initialized and added to visual debug node (if not 
	 * done yet). 
	 * 
	 * @param waypoint
	 * 			Way point to be added.
	 */
	public void addWaypoint(Waypoint waypoint)
	{
		// add way point to way point map
		waypointMap.put(waypoint.getName(), waypoint);
		
		// init way point (init all outgoing segments and add to debug mode)
		initWaypoint(waypoint);
	}


	/**
     * Getter method for the debug node containing way points and segments.
     * 
     * @return
     * 			Debug node
     */
	public Node getDebugNode()
	{
		return debugNode;
	}
	

	/**
	 * Select randomly one of the five closest way points from the given position.
	 * Returns null if no way point available.
	 * 
	 * @param position
	 * 			Position to find five closest way points.
	 * 
	 * @return
	 * 			Random way point close to given position.
	 */
	public Waypoint getRandomNearbyWaypoint(Vector3f position)
	{		
		TreeMap<Float, Waypoint> wpByDistanceMap = new TreeMap<Float, Waypoint>();
		
        for(Waypoint waypoint : waypointMap.values()) 
        {
        	// Preselect all way points which have an outgoing segment
        	if(!waypoint.isEndPoint())
        	{
        		float distance = waypoint.getPosition().distance(position);
        		wpByDistanceMap.put(distance, waypoint);
        	}
        	// ... or are part of a segment (= viaWaypoint)
        	else if(waypoint.isViaWP())
        	{
        		// in case of the way point is a viaWayPoint, select its parent instead
        		Waypoint parentWP = waypoint.getRandomParentSegment().getFromWaypoint(); //FIXME
        		if(!parentWP.isEndPoint())
        		{
        			float distance = waypoint.getPosition().distance(position);
            		wpByDistanceMap.put(distance, parentWP);
        		}
        	}
        }
		
        // return one of the nearest 5 way points randomly
        int random = FastMath.nextRandomInt(0, 5);
        Waypoint returnWaypoint = null;
        Iterator<Waypoint> it = wpByDistanceMap.values().iterator();
        for(int i=0; i<=random && it.hasNext(); i++)
        	 returnWaypoint = it.next();

	    return returnWaypoint;
	}
	
	
	/**
	 * Getter method for the segments map.
	 * 
	 * @return
	 * 			Segments map
	 */
	public HashMap<String, Segment> getSegmentsMap()
	{
		return segmentMap;
	}

	
	/**
	 * Lookup way point from the way point map by name.
	 * 
	 * @param name
	 * 			Name of the way point to look up.
	 * 			
	 * @return
	 * 			Way point
	 */
	public Waypoint getWaypoint(String name)
	{
		return waypointMap.get(name);
	}
    
	
	/**
	 * Getter method for the way point map.
	 * 
	 * @return
	 * 			Way point map
	 */
	public HashMap<String, Waypoint> getWaypointMap()
	{
		return waypointMap;
	}
	

    /**
	 * Getter method for right hand traffic. This can be used to determine 
	 * which segments to be used for regular drive and which for passing.
	 * 
	 * @return
	 * 			true, if right hand traffic.
	 */
	public boolean isRightHandTraffic()
	{
		return isRightHandTraffic;
	}


	/**
	 * Creates a new box.
	 * 
	 * @param waypoint
	 * 				Way point represented by this box.
	 * 
	 * @param material
	 * 				Material to be used to color box.
	 */
	private void createBox(Waypoint waypoint, Material material)
	{
		Geometry wpBox = new Geometry("box_" + waypoint.getName(), new Box(0.3f, 0.3f, 0.3f));
		wpBox.setLocalTranslation(waypoint.getPosition());
		wpBox.setMaterial(material);
		debugNode.attachChild(wpBox);
	}


	/**
	 * Creates a new curve.
	 * 
	 * @param segment
	 * 				Segment represented by this curve.
	 * 
	 * @param material
	 * 				Material to be used to color curve.
	 */
	private void createCurve(Segment segment, Material material)
	{
		Curve curve = new Curve(segment.getSpline(), segment.getNoOfCurveSegments());
		Geometry lineGeometry = new Geometry("segment_" + segment.getName(), curve);
		lineGeometry.setMaterial(material);
		debugNode.attachChild(lineGeometry);
	}
	
	
	/**
	 * Adds way point to visual debug node. Furthermore, all outgoing 
	 * segments will be initialized and added to visual debug node.
	 * 
     * @param waypoint
     * 			The way point to initialize.
	 */
	private void initWaypoint(Waypoint waypoint)
	{	
		if(drawWaypoints)
		{
			// create a red box for each way point
	        createBox(waypoint, redMaterial);
		}
		
		// look up all outgoing segments
		for(Segment segment : waypoint.getOutgoingSegmentList())
		{
			// if segment has not been initialized yet...
			if(true || !segment.isInitialized())
			{
				// ... initialize segment (add start way point, end way point, and spline)
				segment.init(waypointMap, segmentMap, sim);
				
				if(drawSegments)
				{
					// ... create a blue line for each (non-jump) segment
					if(!segment.isJump())
						createCurve(segment, blueMaterial);
				}
			}
		}
	}

}
