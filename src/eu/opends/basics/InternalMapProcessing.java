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

package eu.opends.basics;

import java.nio.FloatBuffer;
import java.util.ArrayList;
import java.util.List;

import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.control.RigidBodyControl;
import com.jme3.bullet.util.CollisionShapeFactory;
import com.jme3.collision.CollisionResult;
import com.jme3.collision.CollisionResults;
import com.jme3.math.Ray;
import com.jme3.math.Vector3f;
import com.jme3.scene.Geometry;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.scene.Spatial.CullHint;
import com.jme3.scene.VertexBuffer;
import com.jme3.scene.VertexBuffer.Format;
import com.jme3.scene.VertexBuffer.Usage;

import eu.opends.main.Simulator;
import eu.opends.tools.Util;

/**
 * This class is used to further process the elements on the map.
 * 
 * @author Rafael Math
 */
public class InternalMapProcessing
{
	private SimulationBasics sim;
	private Node sceneNode;
	private Node mapNode;
	private PhysicsSpace physicsSpace;
	private List<Spatial> triggerList = new ArrayList<Spatial>();
	
	
	public InternalMapProcessing(SimulationBasics sim)
	{
		this.sim = sim;
		this.sceneNode = sim.getSceneNode();
		this.mapNode = sim.getMapNode();
		this.physicsSpace = sim.getBulletPhysicsSpace();
		
		// get list of additional objects (generated from XML file)
		addMapObjectsToScene(sim.getDrivingTask().getSceneLoader().getMapObjects());

		System.out.println("MapModelList:  [" + listToString(sceneNode) + "]");

		// apply triggers to certain visible objects
		if (sim instanceof Simulator) 
		{		
			//generateTrafficLightTriggers();
			generateDrivingTaskTriggers();
			addTriggersToTriggerNode();
		}
	}


	private String listToString(Node sceneNode) 
	{
		String output = "";
        boolean isFirstChild = true;
        for(Spatial child : sceneNode.getChildren())
        {
        	if(isFirstChild)
        	{
        		output += child.getName();
        		isFirstChild = false;
        	}
        	else
        		output += ", " + child.getName();
        }
		return output;
	}
	
	
	/**
	 * Converts a list of map objects into a list of spatial objects which 
	 * can be added to the simulators scene graph.
	 * 
	 * @param mapObjects
	 * 			List of map objects to convert
	 * 
	 * @return
	 * 			List of spatial objects
	 */
	private void addMapObjectsToScene(List<MapObject> mapObjects)
	{			
		for(MapObject mapObject : mapObjects)
		{	
			boolean skipPhysicModel = false;
			
			Node node = new Node(mapObject.getName());
			
			Spatial spatial = mapObject.getSpatial();
			
			/*
			Geometry geo0 = ((Geometry) Util.findGeom(spatial,"test-geom-0"));
			if(geo0!=null)
			{
				FloatBuffer posData  =  (FloatBuffer) geo0.getMesh().getBuffer(VertexBuffer.Type.Position).getData();
				for(int i=0;i<posData.limit();i++)
				{
					if(i%3==1)
						posData.put(i, -10);
				}
				geo0.getMesh().getBuffer(VertexBuffer.Type.Position).setupData(Usage.Static, 3, Format.Float, posData);
			}
			*/
			
			/*
			System.err.println(Util.printTree(sceneNode));
			
			List<Geometry> geometryList = Util.getAllGeometries(sceneNode.getChild("terrain"));
			int level=0;
			for(Geometry geo0 : geometryList)
			{
				level -= 10;
				if(geo0!=null)
				{
					FloatBuffer posData  =  (FloatBuffer) geo0.getMesh().getBuffer(VertexBuffer.Type.Position).getData();
					for(int i=0;i<posData.limit();i++)
					{
						if(i%3==1)
							posData.put(i, level);
					}
					geo0.getMesh().getBuffer(VertexBuffer.Type.Position).setupData(Usage.Static, 3, Format.Float, posData);
				}
			}
			*/
			
			
			/**/
			Geometry geo1 = ((Geometry) Util.findGeom(spatial,"test-geom-0"));
			if(geo1!=null)
			{
				FloatBuffer posData  =  (FloatBuffer) geo1.getMesh().getBuffer(VertexBuffer.Type.Position).getData();
				for(int i=0;i<posData.limit()/3;i++)
				{
					float x = posData.get(3*i);
					float z = posData.get(3*i+2);
					
					
					// reset collision results list
					CollisionResults results = new CollisionResults();

					// ray origin
					Vector3f origin = new Vector3f(x, 1000, z);
						
					// downward direction
					Vector3f direction = new Vector3f(0,-1,0);

					// aim a ray from the camera towards the target
					Ray ray = new Ray(origin, direction);

					// collect intersections between ray and scene elements in results list.
					sceneNode.collideWith(ray, results);
					
						
					// use the results (we mark the hit object)
					if (results.size() > 0) 
					{
						// the closest collision point is what was truly hit
						CollisionResult closest = results.getClosestCollision();
						
						posData.put(3*i+1, closest.getContactPoint().y + 0.1f);
							
					}
					
					
					
					// overwrite y
					//posData.put(3*i+1, 150);
				}
				geo1.getMesh().getBuffer(VertexBuffer.Type.Position).setupData(Usage.Static, 3, Format.Float, posData);
				
				skipPhysicModel = true;
			}
			
			
			Geometry geo2 = ((Geometry) Util.findGeom(spatial,"test-geom-1"));
			if(geo2!=null)
			{
				FloatBuffer posData  =  (FloatBuffer) geo2.getMesh().getBuffer(VertexBuffer.Type.Position).getData();
				for(int i=0;i<posData.limit();i++)
				{
					if(i%3==1)
						posData.put(i, 150);
				}
				geo2.getMesh().getBuffer(VertexBuffer.Type.Position).setupData(Usage.Static, 3, Format.Float, posData);
			}
			/**/
			
			
			
        	// set FaceCullMode of spatial's geometries to off
			// no longer needed, as FaceCullMode.Off is default setting
			//Util.setFaceCullMode(spatial, FaceCullMode.Off);
			
	    	node.attachChild(spatial);

	    	node.setLocalScale(mapObject.getScale());

	        node.updateModelBound();
	        
			// if marked as invisible then cull always else cull dynamic
			if(!mapObject.isVisible())
				node.setCullHint(CullHint.Always);
			
			String collisionShapeString = mapObject.getCollisionShape();
			if(collisionShapeString == null)
				collisionShapeString = "meshShape";
			
			node.setLocalTranslation(mapObject.getLocation());
	        node.setLocalRotation(mapObject.getRotation());
	        
			if(!skipPhysicModel && (collisionShapeString.equalsIgnoreCase("boxShape") || collisionShapeString.equalsIgnoreCase("meshShape")))
			{
		        //node.setLocalTranslation(mapObject.getLocation());
		        //node.setLocalRotation(mapObject.getRotation());
				
		        CollisionShape collisionShape;
		        float mass = mapObject.getMass();

		        if(mass == 0)
		        {
		        	// mesh shape for static objects
			        if(collisionShapeString.equalsIgnoreCase("meshShape"))
			        	collisionShape = CollisionShapeFactory.createMeshShape(node);
			        else
			        	collisionShape = CollisionShapeFactory.createBoxShape(node);
		        }
		        else
		        {
			        // set whether triangle accuracy should be applied
			        if(collisionShapeString.equalsIgnoreCase("meshShape"))
			        	collisionShape = CollisionShapeFactory.createDynamicMeshShape(node);
			        else
			        	collisionShape = CollisionShapeFactory.createBoxShape(node);
		        }		        
		        
		        RigidBodyControl physicsControl = new RigidBodyControl(collisionShape, mass);
		        node.addControl(physicsControl);

		        physicsControl.setPhysicsLocation(mapObject.getLocation());
		        physicsControl.setPhysicsRotation(mapObject.getRotation());
		        
		        //physicsControl.setFriction(100);
		        
		        // add additional map object to physics space
		        physicsSpace.add(physicsControl);
			}

			
	        // attach additional map object to scene node
			if(mapObject.isAddToMapNode())
				mapNode.attachChild(node);
			else
				sceneNode.attachChild(node);
		}
	}
	
	
	/**
	 * Generates blind triggers which replace the original boxes.
	 */
	private void generateDrivingTaskTriggers()
	{		
		for (Spatial object : sceneNode.getChildren()) 
		{
			if (sim.getTriggerActionListMap().containsKey(object.getName()))
			{
				// add trigger to trigger list
				triggerList.add(object);
			}
		}
	}
	
	
	private void addTriggersToTriggerNode()
	{
		for(Spatial object : triggerList)
		{
			// add trigger to trigger node
			sim.getTriggerNode().attachChild(object);
		}
	}	
}
