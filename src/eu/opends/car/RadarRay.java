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

package eu.opends.car;

import com.jme3.collision.CollisionResults;
import com.jme3.material.Material;
import com.jme3.math.ColorRGBA;
import com.jme3.math.Quaternion;
import com.jme3.math.Ray;
import com.jme3.math.Vector3f;
import com.jme3.scene.Geometry;
import com.jme3.scene.Node;
import com.jme3.scene.shape.Cylinder;
import com.jme3.scene.shape.Sphere;

import eu.opends.main.Simulator;


public class RadarRay 
{
	private Simulator sim;
	private Node raySourceNode;
	private Vector3f direction;
	private float maxRange;
	private boolean debug;
	private Node rayTargetNode = new Node();
	private Node rayDistanceNode = new Node();
	
	
	public RadarRay(Simulator sim, Node raySourceNode, String rayID, Vector3f direction, float maxRange, boolean debug)
	{
		this.sim = sim;
		this.raySourceNode = raySourceNode;
		direction.normalizeLocal();
		this.direction = direction;
		this.maxRange = maxRange;
		this.debug = debug;
		
		if(debug)
		{
			// add visual ray source
			Geometry raySource = createSphere("RaySource_" + rayID, ColorRGBA.Green);
			raySourceNode.attachChild(raySource);
			
			// add visual ray cylinder
			Node rayCylinder = createCylinder("RayCylinder_" + rayID, ColorRGBA.Red, maxRange, direction);
			raySourceNode.attachChild(rayCylinder);
			
			// add visual ray target
			Geometry rayTarget = createSphere("RayTarget_" + rayID, ColorRGBA.Blue);
			rayDistanceNode.attachChild(rayTarget);
			raySourceNode.attachChild(rayDistanceNode);
		}
		
		rayTargetNode.setLocalTranslation(direction);
		raySourceNode.attachChild(rayTargetNode);
	}


	public float getDistanceToObstacle() 
	{
		Vector3f targetPos = rayTargetNode.getWorldTranslation();
		Vector3f sourcePos = raySourceNode.getWorldTranslation();
				
		Vector3f worldDirection = targetPos.subtract(sourcePos);
		worldDirection.normalizeLocal();
		
		//System.err.println(dir);
		
		Ray ray = new Ray(sourcePos, worldDirection);
		CollisionResults results = new CollisionResults();
		sim.getSceneNode().collideWith(ray, results);

		
		float distance = maxRange;
		if (results.size() > 0) 
		{
			for(int k=0; k< results.size(); k++)
			{
				String name = results.getCollision(k).getGeometry().getName();
				if(!name.startsWith("RaySource_") && 
				   !name.startsWith("RayTarget_") && 
				   !name.startsWith("RayCylinder_"))
				{
					float d = results.getCollision(k).getDistance();
					if(d < distance)
						distance = d;
					
					//System.err.println(name + " in " + d);
				}
			}
		}
		
		if(debug)
			rayDistanceNode.setLocalTranslation(direction.mult(distance));
		
		return distance;
	}
	

	private Geometry createSphere(String name, ColorRGBA color) 
	{
		// create a colored sphere to mark the ray source or target (debug only)
		Sphere sphere = new Sphere(3, 3, 0.08f);
		Geometry rayEndPoint = new Geometry(name, sphere);
		Material sphere_mat = new Material(sim.getAssetManager(), "Common/MatDefs/Misc/Unshaded.j3md");
		sphere_mat.setColor("Color", color);
		rayEndPoint.setMaterial(sphere_mat);
		return rayEndPoint;
	}
	

	private Node createCylinder(String name, ColorRGBA color, float height, Vector3f direction)
	{
		int axisSamples = 5;		
		int radialSamples = 5;		
		float radius = 0.01f;
		Boolean closed = true;
			
		Node node = new Node(name);
		
		// create a colored cylinder to mark the ray direction (debug only)
		Cylinder cylinder = new Cylinder(axisSamples, radialSamples, radius, height, closed);
		Geometry geometry = new Geometry(name, cylinder);
		
		String matDefinition = "Common/MatDefs/Misc/Unshaded.j3md";
		Material material = new Material(sim.getAssetManager(), matDefinition);
		material.setColor("Color", color);
		geometry.setMaterial(material);
		geometry.setLocalTranslation(new Vector3f(0,0,-height/2f));
				
		node.attachChild(geometry);
				
		Quaternion q = new Quaternion();
		q.lookAt(direction.negate(), Vector3f.UNIT_Y);
		node.setLocalRotation(q);
		
		return node;
	}
}
