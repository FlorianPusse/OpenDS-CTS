package eu.opends.car;

import java.util.ArrayList;

import com.jme3.collision.CollisionResults;
import com.jme3.math.Vector3f;
import com.jme3.scene.Node;

import com.jme3.scene.Spatial;
import eu.opends.main.Simulator;

public class RadarSensor
{
	private static final boolean enabled = false;
	private static final boolean debug = true;
	private static final float maxRange = 10.0f;
	private Node raySourceNode = new Node();
	private ArrayList<RadarRay> radarRayList = new ArrayList<>();

	Node tmpCityNode;
	Simulator sim;


	public RadarSensor(Simulator sim, Node carNode)
	{
		if(enabled)
		{
			this.sim = sim;
			Node cityNode = (Node) sim.getSceneNode().getChild(4);
			tmpCityNode = (Node) cityNode.getChild(0);


			// place ray source node in front of car
			raySourceNode.setLocalTranslation(new Vector3f(0, 0.5f, -1.6f));
			carNode.attachChild(raySourceNode);
		}
	}


	public void update()
	{
		if(enabled)
		{
			int x = 0;
			for (Spatial tmpSpatial : tmpCityNode.getChildren()) {
				Node object = (Node) tmpSpatial;
				Vector3f direction = object.getWorldTranslation().subtract(raySourceNode.getWorldTranslation());
				RadarRay ray = new RadarRay(sim, raySourceNode, "ray_"+x, direction, maxRange, debug);
				ray.getDistanceToObstacle();
				++x;
			}
		}
	}
}


/*
package eu.opends.car;

import java.util.ArrayList;

import com.jme3.math.Vector3f;
import com.jme3.scene.Node;

import eu.opends.main.Simulator;

public class RadarSensor 
{
	private static final boolean enabled = false;
	private static final boolean debug = true;
	private static final float maxRange = 10.0f;
	private Node raySourceNode = new Node();
	private ArrayList<RadarRay> radarRayList = new ArrayList<RadarRay>();
	
	
	public RadarSensor(Simulator sim, Node carNode)
	{		
		if(enabled)
		{
			// place ray source node in front of car
			raySourceNode.setLocalTranslation(new Vector3f(0, 0.25f, -1.6f));
			carNode.attachChild(raySourceNode);
			
			// create rays
			for(float x=-0.8f; x<=0.8f; x+=0.1f)
			{
				for(float y=-0.12f; y<=0.12f; y+=0.04f)
				{
					//Vector3f direction = new Vector3f(-0.3f,0.2f,-0.8f);
					Vector3f direction = new Vector3f(x,y,-0.8f);
					RadarRay ray = new RadarRay(sim, raySourceNode, "ray_"+x+"_"+y, direction, maxRange, debug);
					radarRayList.add(ray);
				}
			}
		}
	}


	public void update()
	{
		if(enabled)
		{
			for(RadarRay rr : radarRayList)
				System.err.println("Obstacle in: " + rr.getDistanceToObstacle() + " m");
		}
	}
}
*/