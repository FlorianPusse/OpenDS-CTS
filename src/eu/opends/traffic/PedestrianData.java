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

import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;

/**
 * 
 * @author Rafael Math
 */
public class PedestrianData 
{
	private String name;
	private boolean enabled;
	private float mass;
	private String animationStand;
	private String animationWalk;
	private float localScale;
	private Vector3f localTranslation;
	private Quaternion localRotation;
	private String modelPath;
	private FollowBoxSettings followBoxSettings;
	
	
	public PedestrianData(String name, boolean enabled, float mass, String animationStand, String animationWalk, 
			float localScale, Vector3f localTranslation, Quaternion localRotation, String modelPath, 
			FollowBoxSettings followBoxSettings) 
	{
		this.name = name;
		this.enabled = enabled;
		this.mass = mass;
		this.animationStand = animationStand;
		this.animationWalk = animationWalk;
		this.localScale = localScale;
		this.localTranslation = localTranslation;
		this.localRotation = localRotation;
		this.modelPath = modelPath;
		this.followBoxSettings = followBoxSettings;
	}


	public String getName() {
		return name;
	}
	

	public boolean getEnabled() {
		return enabled;
	}
	
	
	public void setEnabled(boolean enabled) {
		this.enabled = enabled;
	}
	

	public float getMass() {
		return mass;
	}

	
	public void setMass(float mass) {
		this.mass = mass;
	}
	

	public String getAnimationStand() {
		return animationStand;
	}

	
	public void setAnimationStand(String animationStand) {
		this.animationStand = animationStand;
	}
	

	public String getAnimationWalk() {
		return animationWalk;
	}
	
	
	public void setAnimationWalk(String animationWalk) {
		this.animationWalk = animationWalk;
	}
	

	public float getLocalScale() {
		return localScale;
	}
	
	
	public void setLocalScale(float localScale) {
		this.localScale = localScale;
	}
	
	
	public Vector3f getLocalTranslation() {
		return localTranslation;
	}

	
	public void setLocalTranslation(Vector3f localTranslation) {
		this.localTranslation = localTranslation;
	}
	
	
	public Quaternion getLocalRotation() {
		return localRotation;
	}

	
	public void setLocalRotation(Quaternion localRotation) {
		this.localRotation = localRotation;
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
		
		return "\t\t<pedestrian id=\"" + name + "\">\n" +
			   "\t\t\t<modelPath>" + modelPath + "</modelPath>\n" + 
			   "\t\t\t<mass>"+ mass + "</mass>\n" + 
			   "\t\t\t<animationWalk>"+ animationWalk + "</animationWalk>\n" + 
			   "\t\t\t<animationStand>"+ animationStand + "</animationStand>\n" + 
			   "\t\t\t<localTranslation><vector><entry>" + localTranslation.getX() + "</entry><entry>" + localTranslation.getY() +
			   		"</entry><entry>" + localTranslation.getZ() + "</entry></vector></localTranslation>\n" + 
			   	"\t\t\t<localRotation quaternion=\"true\"><vector><entry>" + localRotation.getX() + "</entry><entry>" + 
			   		localRotation.getY() + "</entry><entry>" + localRotation.getZ() + "</entry><entry>" + localRotation.getW() + 
			   		"</entry></vector></localRotation>\n" + 
			   "\t\t\t<scale>"+ localScale + "</scale>\n" + 
			   "\t\t\t<maxSpeed>"+ followBoxSettings.getMaxSpeed() + "</maxSpeed>\n" + 							  
			   "\t\t\t<giveWayDistance>"+ followBoxSettings.getGiveWayDistance() + "</giveWayDistance>\n" + 
			   "\t\t\t<intersectionObservationDistance>"+ followBoxSettings.getIntersectionObservationDistance() + "</intersectionObservationDistance>\n" + 
			   "\t\t\t<minIntersectionClearance>"+ followBoxSettings.getMinIntersectionClearance() + "</minIntersectionClearance>\n" + 
			   "\t\t\t<enabled>"+ enabled + "</enabled>\n" + 
			   "\t\t\t<minDistanceFromPath>"+ followBoxSettings.getMinDistance() + "</minDistanceFromPath>\n" + 
			   "\t\t\t<maxDistanceFromPath>"+ followBoxSettings.getMaxDistance() + "</maxDistanceFromPath>\n" + 
			   "\t\t\t<startWayPoint>"+ followBoxSettings.getStartWayPointID() + "</startWayPoint>\n" + 
			   preferredSegments + 
			   "\t\t</pedestrian>";	
	}

}
