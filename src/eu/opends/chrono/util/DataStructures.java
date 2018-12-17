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

package eu.opends.chrono.util;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

import com.jme3.math.FastMath;
import com.jme3.math.Matrix3f;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.sun.jna.Pointer;
import com.sun.jna.Structure;

public class DataStructures
{
	private static boolean convertLHtoRH = true;
	private static double previousTime = 0;
	private static float previousHeading = 0;
	
	
    public static class ChVector3d extends Structure
    {
    	public double x;
    	public double y;
    	public double z;
    	
    	public ChVector3d(Pointer p)
    	{
    		super(p);
    		read();
    	}
    	
    	public ChVector3d()
    	{ 
    		x = 0;
			y = 0;
			z = 0;
    	}
    	
        public ChVector3d(double _x, double _y, double _z)
        {
			x = _x;
			y = _y;
			z = _z;
		}
        
        public ChVector3d(Vector3f vector3f)
        {
        	if(convertLHtoRH)
        		x = vector3f.x;
        	else
        		x = -vector3f.x;
			y = vector3f.y;
			z = vector3f.z;
		}
        
        public void fromVector3f(float _x, float _y, float _z)
        {
        	if(convertLHtoRH)
        		x = _x;
        	else
        		x = -_x;
        	
        	y = -_z;
        	z = _y;
        }
        
        public void fromVector3f(Vector3f vector3f)
        {
        	fromVector3f(vector3f.x, vector3f.y, vector3f.z);
        }
        
        public Vector3f toVector3f()
        {
        	if(convertLHtoRH)
        		return new Vector3f((float)x , (float)z, (float)-y);
        	else
        		return new Vector3f((float)-x , (float)z, (float)-y);
        }
        
		protected List<String> getFieldOrder() 
        { 
            return Arrays.asList(new String[] {"x", "y", "z"});
        }
    }
    
    
    public static class ChQuaternion extends Structure
    {
    	public double e0;
    	public double e1;
    	public double e2;
    	public double e3;
    	
    	public ChQuaternion(Pointer p)
    	{
    		super(p);
    		read();
    	}
    	
    	public ChQuaternion()
    	{
    		e0 = 1;
    		e1 = 0;
			e2 = 0;
			e3 = 0;
    	}
    	
        public ChQuaternion(float _e0, float _e1, float _e2, float _e3)
        {
        	e0 = _e0;
			e1 = _e1;
			e2 = _e2;
			e3 = _e3;
		}
        
        public ChQuaternion(Quaternion q)
        {      
        	if(convertLHtoRH)
        	{
        		//transform quaternion
        		Matrix3f m = q.toRotationMatrix();
        		m.set(0, 2, -1*m.get(0, 2));
        		m.set(0, 1, -1*m.get(0, 1));
        		m.set(1, 0, -1*m.get(1, 0));
        		m.set(2, 0, -1*m.get(2, 0));
        		q.fromRotationMatrix(m);
        	}
        	
        	e0 = q.getW();
        	e1 = q.getX();
        	e2 = q.getZ();
        	e3 = -q.getY();
        }
        
        public Quaternion toQuaternion()
        {
        	float w = (float) e0;
        	float x = (float) e1;
        	float y = (float) -e3;
        	float z = (float) e2;
        	
        	Quaternion q = new Quaternion();
        	q.set(x, y, z, w);
        	
        	if(convertLHtoRH)
        	{
        		//transform quaternion
        		Matrix3f m = q.toRotationMatrix();
        		m.set(0, 2, -1*m.get(0, 2));
        		m.set(0, 1, -1*m.get(0, 1));
        		m.set(1, 0, -1*m.get(1, 0));
        		m.set(2, 0, -1*m.get(2, 0));
        		q.fromRotationMatrix(m);
        	}
        	
        	return q;
        }
        
        
        public void fromQuaternion(Quaternion q)
        {      
        	if(convertLHtoRH)
        	{
        		//transform quaternion
        		Matrix3f m = q.toRotationMatrix();
        		m.set(0, 2, -1*m.get(0, 2));
        		m.set(0, 1, -1*m.get(0, 1));
        		m.set(1, 0, -1*m.get(1, 0));
        		m.set(2, 0, -1*m.get(2, 0));
        		q.fromRotationMatrix(m);
        	}
        	
        	e0 = q.getW();
        	e1 = q.getX();
        	e2 = q.getZ();
        	e3 = -q.getY();
        }
        
        
		protected List<String> getFieldOrder() 
        { 
            return Arrays.asList(new String[] {"e0", "e1", "e2", "e3"});
        }
    }
    
    public static class Wheels extends Structure
    {
    	public ChVector3d position;
    	public ChQuaternion rotation;
    	
    	public Wheels(Pointer p)
    	{
    		super(p);
    		read();
    	}
    	
    	public Wheels()
    	{
    	}
    	
        protected List<String> getFieldOrder() 
        { 
            return Arrays.asList(new String[] {"position", "rotation"});
        }
    }
    
    
    public static class UpdateResult extends Structure
    {
    	public double time;
    	public int step_number;
    	public double throttle;
    	public double braking;
    	public double steering;
    	public double powertrain_torque;
    	public double powertrain_engineSpeed;
    	public int powertrain_currentGear;
    	public double driveshaft_speed;
    	public double vehicle_speed;
    	public ChVector3d vehicle_acceleration;
    	public ChVector3d chassisPosition;
    	public ChQuaternion chassisRotation;
    	public int num_wheels;
    	public Wheels[] wheels = new Wheels[4];
    	public float mass;
    	
    	public UpdateResult(Pointer p)
    	{
    		super(p);
    		read();
    	}
    	
    	public UpdateResult()
    	{
    	}
    	
        protected List<String> getFieldOrder() 
        { 
            return Arrays.asList(new String[] {"time", "step_number", "throttle", "braking", "steering",
            		"powertrain_torque", "powertrain_engineSpeed", "powertrain_currentGear", "driveshaft_speed", 
            		"vehicle_speed", "vehicle_acceleration", "chassisPosition", "chassisRotation", "num_wheels", "wheels",
            		"mass"});
        }
        
        
        
        // ---------------------------------
        // PARAMETERS FOR ADAPTIVE INTERFACE
        // ---------------------------------
        
        // Filtered longitudinal velocity from odometer (m/s)
        public double getVLgtFild()
        {
			return vehicle_speed/3.6; // check unit
        }
        
        // Filtered longitudinal acceleration (m/s^2)
        public double getALgtFild()
        {
			return vehicle_acceleration.x; //x?
        }
        
        // Filtered lateral acceleration (m/s^2)
        public double getALatFild()
        {
			return vehicle_acceleration.y; //y?
        }
        
        public float getHeading()
        {
        	float angles[] = new float[3];
			chassisRotation.toQuaternion().toAngles(angles);
			
			// heading in radians
			float heading = 270*FastMath.DEG_TO_RAD - angles[1];
			
			// normalize radian angle
			float angle_rad = (heading + FastMath.TWO_PI) % FastMath.TWO_PI;
			
			return angle_rad;
        }
        
        public float getHeadingDegrees()
        {
        	return getHeading()*FastMath.RAD_TO_DEG;
        }
    	
    	private double doSmoothing(LinkedList<Double> storage, int maxSize, double addValue) 
    	{		
    		double sum = 0;
        	
        	storage.addLast(addValue);

            for (double value : storage)
            	sum += value;
            
            double result = sum / storage.size();
            
            if(storage.size() >= maxSize)
            	storage.removeFirst();

            return result;
    	}

        // Filtered yaw-rate (rad/s)
        public double getYawRateFild()
        {
        	double currentTime = time;
        	float currentHeading = getHeading();
        	
        	double diffTime = currentTime-previousTime;
        	float diffHeading = currentHeading-previousHeading;
        	
        	
        	
        	if(diffHeading > FastMath.PI)  // 180
        		diffHeading -= FastMath.TWO_PI;  // 360
        	
        	if(diffHeading < -FastMath.PI)  // 180
        		diffHeading += FastMath.TWO_PI;  // 360
        	
        	//doSmoothing(headingDiffStorage, 10, diffHeading);
        	
        	previousTime = currentTime;
        	previousHeading = currentHeading;
        	
        	return diffHeading/diffTime;
        }
        
        // Filtered yaw-rate (deg/s)
        public double getYawRateFildDegrees()
        {
        	return getYawRateFild()*FastMath.RAD_TO_DEG;
        }
        
        // Engine Torque (%)
        public double getEngineTorque()
        {
        	return powertrain_torque; // check %
        }
        
        // Engine Speed (1/min)
        public double getEngineSpeed()
        {
        	return powertrain_engineSpeed;
        }
        
        // Actual Gear
        public int getActGear()
        {
        	return powertrain_currentGear;
        }
        
        
    }

}
