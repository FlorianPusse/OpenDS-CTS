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

package eu.opends.chrono;


import com.jme3.math.FastMath;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.scene.Spatial;
import com.sun.jna.Native;

import eu.opends.chrono.util.DataStructures.ChQuaternion;
import eu.opends.chrono.util.DataStructures.UpdateResult;
import eu.opends.main.Simulator;
import eu.opends.chrono.util.DataStructures.ChVector3d;


public class ChronoVehicleControl
{
	private boolean runChronoInThread = false;
	private ChronoLibrary chrono;
	private Simulator sim = null;
	
	private long initialTime = System.currentTimeMillis();
	private boolean runThread = true;
	private UpdateResult result = new UpdateResult();
	
	private float steeringValue = 0;
	private float accelerationValue = 0;
	private float brakeValue = 0;

	private enum WheelPos
	{
		leftFrontWheel, rightFrontWheel, leftBackWheel, rightBackWheel;
	}
	
	public static void main(String[] args)
	{
		Vector3f initialPos = new Vector3f(9.5f,122,-78);
		Quaternion initialRot = new Quaternion().fromAngles(0, 180*FastMath.DEG_TO_RAD, 0);
		String vehicleFile = "generic/vehicle/Vehicle_Rafael.json";
		String tireFile = "generic/tire/RigidTire_Rafael.json";
		String powertrainFile = "generic/powertrain/SimplePowertrain.json";
		String terrainFile = "terrain/RigidMesh2.json";
		
		ChronoVehicleControl vehicle = new ChronoVehicleControl(null, initialPos, initialRot, vehicleFile, 
				tireFile, powertrainFile, terrainFile);
		
		long millis = System.currentTimeMillis();
	    for(int i=0; i<10000; i++)
	    	vehicle.updateChrono();

	    System.err.println("time passed: " + (System.currentTimeMillis() - millis));
	    
	    vehicle.close();
	}
	
	
	public ChronoVehicleControl(Simulator sim, Vector3f initialPos, Quaternion initialRot, String vehicleFile,
			String tireFile, String powertrainFile, String terrainFile)
	{	 
		this.sim = sim;
		
		System.setProperty("jna.library.path", "C:\\ProjectChrono\\chrono_build\\bin\\Release"); //TODO
		chrono = (ChronoLibrary)Native.loadLibrary(("ChronoEngine_interface"), ChronoLibrary.class);
		
		chrono.initSimulation();
		
		ChVector3d position = new ChVector3d();
		position.fromVector3f(initialPos);
		
		ChQuaternion rotation = new ChQuaternion();
		rotation.fromQuaternion(initialRot);
		
		// use external visualization (Irrlicht) if not run in OpenDS
		boolean externalViz = (sim == null);
		chrono.initCar(vehicleFile, tireFile, position, rotation, externalViz);
		chrono.initPowertrain(powertrainFile);
		chrono.initDriver();
		
		Vector3f terrainPos = new Vector3f();
		Quaternion terrainRot = new Quaternion();
		//terrainRot.fromAngles(0, 180*FastMath.DEG_TO_RAD, 0);
		chrono.addTerrain(terrainFile, new ChVector3d(terrainPos), new ChQuaternion(terrainRot), externalViz);

		if(externalViz)	
			chrono.enableGUI();
			
		
		if(sim != null && runChronoInThread)
		{
			new Thread(){
				public void run(){
					while(runThread)	    	
						updateChrono();
				}
			}.start();
		}
	}

	private void setupChronoVehicleProperties(boolean enableVis)
	{
		// JSON file for vehicle model
		// String vehicle_file = "hmmwv/vehicle/HMMWV_Vehicle.json";
		// String vehicle_file = "hmmwv/vehicle/HMMWV_Vehicle_simple.json";
		// String vehicle_file = "hmmwv/vehicle/HMMWV_Vehicle_simple_lugged.json";
		// String vehicle_file = "hmmwv/vehicle/HMMWV_Vehicle_4WD.json";
		// String vehicle_file = "generic/vehicle/Vehicle_DoubleWishbones.json";
		// String vehicle_file = "generic/vehicle/Vehicle_DoubleWishbones_ARB.json";
		// String vehicle_file = "MAN_5t/vehicle/MAN_5t_Vehicle_4WD.json";
		// String vehicle_file = "generic/vehicle/Vehicle_MultiLinks.json";
		// String vehicle_file = "generic/vehicle/Vehicle_SolidAxles.json";
		// String vehicle_file = "generic/vehicle/Vehicle_ThreeAxles.json";
		// String vehicle_file = "generic/vehicle_multisteer/Vehicle_DualFront_Independent.json";
		// String vehicle_file = "generic/vehicle_multisteer/Vehicle_DualFront_Shared.json";
		// String vehicle_file = "generic/vehicle/Vehicle_MacPhersonStruts.json";
		// String vehicle_file = "generic/vehicle/Vehicle_SemiTrailingArm.json";
		// String vehicle_file = "generic/vehicle/Vehicle_ThreeLinkIRS.json";
		String vehicle_file = "generic/vehicle/Vehicle_Rafael.json";
		 
		// JSON files tire models (rigid)
		String tire_file = "generic/tire/RigidTire_Rafael.json";
		//String tire_file = "hmmwv/tire/HMMWV_RigidMeshTire.json"; // ERROR??
		// String tire_file = "MAN_5t/tire/MAN_5t_RigidTire.json";
		
		ChVector3d position = new ChVector3d();
		position.fromVector3f(9.5f,122,-78); // top
		//position.fromVector3f(-140,106.5f,-92); // 1. Kurve
		//position.fromVector3f(224,34,85); // 2.Kurve
		//position.fromVector3f(10,57.3f,47);
		
		ChQuaternion rotation = new ChQuaternion();
		rotation.fromQuaternion(new Quaternion().fromAngles(0, 180*FastMath.DEG_TO_RAD, 0));
		//ChQuaternion rotation = new ChQuaternion(1, 0, 0, 0);
		//ChQuaternion rotation = new ChQuaternion(0.866025f, 0, 0, 0.5f);
		//ChQuaternion rotation = new ChQuaternion(0.7071068f, 0, 0, 0.7071068f);
		//ChQuaternion rotation = new ChQuaternion(0.25882f, 0, 0, 0.965926f);
		chrono.initCar(vehicle_file, tire_file, position, rotation, enableVis);

		// JSON files for powertrain (simple)
		String powertrain_file = "generic/powertrain/SimplePowertrain.json";
		chrono.initPowertrain(powertrain_file);
		
		chrono.initDriver();
	}
	
	
	public void update(float fps)
	{
		if(!sim.isPause())
		{	
			if(!runChronoInThread)
				updateChrono();

			updateVisualVehicle();
		}
	}


	private void updateChrono()
	{
    	/*
    	System.err.println("steeringValue: " + steeringValue + "\naccelerationValue: " + accelerationValue +
    			"\nbrakeValue: " + brakeValue);
    	*/	
		
		long updTime = System.currentTimeMillis();
		
		// speed =   0 --> factor = 0.05
		// speed >= 10 --> factor = 1
		float factor = 1;
		float speed = getCurrentVehicleSpeedKmHour();
		if(speed < 10)
			factor = 0.05f + (speed * 0.095f);

		chrono.update(steeringValue, accelerationValue, factor * brakeValue, result);
		long updMillis = System.currentTimeMillis() - updTime;
		//System.err.println("speed: " + result.vehicle_speed + "\nacceleration: " + result.vehicle_acceleration.toVector3f());
		//System.err.println("Gear: " + result.getActGear() + " - Speed: " + result.getEngineSpeed());
  	
		long millis = System.currentTimeMillis() - initialTime;
		//System.err.println("time: " + result.time + " - step: " + result.step_number + " - actual: " + (float)(millis/1000f) +
		//	" - upd: " + updMillis + " ms");
		
    	/*
    	System.err.println("time: " + result.time + "\nstep_number: " + result.step_number + "\nsteering: " + result.steering);
    	System.err.println("x: " + result.chassisPosition.x + "\ny: " + result.chassisPosition.y + "\nz: " + result.chassisPosition.z);
    	System.err.println("orig. steering: " + steering + "\n");
    	*/
	}

	
	private int previous_step_number = -1;
	public void updateVisualVehicle()
	{
		if(result.step_number>previous_step_number)
		{
			Spatial chassis = sim.getSceneNode().getChild("chassis");
			chassis.setLocalTranslation(result.chassisPosition.toVector3f());
			chassis.setLocalRotation(result.chassisRotation.toQuaternion());
		
			int numberOfWheels = result.num_wheels;
			for(int i=0; i<numberOfWheels; i++)
			{
				Spatial wheel = sim.getSceneNode().getChild(WheelPos.values()[i].toString());
				wheel.setLocalTranslation(result.wheels[i].position.toVector3f());
				wheel.setLocalRotation(result.wheels[i].rotation.toQuaternion());
			}

			previous_step_number = result.step_number;
		}
	}
	
	
	public void close()
	{
		runThread = false;
		chrono.close();
	}

	
	public void steer(float steeringValue)
	{
		//System.err.println("steeringValue: " + steeringValue);
		this.steeringValue = steeringValue;
	}


	public void setAcceleratorPedalIntensity(float accelerationValue)
	{
		System.err.println("accelerationValue: " + accelerationValue);
		this.accelerationValue = accelerationValue;
	}


	public void setBrakePedalIntensity(float brakeValue)
	{
		System.err.println("brakeValue: " + brakeValue);
		this.brakeValue = brakeValue;
	}


	public float getMass() 
	{
		return result.mass;
	}

	
	public Vector3f getPosition() 
	{
		return result.chassisPosition.toVector3f();
	}
	
	
	public Quaternion getRotation() 
	{
		return result.chassisRotation.toQuaternion();
	}


	public float getCurrentVehicleSpeedKmHour()
	{
		return (float) result.vehicle_speed*3.6f;
	}


	public int getNumWheels() 
	{
		return result.num_wheels;
	}

	
}