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

import java.io.File;
import java.util.Random;

import com.jme3.bounding.BoundingBox;
import com.jme3.bullet.collision.shapes.CollisionShape;
import com.jme3.bullet.control.VehicleControl;
import com.jme3.bullet.joints.HingeJoint;
import com.jme3.bullet.objects.PhysicsRigidBody;
import com.jme3.bullet.util.CollisionShapeFactory;
import com.jme3.light.SpotLight;
import com.jme3.material.Material;
import com.jme3.math.ColorRGBA;
import com.jme3.math.FastMath;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.renderer.queue.RenderQueue.ShadowMode;
import com.jme3.scene.Geometry;
import com.jme3.scene.Node;
import com.jme3.scene.Spatial;
import com.jme3.scene.Spatial.CullHint;
import com.jme3.scene.debug.WireBox;
import com.jme3.scene.shape.Box;

import eu.opends.audio.AudioCenter;
import eu.opends.car.LightTexturesContainer.TurnSignalState;
import eu.opends.environment.GeoPosition;
import eu.opends.main.Simulator;
import eu.opends.tools.PanelCenter;
import eu.opends.tools.SpeedControlCenter;
import eu.opends.tools.Vector3d;

/**
 * 
 * @author Rafael Math
 */
public abstract class Car
{
	protected Simulator sim;
	protected Vector3f initialPosition;
	protected Quaternion initialRotation;
	public Geometry frontGeometry;
	protected Geometry centerGeometry;
	
	protected CarModelLoader carModel;
    public CarControl carControl;
    protected Node carNode;
    protected CarControl trailerControl;
    protected Node trailerNode;
    protected LightTexturesContainer lightTexturesContainer;
    
    private float steeringWheelState;
    protected float steeringInfluenceByCrosswind = 0;
    protected float acceleratorPedalIntensity;
    protected float brakePedalIntensity;
    protected float clutchPedalIntensity;
    protected float traction = 0.0f;
    protected int resetPositionCounter;
    protected Vector3f previousPosition;
    private float distanceOfCurrentFrame = 0;
    protected float mileage;
    protected boolean engineOn;

    protected float mass;
    protected boolean isAutoAcceleration = true;
    protected float targetSpeedCruiseControl = 0;
    protected boolean isCruiseControl = false;
    protected float minSpeed;
    protected float maxSpeed;
    protected float acceleration;
    protected float accelerationForce;
    protected float decelerationBrake;
    protected float maxBrakeForce;
    protected float decelerationFreeWheel;
    protected float maxFreeWheelBrakeForce;
    
    protected Transmission transmission;
    protected PowerTrain powerTrain;
    
    protected SpotLight leftHeadLight;
    protected SpotLight rightHeadLight;
    protected float lightIntensity = 0;
    protected String modelPath = "Test";

    
    protected void init()
    {
		previousPosition = initialPosition;
		resetPositionCounter = 0;
		mileage = 0;
		
        // load car model
		carModel = new CarModelLoader(sim, this, modelPath, mass, initialPosition, initialRotation);
		carControl = carModel.getCarControl();
		carNode = carModel.getCarNode();
		carNode.setShadowMode(ShadowMode.Cast);
		
		// generate path to light textures from model path
		File modelFile = new File(modelPath);
		String lightTexturesPath = modelFile.getPath().replace(modelFile.getName(), "lightTextures.xml");
		
		// load light textures
		lightTexturesContainer = new LightTexturesContainer(sim, this, lightTexturesPath);
		//lightTexturesContainer.printAllContent();
		       
        // add car to physics node
        if(carControl.isUseChrono())
        	sim.getChronoPhysicsSpace().add(carControl.getChronoVehicleControl());
        else
        	sim.getBulletPhysicsSpace().add(carControl.getBulletVehicleControl());

		// setup head light
        setupHeadlight(sim);
        
        // add trailer
        boolean hasTrailer = false;
        if(hasTrailer)
        	setupTrailer();

        // set initial position and orientation
        setPosition(initialPosition);
        setRotation(initialRotation);

        // apply continuous braking (simulates friction when free wheeling)
        resetPedals();
        
        setupReferencePoints();
    }

	
	private void setupHeadlight(Simulator sim) 
	{
		leftHeadLight = new SpotLight();
        leftHeadLight.setColor(ColorRGBA.White.mult(lightIntensity));
        leftHeadLight.setSpotRange(100);
        leftHeadLight.setSpotInnerAngle(11*FastMath.DEG_TO_RAD);
        leftHeadLight.setSpotOuterAngle(25*FastMath.DEG_TO_RAD);
        sim.getSceneNode().addLight(leftHeadLight);
        
        rightHeadLight = new SpotLight();
        rightHeadLight.setColor(ColorRGBA.White.mult(lightIntensity));
        rightHeadLight.setSpotRange(100);
        rightHeadLight.setSpotInnerAngle(11*FastMath.DEG_TO_RAD);
        rightHeadLight.setSpotOuterAngle(25*FastMath.DEG_TO_RAD);
        sim.getSceneNode().addLight(rightHeadLight);
	}
	
	
	private void setupTrailer() 
	{
		String trailerModelPath = "Models/Cars/drivingCars/CarRedTrailer/Car.scene";
		float trailerMass = 100;
		
		// load trailer (model and physics)
		CarModelLoader trailerModelLoader = new CarModelLoader(sim, this, trailerModelPath, 
				trailerMass, new Vector3f(), new Quaternion());
		trailerControl = trailerModelLoader.getCarControl();
		sim.getBulletPhysicsSpace().add(trailerControl);
		trailerNode = trailerModelLoader.getCarNode();
		sim.getSceneNode().attachChild(trailerNode);
		
		/*
		// apply joint
		HingeJoint joint=new HingeJoint(carNode.getControl(VehicleControl.class),
				trailerNode.getControl(VehicleControl.class),
		        new Vector3f(0f, 0f, 3f),    // pivot point local to carNode
		        new Vector3f(0f, 0f, -1.5f), // pivot point local to trailerNode 
		        Vector3f.UNIT_Y, 			 // DoF Axis of carNode (Y axis)
		        Vector3f.UNIT_Y);        	 // DoF Axis of trailerNode (Y axis)
		joint.setCollisionBetweenLinkedBodys(false);
		joint.setLimit(-FastMath.HALF_PI, FastMath.HALF_PI);	        
		sim.getPhysicsSpace().add(joint);
		*/
		
		Box sphere = new Box(0.1f, 0.1f, 0.1f);
		Geometry spatial = new Geometry("box", sphere);
		CollisionShape boxShape = CollisionShapeFactory.createBoxShape(spatial);
		PhysicsRigidBody connector = new PhysicsRigidBody(boxShape, 1f);
		sim.getBulletPhysicsSpace().add(connector);	
		
		
		// apply joint1
		HingeJoint joint1 = new HingeJoint(carNode.getControl(VehicleControl.class),
				connector,
		        new Vector3f(0f, 0f, 2.5f),  // pivot point local to carNode
		        new Vector3f(0f, 0f, 0f), 	 // pivot point local to connector 
		        Vector3f.UNIT_Y, 			 // DoF Axis of carNode (Y axis)
		        Vector3f.UNIT_Y);        	 // DoF Axis of connector (Y axis)
		joint1.setCollisionBetweenLinkedBodys(false);
		joint1.setLimit(-FastMath.HALF_PI, FastMath.HALF_PI);	        
		sim.getBulletPhysicsSpace().add(joint1);
		
		// apply joint
		HingeJoint joint2 = new HingeJoint(connector,
				trailerNode.getControl(VehicleControl.class),
		        new Vector3f(0f, 0f, 0f),    // pivot point local to connector
		        new Vector3f(0f, 0f, -1.5f), // pivot point local to trailerNode 
		        Vector3f.UNIT_Y, 			 // DoF Axis of connector (Y axis)
		        Vector3f.UNIT_Y);        	 // DoF Axis of trailerNode (Y axis)
		joint2.setCollisionBetweenLinkedBodys(false);
		joint2.setLimit(-FastMath.HALF_PI, FastMath.HALF_PI);	        
		sim.getBulletPhysicsSpace().add(joint2);
	}
	
	
	private void setupReferencePoints() 
	{
		// add node representing position of front box
		Box frontBox = new Box(0.01f, 0.01f, 0.01f);
		frontGeometry = new Geometry("frontBox", frontBox);
        frontGeometry.setLocalTranslation(0, 0, -1);
		Material frontMaterial = new Material(sim.getAssetManager(), "Common/MatDefs/Misc/Unshaded.j3md");
		frontMaterial.setColor("Color", ColorRGBA.Red);
		frontGeometry.setMaterial(frontMaterial);
		Node frontNode = new Node();
		frontNode.attachChild(frontGeometry);
		frontNode.setCullHint(CullHint.Always);
		getCarNode().attachChild(frontNode);
		
		// add node representing position of center box
		Box centerBox = new Box(0.01f, 0.01f, 0.01f);
		centerGeometry = new Geometry("centerBox", centerBox);
		centerGeometry.setLocalTranslation(0, 0, 0);
		Material centerMaterial = new Material(sim.getAssetManager(), "Common/MatDefs/Misc/Unshaded.j3md");
		centerMaterial.setColor("Color", ColorRGBA.Green);
		centerGeometry.setMaterial(centerMaterial);
		Node centerNode = new Node();
		centerNode.attachChild(centerGeometry);
		centerNode.setCullHint(CullHint.Always);
		getCarNode().attachChild(centerNode);
	}
	
	
	public float getMass()
	{
		return mass;
	}
	
	public float getMinSpeed()
	{
		return minSpeed;
	}
	
	public void setMinSpeed(float minSpeed)
	{
		this.minSpeed = minSpeed;
	}
	
	public float getMaxSpeed()
	{
		return maxSpeed;
	}
	
	public void setMaxSpeed(float maxSpeed)
	{
		this.maxSpeed = maxSpeed;
	}
	
	public float getAcceleration()
	{
		return acceleration;
	}
	
	public float getMaxBrakeForce()
	{
		return maxBrakeForce;
	}
	
	public float getDecelerationFreeWheel()
	{
		return decelerationFreeWheel;
	}
	
	
	public Node getCarNode()
	{
		return carNode;
	}

	
	public CarControl getCarControl()
	{
		return carControl;
	}
	
	public CarModelLoader getCarModel()
	{
		return carModel;
	}
	
	
	public Transmission getTransmission()
	{
		return transmission;
	}
	
	
	public PowerTrain getPowerTrain()
	{
		return powerTrain;
	}

	public void setToRandomResetPosition(){
		setToRandomResetPosition(true);
	}

	public void setToRandomResetPosition(boolean resetPeds){
		ResetPosition reset;
		int numberOfResetPoints = sim.trainingResetPoisitions.size();
		int chosenResetPosition = new Random().nextInt(numberOfResetPoints);
		reset = sim.trainingResetPoisitions.get(chosenResetPosition);

		sim.setPause(true);

		Vector3f location = reset.getLocation();
		Quaternion rotation = reset.getRotation();

		setPosition(location);
        carControl.setPhysicsRotation(rotation);
        carControl.resetVelocity();

		if(resetPeds){
			sim.resetPeds();
		}

		sim.setPause(false);
	}
	
	public void setToNextResetPosition() 
	{
		int numberOfResetPoints = sim.getResetPositionList().size();
		
		setToResetPosition(resetPositionCounter);
		resetPositionCounter = (resetPositionCounter + 1) % numberOfResetPoints;
	}

	
	public void setToResetPosition(int keyNumber) 
	{
		int numberOfResetPoints = sim.getResetPositionList().size();
		
		if (keyNumber < numberOfResetPoints) 
		{
			ResetPosition reset = sim.getResetPositionList().get(keyNumber);
			
			Vector3f location = reset.getLocation();
			Quaternion rotation = reset.getRotation();

			sim.setPause(true);
			setPosition(location);
			setRotation(rotation);
		}
		sim.setPause(false);
	}
	
	public void setPosition(Vector3f v) 
	{
		setPosition(v.x, v.y, v.z);
	}
	
	
	public void setPosition(float x, float y, float z) 
	{
		previousPosition = new Vector3f(x,y,z);
		
		carControl.setPhysicsLocation(previousPosition);
		carControl.resetVelocity();
		
		if(trailerControl != null)
		{
			trailerControl.setPhysicsLocation(previousPosition);
			trailerControl.resetVelocity();
		}
	}

	
	public Vector3f getPosition() 
	{
		//carControl.
		return carControl.getPhysicsLocation();
	}
	
	
	public Vector3d getGeoPosition() 
	{
		return GeoPosition.modelToGeo(getPosition());
	}

	
	public float getHeadingDegree() 
	{
		// get Euler angles from rotation quaternion
		float[] angles = carControl.getPhysicsRotation().toAngles(null);
		
		// heading in radians
		float heading = -angles[1];
		
		// normalize radian angle
		float fullAngle = 2*FastMath.PI;
		float angle_rad = (heading + fullAngle) % fullAngle;
		
		// convert radian to degree
		return angle_rad * 180/FastMath.PI;
	}
	
	
	public float getSlope()
	{
		// get Euler angles from rotation quaternion
		float[] angles = carControl.getPhysicsRotation().toAngles(null);
		
		// slope in radians (with correction due to different suspension heights)
		return angles[0] - 0.031765f;
	}
	
	
	public float getSlopeDegree()
	{
		// convert radian to degree and round to one decimal
		return ((int)(getSlope() * 180/FastMath.PI *10f))/10f;
	}
	
	
	public void setRotation(Quaternion q) 
	{	
		setRotation(q.getX(), q.getY(), q.getZ(), q.getW());
	}
	
	
	public void setRotation(float x, float y, float z, float w) 
	{
		Quaternion rotation = new Quaternion(x,y,z,w);
		
		// compensate that car is actually driving backwards
		float[] angles = rotation.toAngles(null);
		angles[1] = -angles[1]; //FIXME
		rotation = new Quaternion().fromAngles(angles);
		
		carControl.setPhysicsRotation(rotation);
		carControl.resetVelocity();
		
		if(trailerControl != null)
		{
			trailerControl.setPhysicsRotation(rotation);
			trailerControl.resetVelocity();
		}
	}
	
	
	public Quaternion getRotation() 
	{
		return carControl.getPhysicsRotation();
	}
	
	
	/**
	 * Accelerates the car forward or backwards. Does it by accelerating both
	 * suspensions (4WD). If you want a front wheel drive, comment out the
	 * rearSuspension.accelerate(direction) line. If you want a rear wheel drive
	 * car comment out the other one.
	 * 
	 * @param intensity
	 *            -1 for full ahead and 1 for full backwards
	 */
	public void setAcceleratorPedalIntensity(float intensity) 
	{
		acceleratorPedalIntensity = intensity;
	}

	
	public float getAcceleratorPedalIntensity() 
	{
		return Math.abs(acceleratorPedalIntensity);
	}


	/**
	 * Brake pedal
	 * 
	 * @param intensity
	 *            1 for full brake, 0 no brake at all
	 */
	public void setBrakePedalIntensity(float intensity) 
	{
		brakePedalIntensity = intensity;
		SpeedControlCenter.stopBrakeTimer();
	}
	
	
	public float getBrakePedalIntensity() 
	{
		return brakePedalIntensity;
	}

	
	/**
	 * Clutch pedal
	 * 
	 * @param intensity
	 *            1 for fully pressed, 0 for fully released clutch pedal
	 */
	public void setClutchPedalIntensity(float intensity)
	{		
		clutchPedalIntensity = intensity;
	}
	
	
	public float getClutchPedalIntensity() 
	{
		return clutchPedalIntensity;
	}

	
	float previousClutchPedalIntensity = 0;
	public float getTraction() 
	{
		if(clutchPedalIntensity < 0.4f && isEngineOn())
		{
			if(FastMath.abs(previousClutchPedalIntensity-clutchPedalIntensity) < 0.05f)
				traction = Math.min(1.0f, traction + 0.0005f);
			else
				traction = (0.4f - clutchPedalIntensity)*2.5f;
		}
		else
			traction = 0;
		
		previousClutchPedalIntensity = clutchPedalIntensity;
		
		return traction;
	}
	
	
	/**
	 * Free wheel
	 */
	public void resetPedals()
	{
		// reset pedals to initial position
		acceleratorPedalIntensity = 0;
		brakePedalIntensity = 0;
	}
	
	
	/**
	 * Steers the front wheels.
	 * 
	 * @param direction
	 *            1 for right and -1 for left
	 */
	public void steer(final float direction) 
	{
		carControl.steer(direction + steeringInfluenceByCrosswind);
		setSteeringWheelState(direction);
	}

	
	public void setSteeringWheelState(float steeringWheelState) 
	{
		this.steeringWheelState = steeringWheelState;
	}

	
	public float getSteeringWheelState() 
	{
		return steeringWheelState;
	}
	
	
	public float getSteeringInfluenceByCrosswind() 
	{
		return steeringInfluenceByCrosswind;
	}
	

	/**
	 * Unsteer the front wheels
	 */
	public void unsteer() 
	{
		carControl.steer(steeringInfluenceByCrosswind);
		setSteeringWheelState(0);
	}

	
	/**
	 * To get the car speed for using in a HUD
	 * 
	 * @return velocity of the car
	 */
	public float getCurrentSpeedMs() 
	{
		return (getCurrentSpeedKmh()/3.6f);
	}
	

	public float getCurrentSpeedMsRounded()
	{
		return ((int)(getCurrentSpeedMs() * 100)) / 100f;
	}
	
	
	public float getCurrentSpeedKmh()
	{
		return FastMath.abs(carControl.getCurrentVehicleSpeedKmHour());
	}
	
	
	public float getCurrentSpeedKmhRounded()
	{
		return ((int)(getCurrentSpeedKmh() * 100)) / 100f;
	}
	
	
	public float getMileage()
	{
		updateDistanceOfCurrentFrame();
		
		if(distanceOfCurrentFrame > 0.001f)
			mileage += distanceOfCurrentFrame;
		
		return mileage;
	}

	
	private void updateDistanceOfCurrentFrame()
	{
		// compute distance since last measurement
		Vector3f currentPosition = getPosition();
		distanceOfCurrentFrame = previousPosition.distance(currentPosition);
		
		// update values
		previousPosition = currentPosition;
	}
	
	
	public float getDistanceOfCurrentFrameInKm()
	{
		return distanceOfCurrentFrame/1000f;
	}
	
	
	public String getMileageString()
	{
		float mileage = getMileage();
		if(mileage < 1000)
			return ((int)mileage) + " m";
		else
			return ((int)(mileage/10f))/100f + " km";
	}
	
	
	public void resetMileage()
	{
		mileage = 0;
	}


	public Vector3f getInitialPosition() 
	{
		return initialPosition;
	}

	
	public Quaternion getInitialRotation() 
	{
		return initialRotation;
	}
	
	
	public void toggleLight() 
	{
		if(lightIntensity < 1)
			lightIntensity = 1;
		else if(lightIntensity < 2)
			lightIntensity = 2;
		else
			lightIntensity = 0;
	}
	
	
	public boolean isLightOn()
	{
		return (lightIntensity != 0);
	}
	

	public boolean isEngineOn() 
	{
		return engineOn;
	}
	
	
	public void setEngineOn(boolean engineOn) 
	{
		this.engineOn = engineOn;
		resetPedals();
		
		showEngineStatusMessage(engineOn);
		
		if(engineOn)
			AudioCenter.startEngine();
		else
			AudioCenter.stopEngine();
	}


	protected void showEngineStatusMessage(boolean engineOn) 
	{
		if(this instanceof SteeringCar && (!Simulator.isHeadLess))
		{
			if(engineOn)
				PanelCenter.getMessageBox().addMessage("Engine on", 2);
			else
				PanelCenter.getMessageBox().addMessage("Engine off. Press 'e' to start.", 0);
		}
	}

	
	public void setAutoAcceleration(boolean isAutoAcceleration) 
	{
		this.isAutoAcceleration = isAutoAcceleration;
	}
	
	
	public boolean isAutoAcceleration() 
	{
		return isAutoAcceleration;
	}

	
	public void setCruiseControl(boolean isCruiseControl) 
	{
		this.targetSpeedCruiseControl = getCurrentSpeedKmh();
		this.isCruiseControl = isCruiseControl;
	}
	
	
	public boolean isCruiseControl() 
	{
		return isCruiseControl;
	}
	
	
	public Simulator getSimulator() 
	{
		return sim;
	}


	public String getLightState() 
	{
		if(lightIntensity == 2)
			return "HighBeam";
		else if(lightIntensity == 1)
			return "LowBeam";
		else
			return "Off";
	}
	
	
	public void setBrakeLight(boolean setToOn)
	{
		lightTexturesContainer.setBrakeLight(setToOn);
	}
	
	
	public boolean isBrakeLightOn()
	{
		return lightTexturesContainer.isBrakeLightOn();
	}

	
	public void setTurnSignal(TurnSignalState turnSignalState)
	{
		lightTexturesContainer.setTurnSignal(turnSignalState);
	}
	

	public TurnSignalState getTurnSignal() 
	{
		return lightTexturesContainer.getTurnSignal();
	}
	
	
	public void close()
	{
		lightTexturesContainer.close();
	}
	
	
	public Spatial getFrontGeometry()
	{
		return frontGeometry;
	}
	
	
	public Spatial getCenterGeometry()
	{
		return centerGeometry;
	}
}
