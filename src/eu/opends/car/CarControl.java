package eu.opends.car;

import com.jme3.bullet.control.VehicleControl;
import com.jme3.bullet.objects.VehicleWheel;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;

import eu.opends.chrono.ChronoVehicleControl;

public class CarControl
{
	private enum PhysicsType
	{
		BULLET, CHRONO;
	}

	private PhysicsType type;
	private VehicleControl bulletVehicleControl;
	private ChronoVehicleControl chronoVehicleControl;


	public CarControl(VehicleControl bulletVehicleControl)
	{
		type = PhysicsType.BULLET;
		this.bulletVehicleControl = bulletVehicleControl;
		this.chronoVehicleControl = null;
	}


	public CarControl(ChronoVehicleControl chronoVehicle)
	{
		type = PhysicsType.CHRONO;
		this.bulletVehicleControl = null;
		this.chronoVehicleControl = chronoVehicle;
	}


	public boolean isUseChrono()
	{
		return (type == PhysicsType.CHRONO);
	}


	public boolean isUseBullet()
	{
		return (type == PhysicsType.BULLET);
	}


	public VehicleControl getBulletVehicleControl()
	{
		return bulletVehicleControl;
	}


	public ChronoVehicleControl getChronoVehicleControl()
	{
		return chronoVehicleControl;
	}


	public synchronized void resetVelocity()
	{
		if(type == PhysicsType.BULLET)
		{
			bulletVehicleControl.setLinearVelocity(Vector3f.ZERO);
			bulletVehicleControl.setAngularVelocity(Vector3f.ZERO);
			bulletVehicleControl.resetSuspension();
		}
		else
		{
			System.err.println("Cannot reset velocity of Chrono vehicles");
		}
	}


	public synchronized void setPhysicsLocation(Vector3f pos)
	{
		if(type == PhysicsType.BULLET)
			bulletVehicleControl.setPhysicsLocation(pos);
		else
			System.err.println("Cannot set location of Chrono vehicles");
	}


	public synchronized Vector3f getPhysicsLocation()
	{
		if(type == PhysicsType.BULLET)
			return bulletVehicleControl.getPhysicsLocation();
		else
			return chronoVehicleControl.getPosition();
	}


	public synchronized void setPhysicsRotation(Quaternion rot)
	{
		if(type == PhysicsType.BULLET)
			bulletVehicleControl.setPhysicsRotation(rot);
		else
			System.err.println("Cannot set rotation of Chrono vehicles");
	}


	public synchronized Quaternion getPhysicsRotation()
	{
		if(type == PhysicsType.BULLET)
			return bulletVehicleControl.getPhysicsRotation();
		else
			return chronoVehicleControl.getRotation();
	}


	public synchronized void steer(float steering)
	{
		if(type == PhysicsType.BULLET)
			bulletVehicleControl.steer(steering);
		else
			chronoVehicleControl.steer(steering);
	}


	public synchronized void accelerate(float acceleration)
	{
		//System.err.println("accelerationValue: " + acceleration);

		if(type == PhysicsType.BULLET)
		{
			// apply double force to front wheels instead of single force to all wheels
			bulletVehicleControl.accelerate(0, 2*acceleration);
			bulletVehicleControl.accelerate(1, 2*acceleration);
		}
		else
			chronoVehicleControl.setAcceleratorPedalIntensity(-acceleration/1000f);
	}


	public synchronized void brake(float brake)
	{
		//System.err.println("brakeValue: " + brake);

		if(type == PhysicsType.BULLET)
			bulletVehicleControl.brake(brake);
		else
			chronoVehicleControl.setBrakePedalIntensity(brake/32f);
	}


	public synchronized float getCurrentVehicleSpeedKmHour()
	{
		if(type == PhysicsType.BULLET)
			return bulletVehicleControl.getCurrentVehicleSpeedKmHour();
		else
			return chronoVehicleControl.getCurrentVehicleSpeedKmHour();
	}


	public int getNumWheels()
	{
		if(type == PhysicsType.BULLET)
			return bulletVehicleControl.getNumWheels();
		else
			return chronoVehicleControl.getNumWheels();
	}


	public void setFrictionSlip(int i, float friction)
	{
		if(type == PhysicsType.BULLET)
			bulletVehicleControl.setFrictionSlip(i, friction);
		else
			System.err.println("Cannot set friction slip of Chrono vehicles");
	}


	public VehicleWheel getBulletWheel(int wheel)
	{
		if(type == PhysicsType.BULLET)
			return bulletVehicleControl.getWheel(wheel);
		else
			return null;
	}


	public float getMass()
	{
		if(type == PhysicsType.BULLET)
			return bulletVehicleControl.getMass();
		else
		{
			//System.err.println("Mass: " + chronoVehicleControl.getMass());
			return chronoVehicleControl.getMass();
		}
	}


	public Vector3f getLinearVelocity()
	{
		if(type == PhysicsType.BULLET)
			return bulletVehicleControl.getLinearVelocity();
		else
			return new Vector3f(0,0,0); //FIXME (only needed for Motion Seat)
	}



}