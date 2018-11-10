package org.usfirst.frc.team686.robot.lib.sensors;

import org.usfirst.frc.team686.robot.Constants;
import com.kauailabs.navx.frc.AHRS;

public class NavX extends GyroBase
{
	private static NavX instance = new NavX();
	public static NavX getInstance() { return instance; }
	
	 AHRS ahrs;
	
    // constructors
    public NavX() 
    {
    	ahrs = new AHRS(Constants.NAVX_PORT, Constants.NAVX_UPDATE_RATE);
    }
	
	/**
	 * Returns heading for the GyroBase class.
	 *
	 */
	public double getHeadingDeg() {
		return -ahrs.getAngle();	// sign correction so that heading increases as robot turns to the left	 
	}
	
	public double getWorldLinearAccelerationX(){
		return ahrs.getWorldLinearAccelX();
	}
	
	public double getWorldLinearAccelerationY(){
		return ahrs.getWorldLinearAccelY();
	}
}
