package org.usfirst.frc.team686.robot.auto.actions;

import org.usfirst.frc.team686.robot.Constants;
import org.usfirst.frc.team686.robot.command_status.DriveState;
import org.usfirst.frc.team686.robot.lib.sensors.NavX;
import org.usfirst.frc.team686.robot.lib.util.DataLogger;

/**
 * Action for following a path defined by a Path object.
 * 
 * Serially configures a PathFollower object to follow each path 
 */
public class CollisionDetectionAction implements Action 
{
	public static NavX gyro;
	private double lastAccelX;
	private double lastAccelY;
	double jerkX = 0.0;
	double jerkY = 0.0;
	double lMotorCurrent = 0.0;
	double rMotorCurrent = 0.0;
	
    public CollisionDetectionAction() 
    {
    	gyro = NavX.getInstance();
    }

    @Override
    public void start() 
    {
		System.out.println("Starting CollisionDetectionAction");
    	lastAccelX = gyro.getWorldLinearAccelerationX();
    	lastAccelY = gyro.getWorldLinearAccelerationY();
    }


    @Override
    public void update() 
    {
    	// do nothing -- just waiting for a collision
    }	
	
	
    @Override
    public boolean isFinished() 
    {
        double accelX = gyro.getWorldLinearAccelerationX();
        double accelY = gyro.getWorldLinearAccelerationY();
        jerkX = accelX - lastAccelX;
        jerkY = accelY - lastAccelY;
        lastAccelX = accelX;
        lastAccelY = accelY;

        //System.out.println(this.toString());  
       
        boolean collisionDetected = false;
        if ( ( Math.abs(jerkX) > Constants.kCollisionJerkThreshold ) || ( Math.abs(jerkY) > Constants.kCollisionJerkThreshold) )
        	collisionDetected = true;
        
        lMotorCurrent = DriveState.getInstance().getLeftMotorCurrent();
        rMotorCurrent = DriveState.getInstance().getRightMotorCurrent();
        if ( ( lMotorCurrent > Constants.kCollisionCurrentThreshold ) || ( rMotorCurrent > Constants.kCollisionCurrentThreshold) )
        	collisionDetected = true;
        
    	return collisionDetected;
    }

    @Override
    public void done() 
    {
		System.out.println("Finished CollisionDetectionAction");
        System.out.println(this.toString());  
		
		// cleanup code, if any
    }

    public String toString()
    {
    	return String.format("Collision Detection -- JerkX: % 5.3f, JerkY: % 5.3f, JerkThresh: %4.1f, lMotorCurrent: % 5.3f, rMotorCurrent: % 5.3f, CurrentThresh: %4.1f, ", jerkX, jerkY, Constants.kCollisionJerkThreshold, lMotorCurrent, rMotorCurrent, Constants.kCollisionCurrentThreshold);
    }
    
    
	private final DataLogger logger = new DataLogger()
    {
        @Override
        public void log()
        {
    		put("AutoAction/AutoAction", "CollisionDetectionAction" );
   			put("CollisionDetectionAction/JerkX", jerkX );
   			put("CollisionDetectionAction/JerkY", jerkY );
   			put("CollisionDetectionAction/JerkThresh", Constants.kCollisionJerkThreshold );
   			put("CollisionDetectionAction/lMotorCurrent", jerkX );
   			put("CollisionDetectionAction/rMotorCurrent", jerkY );
   			put("CollisionDetectionAction/CurrentThresh", Constants.kCollisionCurrentThreshold );
        }
    };
     
    public DataLogger getLogger() { return logger; }
}
