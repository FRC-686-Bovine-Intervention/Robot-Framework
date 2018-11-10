package org.usfirst.frc.team686.robot.lib.joystick;

import org.usfirst.frc.team686.robot.lib.joystick.JoystickControlsBase;
import org.usfirst.frc.team686.robot.lib.util.Util;
import org.usfirst.frc.team686.robot.Constants;
import org.usfirst.frc.team686.robot.command_status.DriveCommand;

/**
 * Implements a simple arcade drive, where single stick is used for throttle and turn.
 */
public class ArcadeDriveJoystick extends JoystickControlsBase 
{
    private static JoystickControlsBase mInstance = new ArcadeDriveJoystick();

    public static JoystickControlsBase getInstance() 
    {
        return mInstance;
    }

    
    public DriveCommand getDriveCommand()
    {
//	    boolean squaredInputs = true;	// set to true to increase fine control while permitting full power
	    boolean squaredInputs = false;	// set to true to increase fine control while permitting full power
	    
    	double throttle = -mStick.getY();	// TODO: figure out why Y-axis is negated
        double turn     = -mStick.getX();	// TODO: figure out why X-axis is negated
        
        if(throttle < Constants.kDriveDeadzone && throttle > -Constants.kDriveDeadzone){throttle = 0;}
        if(turn < Constants.kDriveDeadzone && turn > -Constants.kDriveDeadzone){turn = 0;}
     
	    double moveValue   = Util.limit(throttle, 1.0);
	    double rotateValue = Util.limit(turn,     1.0);
	    double lMotorSpeed, rMotorSpeed;
	    
	    if (squaredInputs) {
	      // square the inputs (while preserving the sign) to increase fine control
	      // while permitting full power
	      if (moveValue >= 0.0) {
	        moveValue = (moveValue * moveValue);
	      } else {
	        moveValue = -(moveValue * moveValue);
	      }
	      if (rotateValue >= 0.0) {
	        rotateValue = (rotateValue * rotateValue);
	      } else {
	        rotateValue = -(rotateValue * rotateValue);
	      }
	    }
	
	    if (moveValue > 0.0) {
	      if (rotateValue > 0.0) {
	        lMotorSpeed = moveValue - rotateValue;
	        rMotorSpeed = Math.max(moveValue, rotateValue);
	      } else {
	        lMotorSpeed = Math.max(moveValue, -rotateValue);
	        rMotorSpeed = moveValue + rotateValue;
	      }
	    } else {
	      if (rotateValue > 0.0) {
	        lMotorSpeed = -Math.max(-moveValue, rotateValue);
	        rMotorSpeed = moveValue + rotateValue;
	      } else {
	        lMotorSpeed = moveValue - rotateValue;
	        rMotorSpeed = -Math.max(-moveValue, -rotateValue);
	      }
	    }
	    
	    DriveCommand signal = new DriveCommand(lMotorSpeed, rMotorSpeed);
	   	    
	    return signal;        
    }
}
