package org.usfirst.frc.team686.robot.lib.joystick;

import org.usfirst.frc.team686.robot.lib.joystick.JoystickControlsBase;
import org.usfirst.frc.team686.robot.Constants;
import org.usfirst.frc.team686.robot.command_status.DriveCommand;

/**
 * For use with Xbox steering wheel
 */
public class TankDriveJoystick extends JoystickControlsBase 
{
    private static JoystickControlsBase mInstance = new TankDriveJoystick();

    public static JoystickControlsBase getInstance() 
    {
        return mInstance;
    }

    
    public DriveCommand getDriveCommand()
    {
    	double lMotorSpeed = -mStick.getRawAxis(Constants.kXboxLStickYAxis);
        double rMotorSpeed = -mStick.getRawAxis(Constants.kXboxRStickYAxis);
	    
	    DriveCommand signal = new DriveCommand(lMotorSpeed, rMotorSpeed);
	   	    
	    return signal;        
    }
}
