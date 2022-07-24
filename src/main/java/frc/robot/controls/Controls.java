package frc.robot.controls;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.drive.Vector2d;
import frc.robot.Constants;
import frc.robot.command_status.DriveCommand;

public class Controls {
    private static Controls instance;
    public static Controls getInstance() {if(instance == null){instance = new Controls();}return instance;}

    Joystick thrustmaster, buttonboard;

    public Controls()
    {
        thrustmaster =  new Joystick(Constants.kThrustmasterPort);
        buttonboard =   new Joystick(Constants.kButtonboardPort);
    }

    public enum JoystickEnum {THRUSTMASTER, BUTTONBOARD}
    
    public Vector2d getAxis(JoystickEnum joystick)
    {
        switch(joystick)
        {
            case THRUSTMASTER:  default:    return new Vector2d(-thrustmaster.getRawAxis(0),    -thrustmaster.getRawAxis(1));
            case BUTTONBOARD:               return new Vector2d(buttonboard.getRawAxis(0),      buttonboard.getRawAxis(1));
        }
    }

    public int getPOV(JoystickEnum joystick)
    {
        switch(joystick)
        {
            case THRUSTMASTER:  default:    return thrustmaster.getPOV();
            case BUTTONBOARD:               return buttonboard.getPOV();
        }
    }

    public enum ButtonControlEnum {
        INTAKE,
        OUTTAKE,
        CLIMBER_NEXT_STATE,
        CLIMBER_PREV_STATE,
        CLIMBER_RESET_STATE
    }
    
    public boolean getButton(ButtonControlEnum button)
    {
        switch(button)
        {
            case INTAKE:                    return thrustmaster.getRawButton(Thrustmaster.kTriggerButton);
            case OUTTAKE:                   return thrustmaster.getRawButton(Thrustmaster.kBottomThumbButton);
            case CLIMBER_NEXT_STATE:          return thrustmaster.getRawButton(Thrustmaster.kTopButton3);
            case CLIMBER_PREV_STATE:          return thrustmaster.getRawButton(Thrustmaster.kTopButton2);
            case CLIMBER_RESET_STATE:         return thrustmaster.getRawButton(Thrustmaster.kBottomButton1) && thrustmaster.getRawButton(Thrustmaster.kBottomButton2) && thrustmaster.getRawButton(Thrustmaster.kBottomButton3);
            default:                        return false;
        }
    }

    public DriveCommand getDriveCommand()
    {
        Vector2d a = getAxis(JoystickEnum.THRUSTMASTER);
        a.x = 0.8*a.x*a.x*a.x - 0.8*a.x + a.x;
        a.y = 0.7*a.y*a.y*a.y - 0.7*a.y + a.y;

        a.x *= 0.7;

        double leftPower = a.y-a.x;
        double rightPower = a.y+a.x;
        return new DriveCommand(leftPower, rightPower);
    }
}
