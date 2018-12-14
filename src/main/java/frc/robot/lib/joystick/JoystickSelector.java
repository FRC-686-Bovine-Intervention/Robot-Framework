package frc.robot.lib.joystick;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Select joystick control configuration using SmartDashboard / SendableChooser
 */
public class JoystickSelector
{
    SendableChooser<JoystickOption> joystickChooser;

    enum JoystickOption
    {
        ARCADE_DRIVE, 
        TRIGGER_DRIVE, // works for Xbox controller and Xbox steering wheel
        TANK_DRIVE, 
        CHEESY_ARCADE_DRIVE, 
        CHEESY_TRIGGER_DRIVE, 
        CHEESY_2STICK_DRIVE;
    }

    public JoystickSelector()
    {
        joystickChooser = new SendableChooser<JoystickOption>();
        joystickChooser.addObject("Arcade Drive", JoystickOption.ARCADE_DRIVE);
        joystickChooser.addDefault("Trigger Drive", JoystickOption.TRIGGER_DRIVE);
        joystickChooser.addObject("Tank Drive", JoystickOption.TANK_DRIVE);
        joystickChooser.addObject("Cheesy Arcade Drive", JoystickOption.CHEESY_ARCADE_DRIVE);
        joystickChooser.addObject("Cheesy Trigger Drive", JoystickOption.CHEESY_TRIGGER_DRIVE);
        joystickChooser.addObject("Cheesy Two-Stick Drive", JoystickOption.CHEESY_2STICK_DRIVE);
        SmartDashboard.putData("Joystick Chooser", joystickChooser);
    }

    public JoystickControlsBase getJoystickControlsMode()
    {
        JoystickOption selMode = (JoystickOption) joystickChooser.getSelected();

        switch (selMode)
        {
        case ARCADE_DRIVE:
            return ArcadeDriveJoystick.getInstance();

        case TRIGGER_DRIVE:
            return TriggerDriveJoystick.getInstance();

        case TANK_DRIVE:
            return TankDriveJoystick.getInstance();

        case CHEESY_ARCADE_DRIVE:
            return CheesyArcadeDriveJoystick.getInstance();

        case CHEESY_TRIGGER_DRIVE:
            return CheesyTriggerDriveJoystick.getInstance();

        case CHEESY_2STICK_DRIVE:
            return CheesyTwoStickDriveJoystick.getInstance();

        default:
            System.out.println("ERROR: unexpected joystick selection: " + selMode);
            return ArcadeDriveJoystick.getInstance();
        }

    }
}
