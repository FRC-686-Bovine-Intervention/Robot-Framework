package frc.robot;

import frc.robot.controls.Controls;
import frc.robot.controls.Controls.ButtonControlEnum;
import frc.robot.controls.Controls.JoystickEnum;
import frc.robot.lib.util.RisingEdgeDetector;
import frc.robot.subsystems.TestClimber;
import frc.robot.subsystems.TestClimberCommand;
import frc.robot.subsystems.TestClimberStatus;
import frc.robot.subsystems.TestClimberStatus.ClimberState;
import frc.robot.subsystems.TestIntake;
import frc.robot.subsystems.TestIntakeCommand;
import frc.robot.subsystems.TestIntakeStatus.IntakeState;

public class DriverInteraction {
    private static DriverInteraction instance;
    public static DriverInteraction getInstance() {if(instance == null){instance = new DriverInteraction();}return instance;}

    private TestIntake intake;
    private TestClimber climber;
    private Controls controls;

    private double kClimberMaxPercent = 0.8;

    private DriverInteraction()
    {
        intake = TestIntake.getInstance();
        climber = TestClimber.getInstance();
        controls = Controls.getInstance();
    }
    
    public void run()
    {
        TestIntakeCommand intakeCmd = TestIntakeCommand.fromState(IntakeState.DEFENSE);
        TestClimberCommand climberCmd = TestClimberCommand.fromState(ClimberState.DEFENSE);

        intakeCmd = TestIntakeCommand.fromButtons(controls.getButton(ButtonControlEnum.INTAKE), controls.getButton(ButtonControlEnum.OUTTAKE));

        climberCmd = TestClimberCommand.fromButtons(controls.getButton(ButtonControlEnum.CLIMBER_NEXT_STATE), controls.getButton(ButtonControlEnum.CLIMBER_PREV_STATE), controls.getButton(ButtonControlEnum.CLIMBER_RESET_STATE));

        if (TestClimberStatus.getInstance().getIntakeCommand() != null)
        {
            intakeCmd = TestClimberStatus.getInstance().getIntakeCommand();
        }
        climberCmd.setPower(controls.getAxis(JoystickEnum.THRUSTMASTER).y*kClimberMaxPercent);

        intake.setIntakeCommand(intakeCmd);
        climber.setClimberCommand(climberCmd);
    }
}
