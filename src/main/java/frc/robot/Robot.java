// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.subsystems.SubsystemController;
import frc.robot.subsystems.TestClimber;
import frc.robot.subsystems.TestIntake;

public class Robot extends TimedRobot {

  SubsystemController subsystemController = SubsystemController.getInstance();
  @Override
  public void robotInit() {
    subsystemController.register(TestIntake.getInstance());
    subsystemController.register(TestClimber.getInstance());
    subsystemController.start();
  }

  @Override
  public void robotPeriodic() {
    
  }

  @Override
  public void autonomousInit() {

  }

  @Override
  public void autonomousPeriodic() {
    
  }

  @Override
  public void teleopInit() {

  }

  @Override
  public void teleopPeriodic() {
    DriverInteraction.getInstance().run();
  }

  @Override
  public void disabledInit() {

  }

  @Override
  public void disabledPeriodic() {

  }

  @Override
  public void testInit() {

  }

  @Override
  public void testPeriodic() {

  }
}
