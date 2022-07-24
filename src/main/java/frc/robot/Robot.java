// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.auto.AutoManager;
import frc.robot.command_status.DriveState;
import frc.robot.command_status.RobotState;
import frc.robot.loops.DriveLoop;
import frc.robot.loops.LoopController;
import frc.robot.loops.RobotStateLoop;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.Vision;

public class Robot extends TimedRobot {

  SubsystemManager subsystemManager = SubsystemManager.getInstance();
  AutoManager autoManager = AutoManager.getInstance();
  DriverInteraction driverInteraction = DriverInteraction.getInstance();

  private NetworkTableEntry headingEntry = Shuffleboard.getTab("Robot Status").add("Heading Degrees", -999999).getEntry();
  private NetworkTableEntry averageDistanceEntry = Shuffleboard.getTab("Robot Status").add("Average Distance", -999999).getEntry();
  private NetworkTableEntry leftDistanceEntry = Shuffleboard.getTab("Robot Status").add("Left Distance", -999999).getEntry();
  private NetworkTableEntry rightDistanceEntry = Shuffleboard.getTab("Robot Status").add("Right Distance", -999999).getEntry();
  private NetworkTableEntry poseEntry = Shuffleboard.getTab("Robot Status").add("Pose", "not updating").getEntry();
  private double startingDistance;
  private double startingLeftDistance;
  private double startingRightDistance;

  private double getDistance()
    {
        return (DriveState.getInstance().getLeftDistanceInches() + DriveState.getInstance().getRightDistanceInches())/2;
    }

  @Override
  public void robotInit() {
    Vision.getInstance().init();

    DataLogManager.start();

    subsystemManager.init();
    autoManager.InitChoices();
    LoopController.getInstance().register(Drive.getInstance().getVelocityPIDLoop());
    LoopController.getInstance().register(DriveLoop.getInstance());
    LoopController.getInstance().register(RobotStateLoop.getInstance());
  }

  @Override
  public void robotPeriodic() {subsystemManager.updateShuffleboard(); LoopController.getInstance().run();
    averageDistanceEntry.setDouble(getDistance() - startingDistance);
    leftDistanceEntry.setDouble(DriveState.getInstance().getLeftDistanceInches() - startingLeftDistance);
    rightDistanceEntry.setDouble(DriveState.getInstance().getRightDistanceInches() - startingRightDistance);
    headingEntry.setDouble(RobotState.getInstance().getLatestFieldToVehicle().getHeadingDeg());
    poseEntry.setString(RobotState.getInstance().getLatestFieldToVehicle().toString());
  }

  @Override
  public void autonomousInit() {
    startingDistance = getDistance();
    startingLeftDistance = DriveState.getInstance().getLeftDistanceInches();
    startingRightDistance = DriveState.getInstance().getRightDistanceInches();
    autoManager.init();
  }

  @Override
  public void autonomousPeriodic() {
    subsystemManager.run();
  }

  @Override
  public void teleopInit() {
    startingDistance = getDistance();
    startingLeftDistance = DriveState.getInstance().getLeftDistanceInches();
    startingRightDistance = DriveState.getInstance().getRightDistanceInches();
    LoopController.getInstance().start();
    driverInteraction.init();
  }

  @Override
  public void teleopPeriodic() {
    subsystemManager.run();
    driverInteraction.run();
  }

  @Override
  public void disabledInit() {LoopController.getInstance().start(); autoManager.stop();}

  @Override
  public void disabledPeriodic() {
    subsystemManager.disable();
  }

  @Override
  public void testInit() {LoopController.getInstance().start();}

  @Override
  public void testPeriodic() {
    subsystemManager.run();
  }
}
