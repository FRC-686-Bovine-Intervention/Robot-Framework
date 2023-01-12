// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import frc.robot.subsystems.SubsystemController;
import frc.robot.subsystems.TestBoard;

public class Robot extends LoggedRobot {
  SubsystemController subsystemController = SubsystemController.getInstance();

  @Override
  public void robotInit() {
    setUseTiming(isReal()); // Run as fast as possible during replay
    //LoggedNetworkTables.getInstance().addTable("/SmartDashboard"); // Log & replay "SmartDashboard" values (no tables are logged by default).
    Logger.getInstance().recordMetadata("ProjectName", "MyProject"); // Set a metadata value

    if (isReal()) {
      Logger.getInstance().addDataReceiver(new WPILOGWriter("/media/sda1"));
      Logger.getInstance().addDataReceiver(new NT4Publisher());
  } else {
      String logPath = LogFileUtil.findReplayLog();
      Logger.getInstance().setReplaySource(new WPILOGReader(logPath));
      Logger.getInstance().addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
  }

    Logger.getInstance().start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.

    subsystemController.register(TestBoard.getInstance());
    subsystemController.start();
    io.github.oblarg.oblog.Logger.configureLoggingAndConfig(this, false);
  }

  @Override
  public void robotPeriodic() {
    subsystemController.run();
    io.github.oblarg.oblog.Logger.updateEntries();
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
