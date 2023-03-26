// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.robot.commands.BalanceCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;

public class Robot extends TimedRobot {

  UsbCamera camera1;
  UsbCamera camera2;
  VideoSink server;
  NetworkTableEntry cameraSelection;

  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    CameraServer.startAutomaticCapture();
    // camera1 = CameraServer.startAutomaticCapture(0);
    // cameraSelection = NetworkTableInstance.getDefault().getTable("").getEntry("CameraSelection");
    // server = CameraServer.getServer();
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    // server.setSource(camera1);
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {

  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
    m_robotContainer.armSubsystem.setTargetRadianUsingState(Constants.ArmStates.Ground);
    m_robotContainer.telescopeSubsystem.setLength(Constants.ArmStates.Ground);
    m_robotContainer.telescopeSubsystem.telePID.reset();
    m_robotContainer.armSubsystem.elbowPID.reset();
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getMainMiddle();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    //m_autonomousCommand.execute();
  }

  @Override
  public void autonomousExit() {
    BalanceCommand.isFinished = true;
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }
}
