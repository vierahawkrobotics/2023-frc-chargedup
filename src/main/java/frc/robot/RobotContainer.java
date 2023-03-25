// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Needs a joystick for the error to be solved
package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ClawStates;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import java.util.HashMap;
import java.util.List;

import javax.print.attribute.standard.Sides;
import javax.swing.text.AbstractDocument.LeafElement;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

//import java.util.function.Supplier;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotContainer {
  ArmSubsystem armSubsystem;
  motorClawSubsystem clawSubsystem;
  TelescopeSubsystem telescopeSubsystem;
  CANdleSystem candleSubsystem;

  SendableChooser<Command> m_chooser = new SendableChooser<>();

  Lemonlight lemonlight;
  LEDSubsystems ledSubsystems;
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  XboxController joystickArm;
  XboxController joystick;
  SequentialCommandGroup balanceSequence;

  public RobotContainer() {
    joystickArm = new XboxController(1);
    joystick = new XboxController(0);

    armSubsystem = new ArmSubsystem();
    // clawSubsystem = new PneumaticClawSubsystem();
    telescopeSubsystem = new TelescopeSubsystem();
    candleSubsystem = new CANdleSystem();

    clawSubsystem = new motorClawSubsystem();

    SmartDashboard.putData(m_chooser);

    configureBindings();
  }

  // FYI: you can rebind the buttons on the back on the controller by holding down
  // the middle back button,
  // pressing the button you want to rebind to, and then the button you want to be
  // rebinded
  // example: Hold down Back Middle button, press A, press Back Left
  private void configureBindings() {
    new Trigger(joystickArm.povUp(new EventLoop()))
        .onTrue(new JoystickArmStateCommand(1, armSubsystem, telescopeSubsystem));
    new Trigger(joystickArm.povDown(new EventLoop()))
        .onTrue(new JoystickArmStateCommand(-1, armSubsystem, telescopeSubsystem));
    // new JoystickButton(joystickArm, 1).onTrue(new SetClawCommand(ClawStates.Toggle, clawSubsystem));

    new JoystickButton(joystick, 1).whileTrue(new InstantCommand(() -> m_robotDrive.zeroHeading(), m_robotDrive));

    new JoystickButton(joystick, 2).whileTrue(new RepeatCommand(new BalanceCommand(m_robotDrive)));
    
    new JoystickButton(joystick, Button.kR1.value)
    .whileTrue(new RunCommand(
      () -> m_robotDrive.setX(),
      m_robotDrive));

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        new RunCommand(
            () -> m_robotDrive.drive(
                MathUtil.applyDeadband(-joystick.getLeftY(), 0.15),
                MathUtil.applyDeadband(-joystick.getLeftX(), 0.15),
                MathUtil.applyDeadband(-joystick.getRightX(), 0.15),
                true),
            m_robotDrive));

    armSubsystem.setDefaultCommand(
        new RunCommand(() -> armSubsystem.updatePID(MathUtil.clamp(joystickArm.getLeftY(),
            -Constants.RotationArmConstants.angleAdjustmentRange, Constants.RotationArmConstants.angleAdjustmentRange)),
            armSubsystem));

    telescopeSubsystem.setDefaultCommand(
        new RunCommand(() -> telescopeSubsystem.setPosition(), telescopeSubsystem));

    clawSubsystem.setDefaultCommand(
        new RunCommand(() -> clawSubsystem.updatePID(joystickArm.getAButton(), joystickArm.getBButton(), joystickArm.getXButton(), joystickArm.getYButton()), clawSubsystem));


  }

  public Command getAutonomousCommand() {
    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("RaiseArmToHigh",
        new SetArmStateCommand(Constants.ArmStates.Middle, armSubsystem, telescopeSubsystem));
    eventMap.put("RaiseArmToLow", new SetArmStateCommand(Constants.ArmStates.Low, armSubsystem, telescopeSubsystem));
    eventMap.put("RaiseArmToMid", new SetArmStateCommand(Constants.ArmStates.Middle, armSubsystem, telescopeSubsystem));
    eventMap.put("LowerArmToGround",
        new SetArmStateCommand(Constants.ArmStates.Ground, armSubsystem, telescopeSubsystem));
    // eventMap.put("OpenClaw", new SetClawCommand(ClawStates.Open, clawSubsystem));
    // eventMap.put("CloseClaw", new SetClawCommand(ClawStates.Closed, clawSubsystem));
    // eventMap.put("ToggleClaw", new SetClawCommand(ClawStates.Toggle, clawSubsystem));
    eventMap.put("Wait", new WaitCommand(.5));
    // eventMap.put("Balance", new BalanceCommand(m_robotDrive));

    PathPlannerTrajectory TopBalance = PathPlanner.loadPath("TopBalance", new PathConstraints(4, 3));
    HashMap<String, Command> topEventMap = new HashMap<>();
    // topEventMap.put(null, getAutonomousCommand());

    PathPlannerTrajectory TopPlace = PathPlanner.loadPath("TopPlace", new PathConstraints(1, .5));
    HashMap<String, Command> topPlaceEventMap = new HashMap<>();

    PathPlannerTrajectory MiddleBalance = PathPlanner.loadPath("MiddleBalance", new PathConstraints(.5, .5));
    HashMap<String, Command> middleBalanceEventMap = new HashMap<>();

    PathPlannerTrajectory BottomBalance = PathPlanner.loadPath("BottomBalance", new PathConstraints(1, .5));
    HashMap<String, Command> bottomBalanceEventMap = new HashMap<>();

    PathPlannerTrajectory BottomPlace = PathPlanner.loadPath("BottomPlace", new PathConstraints(1, .5));
    HashMap<String, Command> bottomPlaceEventMap = new HashMap<>();

    PathPlannerTrajectory Straight = PathPlanner.loadPath("Straight", new PathConstraints(1, .5));

    PathPlannerTrajectory Simple = PathPlanner.loadPath("Simple", new PathConstraints(2, .5));

    PathPlannerTrajectory Middle = PathPlanner.loadPath("Middle", new PathConstraints(2, 1));

    PathPlannerTrajectory Sides = PathPlanner.loadPath("Sides", new PathConstraints(.5, .5));

    PathPlannerTrajectory Bal = PathPlanner.loadPath("Bal", new PathConstraints(2, .5));

    // PathPlannerTrajectory Fun = PathPlanner.loadPath("Fun", new
    // PathConstraints(2,.5));

    SwerveAutoBuilder swerveAutoBuilder = new SwerveAutoBuilder(
        m_robotDrive::getPose,
        m_robotDrive::resetOdometry,
        DriveConstants.kDriveKinematics,
        new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y
                                         // PID controllers)
        new PIDConstants(8, 0.0, 0.0),
        m_robotDrive::setModuleStates,
        eventMap,
        true,
        m_robotDrive);

    Command FullAuto = swerveAutoBuilder.fullAuto(Bal);

    // Run path following command, then stop at the end.
    // return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0,
    // false));
    return FullAuto;
  }

  public SequentialCommandGroup BalanceGroup(boolean Balance) {

    if (Balance) {
      balanceSequence = new SequentialCommandGroup(
          getAutonomousCommand(),
          new RepeatCommand(new BalanceCommand(m_robotDrive)));
    } else {
      balanceSequence = (SequentialCommandGroup) getAutonomousCommand();
    }

    return balanceSequence;

  }
}
