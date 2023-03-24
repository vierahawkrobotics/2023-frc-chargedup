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
import frc.robot.AutonmousCommands.AutoSetArmCommandHigh;
import frc.robot.AutonmousCommands.AutoSetArmCommandLow;
import frc.robot.AutonoumousRoutines.Autonomous;
import frc.robot.AutonoumousRoutines.Bal;
import frc.robot.AutonoumousRoutines.Middle;
import frc.robot.AutonoumousRoutines.SideAutoRoutine;
import frc.robot.Constants.ArmStates;
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
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.FollowPathWithEvents;

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
  //ClawSubsystem clawSubsystem;
  Autonomous auto;


  Lemonlight lemonlight;
  LEDSubsystems ledSubsystems;
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  XboxController joystickArm;
  XboxController joystick;
  SequentialCommandGroup balanceSequence;
  private final SendableChooser<SequentialCommandGroup> m_chooser;
  
  private SideAutoRoutine SideAuto;
  private Middle MiddleBalance;
  private Bal bal;
  

  public RobotContainer() {
    joystickArm = new XboxController(1);
    joystick = new XboxController(0);

    armSubsystem = new ArmSubsystem();
    // clawSubsystem = new PneumaticClawSubsystem();
    telescopeSubsystem = new TelescopeSubsystem();
    candleSubsystem = new CANdleSystem();
    auto = new Autonomous(m_robotDrive, armSubsystem, telescopeSubsystem, clawSubsystem);

    SideAuto = new SideAutoRoutine(m_robotDrive, armSubsystem, telescopeSubsystem, clawSubsystem);
    MiddleBalance = new Middle(m_robotDrive, armSubsystem, telescopeSubsystem, clawSubsystem);
    bal = new Bal(m_robotDrive, armSubsystem, telescopeSubsystem, clawSubsystem);

    m_chooser = new SendableChooser<>();
    m_chooser.setDefaultOption("Back Out", bal);
    m_chooser.addOption("Side Auto Routine", SideAuto);
    m_chooser.addOption("Middle Balance", MiddleBalance);
    SmartDashboard.putData(m_chooser);
  

    configureBindings();
  }

  // FYI: you can rebind the buttons on the back on the controller by holding down
  // the middle back button,
  // pressing the button you want to rebind to, and then the button you want to be
  // rebinded
  // example: Hold down Back Middle button, press A, press Back Left
  private void configureBindings() {
    // tab.add("X", 0.9).getEntry();
    // tab.add("Y", 0.1).getEntry();
    // armSubsystem.setDefaultCommand(new SetArmPosCommand(() -> {return
    // -joystick.getLeftY() * 2;}, armSubsystem));
    // armSubsystem.setDefaultCommand(new SetArmToPoint(() -> {return
    // joystick.getLeftX() * 2;},() -> {return -joystick.getLeftY() * 2 +
    // Constants.armHeight;},telescopeSubsystem, armSubsystem));
    
    
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
      
    new JoystickButton(joystickArm, 1).whileTrue(new RepeatCommand(new DepositCommand(clawSubsystem)));
    new JoystickButton(joystickArm, 2).onTrue(new CollectCommand(clawSubsystem));

    // new JoystickButton(joystickArm, Constants.XboxControllerButtonLayout.Y).onTrue(new SetArmStateCommand(Constants.ArmStates.Middle, armSubsystem, telescopeSubsystem));
    // new JoystickButton(joystickArm, Constants.XboxControllerButtonLayout.B).onTrue(new SetArmStateCommand(Constants.ArmStates.Ground, armSubsystem, telescopeSubsystem));
    
    new Trigger(joystickArm.povUp(new EventLoop()))
    .onTrue(new JoystickArmStateCommand(1, armSubsystem, telescopeSubsystem));
    new Trigger(joystickArm.povDown(new EventLoop()))
    .onTrue(new JoystickArmStateCommand(-1, armSubsystem, telescopeSubsystem));
    // new JoystickButton(joystick,
    // Constants.CandleConstants.BlockButton).whenPressed(ledSubsystems::setColors,
    // ledSubsystems);
    
 

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
        new RunCommand(() -> clawSubsystem.updatePID(), clawSubsystem));

  }

  public SequentialCommandGroup getAutonomousCommand(){
    return m_chooser.getSelected();
  }

  // public Command getAutonomousCommand() {
  //   HashMap<String, Command> eventMap = new HashMap<>();
  //   eventMap.put("RaiseArmToHigh", new AutoSetArmCommandHigh(Constants.ArmStates.High, armSubsystem, telescopeSubsystem));
  //   eventMap.put("LowerArmToGround", new AutoSetArmCommandLow(Constants.ArmStates.Ground, armSubsystem, telescopeSubsystem));
  //   eventMap.put("ToggleClaw", new SetClawCommand(ClawStates.Toggle, clawSubsystem));
  //  // eventMap.put("Wait", new WaitCommand(.5));
  //   //eventMap.put("Balance", new BalanceCommand(m_robotDrive));

  //   PathPlannerTrajectory TopBalance = PathPlanner.loadPath("TopBalance", new PathConstraints(4, 3));
  //   HashMap<String, Command> topEventMap = new HashMap<>();
  //   // topEventMap.put(null, getAutonomousCommand());

  //   PathPlannerTrajectory TopPlace = PathPlanner.loadPath("TopPlace", new PathConstraints(1, .5));
  //   HashMap<String, Command> topPlaceEventMap = new HashMap<>();

  //   PathPlannerTrajectory MiddleBalance = PathPlanner.loadPath("MiddleBalance", new PathConstraints(.5, .5));
  //   HashMap<String, Command> middleBalanceEventMap = new HashMap<>();

  //   PathPlannerTrajectory BottomBalance = PathPlanner.loadPath("BottomBalance", new PathConstraints(1, .5));
  //   HashMap<String, Command> bottomBalanceEventMap = new HashMap<>();

  //   PathPlannerTrajectory BottomPlace = PathPlanner.loadPath("BottomPlace", new PathConstraints(1, .5));
  //   HashMap<String, Command> bottomPlaceEventMap = new HashMap<>();

  //   PathPlannerTrajectory Straight = PathPlanner.loadPath("Straight", new PathConstraints(1, .5));

  //   PathPlannerTrajectory Simple = PathPlanner.loadPath("Simple", new PathConstraints(2, .5));

  //   PathPlannerTrajectory Middle = PathPlanner.loadPath("Middle", new PathConstraints(2, 1));

  //   PathPlannerTrajectory Bal = PathPlanner.loadPath("Bal", new PathConstraints(2,.5));


  //   //PathPlannerTrajectory Fun = PathPlanner.loadPath("Fun", new PathConstraints(2,.5));

  //   SwerveAutoBuilder swerveAutoBuilder = new SwerveAutoBuilder(
  //       m_robotDrive::getPose,
  //       m_robotDrive::resetOdometry,
  //       DriveConstants.kDriveKinematics,
  //       new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y
  //                                        // PID controllers)
  //       new PIDConstants(8, 0.0, 0.0),
  //       m_robotDrive::setModuleStates,
  //       eventMap,
  //       true,
  //       m_robotDrive);

  //   Command FullAuto = swerveAutoBuilder.fullAuto(Middle);

  //   // Run path following command, then stop at the end.
  //   // return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0,
  //   // false));
  //   return FullAuto;
  // }


  

  // public SequentialCommandGroup BalanceGroup(boolean Balance){

  //   if(Balance){
  //     balanceSequence = new SequentialCommandGroup(
  //     getAutonomousCommand(),
  //     new RepeatCommand(new BalanceCommand(m_robotDrive))
  //   );
  //   } else{
  //     balanceSequence = (SequentialCommandGroup)getAutonomousCommand();
  //   }
  
  //   return balanceSequence;

  // }


  // public SequentialCommandGroup MiddleBalance (){
  //   PathPoint startPoint = new PathPoint(new Translation2d(2.55, 3.23), new Rotation2d(0), new Rotation2d(180));
  //   PathPoint firstWayPathPoint = new PathPoint(new Translation2d(1.75, 3.23), new Rotation2d(0), new Rotation2d(180));
  //   PathPoint secondWayPathPoint = new PathPoint(new Translation2d(2.36, 3.23), new Rotation2d(0), new Rotation2d(180));
  //   PathPoint endPoint = new PathPoint(new Translation2d(3.9, 3.23), new Rotation2d(0), new Rotation2d(180));

  //   PathPlannerTrajectory firstPath = PathPlanner.generatePath(
  //     new PathConstraints(2, 1),
  //     startPoint, 
  //     firstWayPathPoint
  //   );

  //   PathPlannerTrajectory secondPath = PathPlanner.generatePath(
  //     new PathConstraints(2, 1),
  //     firstWayPathPoint, 
  //     secondWayPathPoint
  //   );

  //   PathPlannerTrajectory thirdPath = PathPlanner.generatePath(
  //     new PathConstraints(2, 1),
  //     secondWayPathPoint, 
  //     endPoint
  //   );

  //   MiddleBalance = new SequentialCommandGroup(
  //     new SetArmStateCommand(ArmStates.High, armSubsystem, telescopeSubsystem),
  //     new WaitCommand(.2),
  //     auto.getAutoPath(firstPath),
  //     //new SetClawCommand(ClawStates.Toggle, clawSubsystem),
  //     new WaitCommand(.2),
  //     auto.getAutoPath(secondPath),
  //     new SetArmStateCommand(ArmStates.Ground, armSubsystem, telescopeSubsystem),
  //     new WaitCommand(.5),
  //     auto.getAutoPath(thirdPath),
  //     new BalanceCommand(m_robotDrive)
  //   );

  //   return MiddleBalance;

  // }

  // public SequentialCommandGroup middleBalance(){
  //   return new Middle(m_robotDrive, armSubsystem, telescopeSubsystem, clawSubsystem);
  // }

  // public SequentialCommandGroup Bal(){
  //   return new Bal(m_robotDrive, armSubsystem, telescopeSubsystem, clawSubsystem);
  // }

  // public SequentialCommandGroup Sides(){
  //   return new SideAutoRoutinfe(m_robotDrive, armSubsystem, telescopeSubsystem, clawSubsystem);
  // }


}
