// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// Needs a joystick for the error to be solved
package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

//import java.util.function.Supplier;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.EventLoop;

public class RobotContainer {
  ArmSubsystem armSubsystem;
  ClawSubsystem clawSubsystem;
  TelescopeSubsystem telescopeSubsystem;
  TelescopeSubsystem tele;
  Lemonlight lemonlight;
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();


  public RobotContainer() {
    armSubsystem = new ArmSubsystem();
    clawSubsystem = new ClawSubsystem();
    telescopeSubsystem = new TelescopeSubsystem();
    configureBindings();
  }
//FYI: you can rebind the buttons on the back on the controller by holding down the middle back button, 
//pressing the button you want to rebind to, and then the button you want to be rebinded
//example: Hold down Back Middle button, press A, press Back Left
  private void configureBindings() {
    //tab.add("X", 0.9).getEntry();
    //tab.add("Y", 0.1).getEntry();
    XboxController joystickArm = new XboxController(1);
    XboxController joystick = new XboxController(0);
    //armSubsystem.setDefaultCommand(new SetArmPosCommand(() -> {return -joystick.getLeftY() * 2;}, armSubsystem));
    //armSubsystem.setDefaultCommand(new SetArmToPoint(() -> {return joystick.getLeftX() * 2;},() -> {return -joystick.getLeftY() * 2 + Constants.armHeight;},telescopeSubsystem, armSubsystem));
    new Trigger(joystickArm.povUp(new EventLoop())).onTrue(new JoystickArmStateCommand(1, armSubsystem, telescopeSubsystem));
    new Trigger(joystickArm.povDown(new EventLoop())).onTrue(new JoystickArmStateCommand(-1, armSubsystem, telescopeSubsystem));
    new JoystickButton(joystickArm, 1).onTrue(new SetClawCommand(ClawStates.Toggle, clawSubsystem));
    
    new JoystickButton(joystick, Button.kR1.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));

    

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                MathUtil.applyDeadband(joystick.getLeftY(), 0.06),
                MathUtil.applyDeadband(joystick.getLeftX(), 0.06),
                MathUtil.applyDeadband(-joystick.getRightX(), 0.06),
                true),
            m_robotDrive));
  }

  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);
    
    HashMap <String, Command> eventMap = new HashMap<>();
    eventMap.put("RaiseArmToHigh", ());
    eventMap.put("RaiseArmToStow", ());
    eventMap.put("RaiseArmToMid", ());
    eventMap.put("LowerArmToGround", ());
    eventMap.put("ReleaseClaw", new SetClawCommand(ClawStates.Open, clawSubsystem));
    eventMap.put("EngageClaw", new SetClawCommand(ClawStates.Closed, clawSubsystem));




        
    PathPlannerTrajectory TopBalance = PathPlanner.loadPath("TopBalance", new PathConstraints(4, 3));
    HashMap<String, Command> topEventMap = new HashMap<>();
    //topEventMap.put(null, getAutonomousCommand());

    PathPlannerTrajectory TopPlace = PathPlanner.loadPath("TopPlace", new PathConstraints(1, .5));
    HashMap<String, Command> topPlaceEventMap = new HashMap<>();

    PathPlannerTrajectory MiddleBalance = PathPlanner.loadPath("MiddleBalance", new PathConstraints(4, 3));
    HashMap<String, Command> middleBalanceEventMap = new HashMap<>();

    PathPlannerTrajectory BottomBalance = PathPlanner.loadPath("BottomBalance", new PathConstraints(4, 3));
    HashMap<String, Command> bottomBalanceEventMap = new HashMap<>();

    PathPlannerTrajectory BottomPlace = PathPlanner.loadPath("BottomPlace", new PathConstraints(2, .5));
    HashMap<String, Command> bottomPlaceEventMap = new HashMap<>();


    //eventMap.put("marker1", new PrintCommand("Passed marker 1"));
    //eventMap.put("intakeDown", new IntakeDown());


    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
   //if(lemonlight.aprilTag()){
     //m_robotDrive.resetOdometry(lemonlight.getPose2d());
    //}
    m_robotDrive.resetOdometry(m_robotDrive.getPose());

    SwerveAutoBuilder swerveAutoBuilder = new SwerveAutoBuilder(
        m_robotDrive::getPose,
        m_robotDrive::resetOdometry,
        DriveConstants.kDriveKinematics,
        new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
        new PIDConstants(0.5, 0.0, 0.0),
        m_robotDrive::setModuleStates,
        topPlaceEventMap,
        true,
        m_robotDrive
    );

    Command FullAuto = swerveAutoBuilder.fullAuto(TopPlace);

    // Run path following command, then stop at the end.
    //return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
    return FullAuto;
  }
}
