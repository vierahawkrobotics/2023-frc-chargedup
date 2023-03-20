// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;

import java.security.spec.MGF1ParameterSpec;
import java.util.Map;

import com.fasterxml.jackson.databind.deser.impl.ExternalTypeHandler.Builder;
import com.kauailabs.navx.frc.AHRS;

import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.BalanceCommand;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

import java.util.List;

import org.ejml.dense.row.decompose.lu.LUDecompositionAlt_CDRM;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor

  // Shuffleboard entries
  private static GenericEntry targetSpeedWidget;
  private static GenericEntry translationalSpeedWidget;
  private static GenericEntry currentSpeedDrivetrainWidget;
  private static GenericEntry currentRotSpeedWidget;
  private static GenericEntry pitchWidget;
  private static GenericEntry rollWidget;
  private static GenericEntry yawWidget;
  private static GenericEntry xPowerWidget;
  private static GenericEntry yPowerWidget;
  private static GenericEntry rotPowerWidget;
  private static GenericEntry TranslationalSpeedCapWidget;
  private static GenericEntry RotationalSpeedCapWidget;
  
  private static ComplexWidget zeroYaw;

  // Variables for shuffleboard
  private static double targSpeed = 0;
  private static double translationalSpeed = 0;
  private static double currentRotationSpeed = 0;
  private static double xPower = 0;
  private static double yPower = 0;
  private static double rotPower = 0;
  public final AHRS m_gyro = new AHRS(SPI.Port.kMXP);
  private AutoConstants autoConstants = new AutoConstants();
  private final Lemonlight lemonlight = new Lemonlight();

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(m_gyro.getAngle()),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    m_gyro.calibrate();
    initShuffleboard();
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        Rotation2d.fromDegrees(-m_gyro.getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()

        });

    updateSB();
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    // if(lemonlight.aprilTag()){
    //   return lemonlight.getPose2d();
    // }
    // else {
      return m_odometry.getPoseMeters();
    //}
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
      m_odometry.resetPosition(
        Rotation2d.fromDegrees(-m_gyro.getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);

  }
    
  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {

    DriveSubsystem.xPower = xSpeed;
    DriveSubsystem.yPower = ySpeed;
    DriveSubsystem.rotPower = rot;

    // Adjust input based on max speed
    xSpeed *= DriveConstants.kMaxSpeedMetersPerSecond;
    ySpeed *= DriveConstants.kMaxSpeedMetersPerSecond;
    rot *= DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Rotation2d.fromDegrees(-m_gyro.getAngle()))
            : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);

    // Update variables for shuffleboard
    DriveSubsystem.targSpeed = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));
    DriveSubsystem.translationalSpeed = m_frontLeft.m_drivingEncoder.getVelocity();
    DriveSubsystem.currentRotationSpeed = Math.toRadians(m_gyro.getRate());
  }
  
  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
    m_gyro.calibrate();
    System.out.println("zeroed heading");
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return (-m_gyro.getAngle());
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  private void updateSB() {
    targetSpeedWidget.setDouble(targSpeed);
    currentSpeedDrivetrainWidget.setDouble(translationalSpeed);

    translationalSpeedWidget.setDouble(translationalSpeed);
    currentRotSpeedWidget.setDouble(currentRotationSpeed);
    pitchWidget.setDouble(m_gyro.getRoll());
    rollWidget.setDouble(m_gyro.getPitch());
    yawWidget.setDouble(-m_gyro.getAngle());

    xPowerWidget.setDouble(xPower);
    yPowerWidget.setDouble(yPower);
    rotPowerWidget.setDouble(rotPower);

    Constants.DriveConstants.kMaxSpeedMetersPerSecond = TranslationalSpeedCapWidget.getDouble(Constants.DriveConstants.kMaxSpeedMetersPerSecond);
    Constants.DriveConstants.kMaxAngularSpeed = RotationalSpeedCapWidget.getDouble(Constants.DriveConstants.kMaxAngularSpeed);
  }

  private void initShuffleboard() {
    // Testing
    targetSpeedWidget = Shuffleboard.getTab("Drivetrain").add("targetSpeed", 0).withWidget(BuiltInWidgets.kDial).withProperties(Map.of("min", 0, "max", 5)).withPosition(0, 0).getEntry();
    currentSpeedDrivetrainWidget = Shuffleboard.getTab("Drivetrain").add("translationalSpeed", 0).withSize(1, 1).getEntry();
    
      
    // Final Driverstation
    TranslationalSpeedCapWidget = Shuffleboard.getTab("Main").getLayout("Speed Caps", BuiltInLayouts.kList).withPosition(7, 0).withSize(1, 2).withProperties(Map.of("Label Position", "TOP")).add("Translational", Constants.DriveConstants.kMaxSpeedMetersPerSecond).withSize(1, 2).getEntry();
    RotationalSpeedCapWidget = Shuffleboard.getTab("Main").getLayout("Speed Caps", BuiltInLayouts.kList).add("Rotational", Constants.DriveConstants.kMaxAngularSpeed).withSize(1, 2).getEntry();
    
    InstantCommand zero = new InstantCommand(() -> this.zeroHeading(), this);
    zero.setName("Zero");
    zeroYaw = Shuffleboard.getTab("Main").add("Zero Heading" , zero).withWidget(BuiltInWidgets.kCommand).withSize(1, 1).withPosition(3, 0);

    translationalSpeedWidget = Shuffleboard.getTab("Main").getLayout("Speed", BuiltInLayouts.kList).withPosition(2, 0).withSize(1, 2).withProperties(Map.of("Label Position", "TOP")).add("Translational", 0).withSize(1, 2).getEntry();
    currentRotSpeedWidget = Shuffleboard.getTab("Main").getLayout("Speed", BuiltInLayouts.kList).add("Rotational", 0).getEntry();

    yawWidget = Shuffleboard.getTab("Main").getLayout("Attitude", BuiltInLayouts.kList).withPosition(0, 2 ) .withSize(1, 2).withProperties(Map.of("Label Position", "TOP")).add("Yaw", 0).getEntry();
    pitchWidget = Shuffleboard.getTab("Main").getLayout("Attitude", BuiltInLayouts.kList).add("Pitch", 0).getEntry();
    rollWidget = Shuffleboard.getTab("Main").getLayout("Attitude", BuiltInLayouts.kList).add("Roll", 0).getEntry();

    xPowerWidget = Shuffleboard.getTab("Main").getLayout("Throttle", BuiltInLayouts.kList).withPosition(0, 0).withSize(1, 2).withProperties(Map.of("Label Position", "LEFT")).add("X" , 0).getEntry();
    yPowerWidget = Shuffleboard.getTab("Main").getLayout("Throttle", BuiltInLayouts.kList).add("Y" , 0).getEntry();
    rotPowerWidget = Shuffleboard.getTab("Main").getLayout("Throttle", BuiltInLayouts.kList).add("Z" , 0).getEntry();

    Shuffleboard.getTab("Main").getLayout("Balance PID", BuiltInLayouts.kList).withPosition(8, 0).withProperties(Map.of("Label Position", "HIDDEN")).withSize(2, 2).add(BalanceCommand.gyroPID).withWidget(BuiltInWidgets.kPIDController);

  }

  

  
}
