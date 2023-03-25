package frc.robot;

import java.util.ArrayList;

import com.pathplanner.lib.PathPoint;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;

public class Constants {

  public static final int CANdleID = 1;

  public final static double midHeight = .87;
  public final static double highHeight = 1.17;

  public enum ArmStates {
    Ground,
    Collect,
    Low,
    Middle,
    High;
  }

  public enum ClawStates {
    Open,
    Closed,
    Toggle;
  }

  public final static double clawLength = 0.305;
  public final static double armHeight = 1.05;

  public static final class TelescopeArmConstants {
    public final static double ScopeP = 4;
    public final static double ScopeI = 0.00;
    public final static double ScopeD = 0;

    public final static int scopeMotorID = 10;
    public final static double ArmBLength = 0.45;
    public final static double ArmWinchRadius = 0.00118443982 * 1.11111111111 * 1.21348314607; // radius of gear
                                                                                               // 4.5466cm / gear ration
                                                                                               // 27

    public final static double groundGoalTeleLength = 0.015 * ArmBLength;
    public final static double lowGoalTeleLength = Units.inchesToMeters(4);;
    public final static double collectGoalTeleLength = Units.inchesToMeters(7);
    public final static double middleGoalTeleLength = .60 * ArmBLength;
    public final static double highGoalTeleLength = 1 * ArmBLength;
  }

  public static final class RotationArmConstants {
    public final static double ArmP = 1.4;
    public final static double ArmI = 0.00;
    public final static double ArmD = 0.001;

    public final static int armMotorID = 9;
    public final static double ArmALength = 0.6477;

    public final static double lowGoalRadian = 0.22;
    public final static double middleGoalRadian = 1.58;
    public final static double highGoalRadian = 1.95;
    public final static double groundGoalRadian = 0.0;

    public final static double angleAdjustmentRange = Math.toRadians(10);
  }

  public static class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static double kMaxSpeedMetersPerSecond = 4;
    public static double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(21.625);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(21.625);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 2;
    public static final int kRearLeftDrivingCanId = 6;
    public static final int kFrontRightDrivingCanId = 4;
    public static final int kRearRightDrivingCanId = 8;

    public static final int kFrontLeftTurningCanId = 1;
    public static final int kRearLeftTurningCanId = 5;
    public static final int kFrontRightTurningCanId = 3;
    public static final int kRearRightTurningCanId = 7;

    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth
    // will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Invert the turning encoder, since the output shaft rotates in the opposite
    // direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0.0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class OIConstants {
    public static final int DriverControllerPort = 0;
    public static final int OperatorControllerPort = 1;
  }

  public static final class LemonlightConstants {
    public static final double aprilTagThreshold = .2;
  }

  public static final class CandleConstants {
    public static final int CANdleID = 1;
    public static final int JoystickId = 0;
    public static final int IncrementAnimButton = XboxController.Button.kRightBumper.value;
    public static final int DecrementAnimButton = XboxController.Button.kLeftBumper.value;
    public static final int BlockButton = XboxController.Button.kStart.value;
    public static final int MaxBrightnessAngle = 90;
    public static final int MidBrightnessAngle = 180;
    public static final int ZeroBrightnessAngle = 270;
    public static final int VbatButton = XboxController.Button.kA.value;
    public static final int V5Button = XboxController.Button.kB.value;
    public static final int CurrentButton = XboxController.Button.kX.value;
    public static final int TemperatureButton = XboxController.Button.kY.value;
  }

  public static final class motorClawSubsystemConstants{
    public static final int clawMotorID = 13;
    public static double clawP = 0;
    public static double clawI = 0;
    public static double clawD = 0;

    public static double collectionSpeed = 100;
    public static double depositSpeed = -100;

  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 4.5;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;

    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class FieldConstants {
    private final double translationXCoor = 1.97;
    private final double initialPose = .1;
    private ArrayList<PathPoint> topAprilTagPoseList;
    private ArrayList<PathPoint> middleAprilTagPoseList;
    private ArrayList<PathPoint> bottomAprilTagPoseList;

    public PathPoint aprilTagPathPoint1 = new PathPoint(new Translation2d(translationXCoor + initialPose, 1.05),
        new Rotation2d().fromDegrees(180), new Rotation2d(0).fromDegrees(-180));
    public PathPoint aprilTagPathPoint2 = new PathPoint(new Translation2d(translationXCoor + initialPose, 2.75),
        new Rotation2d().fromDegrees(180), new Rotation2d(0).fromDegrees(-180));
    public PathPoint aprilTagPathPoint3 = new PathPoint(new Translation2d(translationXCoor + initialPose, 4.40),
        new Rotation2d().fromDegrees(180), new Rotation2d(0).fromDegrees(-180));

    // 1
    PathPoint firstPose = new PathPoint(new Translation2d(translationXCoor + initialPose, 3.85),
        new Rotation2d().fromDegrees(0), new Rotation2d().fromDegrees(180));
    // 2
    PathPoint secondPose = new PathPoint(new Translation2d(translationXCoor + initialPose, 3.85),
        new Rotation2d().fromDegrees(0), new Rotation2d().fromDegrees(180));
    // 3
    PathPoint thirdPose = new PathPoint(new Translation2d(translationXCoor + initialPose, 3.29),
        new Rotation2d().fromDegrees(0), new Rotation2d().fromDegrees(180));
    // 4
    PathPoint fourthPose = new PathPoint(new Translation2d(translationXCoor + initialPose, 2.19),
        new Rotation2d().fromDegrees(0), new Rotation2d().fromDegrees(180));
    // 5
    PathPoint fifthPose = new PathPoint(new Translation2d(translationXCoor + initialPose, 1.6),
        new Rotation2d().fromDegrees(0), new Rotation2d().fromDegrees(180));
    // 6
    PathPoint sixthPose = new PathPoint(new Translation2d(translationXCoor + initialPose, .51),
        new Rotation2d().fromDegrees(0), new Rotation2d().fromDegrees(180));

    public ArrayList<PathPoint> getUpperPathPoints() {
      topAprilTagPoseList.add(firstPose);
      topAprilTagPoseList.add(aprilTagPathPoint1);
      topAprilTagPoseList.add(secondPose);

      return topAprilTagPoseList;
    }

    public ArrayList<PathPoint> getMiddPoints() {
      middleAprilTagPoseList.add(thirdPose);
      middleAprilTagPoseList.add(aprilTagPathPoint2);
      middleAprilTagPoseList.add(fourthPose);

      return topAprilTagPoseList;
    }

    public ArrayList<PathPoint> getBottomPoints() {
      bottomAprilTagPoseList.add(fifthPose);
      bottomAprilTagPoseList.add(aprilTagPathPoint2);
      bottomAprilTagPoseList.add(sixthPose);

      return topAprilTagPoseList;
    }
  }
}

// ~37 distance between fully extended arm touching ground and robot front
// 64.77 cm arm A length
// 71.12 cm - 10in. arm B length
// 12 in. claw pads to armB
// Claw <2lb
// Whole arm ~15 lb