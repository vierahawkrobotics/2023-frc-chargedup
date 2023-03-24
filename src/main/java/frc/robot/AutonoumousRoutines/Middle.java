package frc.robot.AutonoumousRoutines;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ArmStates;
import frc.robot.commands.BalanceCommand;
import frc.robot.commands.SetArmStateCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;
import frc.robot.subsystems.motorClawSubsystem;

public class Middle extends SequentialCommandGroup {
    PathPlannerTrajectory Middle;
    DriveSubsystem drive;
    ArmSubsystem armSubsystem;
    TelescopeSubsystem telescopeSubsystem;
    motorClawSubsystem clawSubsystem;
    Autonomous auto;

    public Middle(DriveSubsystem drive, ArmSubsystem armSubsystem, TelescopeSubsystem telescopeSubsystem,motorClawSubsystem clawSubsystem) {
        this.drive = drive;
        this.armSubsystem = armSubsystem;
        this.telescopeSubsystem = telescopeSubsystem;
        this.clawSubsystem = clawSubsystem;
        auto = new Autonomous(drive, armSubsystem, telescopeSubsystem, clawSubsystem);

        PathPoint startPoint = new PathPoint(new Translation2d(2.55, 3.23), new Rotation2d(0), new Rotation2d(180));
        PathPoint firstWayPathPoint = new PathPoint(new Translation2d(1.75, 3.23), new Rotation2d(0),
                new Rotation2d(180));
        PathPoint secondWayPathPoint = new PathPoint(new Translation2d(2.36, 3.23), new Rotation2d(0),
                new Rotation2d(180));
        PathPoint endPoint = new PathPoint(new Translation2d(3.9, 3.23), new Rotation2d(0), new Rotation2d(180));

        PathPlannerTrajectory firstPath = PathPlanner.generatePath(
                new PathConstraints(2, 1),
                startPoint,
                firstWayPathPoint);

        PathPlannerTrajectory secondPath = PathPlanner.generatePath(
                new PathConstraints(2, 1),
                firstWayPathPoint,
                secondWayPathPoint);

        PathPlannerTrajectory thirdPath = PathPlanner.generatePath(
                new PathConstraints(2, 1),
                secondWayPathPoint,
                endPoint);

        addCommands(
                new SetArmStateCommand(ArmStates.High, armSubsystem, telescopeSubsystem),
                new WaitCommand(.2),
                auto.getAutoPath(firstPath),
                // new SetClawCommand(ClawStates.Toggle, clawSubsystem),
                new WaitCommand(.2),
                auto.getAutoPath(secondPath),
                new SetArmStateCommand(ArmStates.Ground, armSubsystem, telescopeSubsystem),
                new WaitCommand(.5),
                auto.getAutoPath(thirdPath),
                new BalanceCommand(drive)
                
        );
    }

}
