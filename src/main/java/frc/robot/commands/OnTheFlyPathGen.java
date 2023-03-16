package frc.robot.commands;

import java.nio.file.Path;
import java.util.ArrayList;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Lemonlight;

public class OnTheFlyPathGen extends CommandBase {

    private DriveSubsystem drive;
    private Lemonlight lemonlight;
    private PathPoint currentPathPoint;
    private PathPoint chosePathPoint;

    FieldConstants fieldConstants = new FieldConstants();
    AutoConstants autoConstants = new AutoConstants();


    public OnTheFlyPathGen(DriveSubsystem drive, Lemonlight lemonlight, PathPoint pathPoint) {
        this.drive = drive;
        this.lemonlight = lemonlight;
        this.chosePathPoint = pathPoint;
        //this.aprilTagID = lemonlight.getAprilTagID;
        addRequirements(drive);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        drive.drive(0, 0, 0, true);
       // currentPathPoint = new PathPoint(lemonlight.getTranslation2D(), new Rotation2d(drive.getHeading()), lemonlight.getRobotPose().getTranslation().getZ());

    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        //drive.pathGeneration(currentPathPoint, chosePathPoint);
        
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drive.drive(0, 0, 0, true);

    }

    // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
