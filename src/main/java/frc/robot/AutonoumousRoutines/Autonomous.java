package frc.robot.AutonoumousRoutines;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.AutonmousCommands.AutoSetArmCommandLow;
import frc.robot.AutonmousCommands.AutoSetArmCommandHigh;
import frc.robot.AutonmousCommands.AutoSetArmCommandLow;
import frc.robot.Constants.ClawStates;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.CollectCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;
import frc.robot.subsystems.motorClawSubsystem;

public class Autonomous {
    DriveSubsystem drive;
    ArmSubsystem armSubsystem;
    TelescopeSubsystem telescopeSubsystem;
    motorClawSubsystem clawSubsystem;

    public Autonomous(DriveSubsystem drive, ArmSubsystem armSubsystem, TelescopeSubsystem telescopeSubsystem,
            motorClawSubsystem clawSubsystem) {
        this.drive = drive;
        this.armSubsystem = armSubsystem;
        this.telescopeSubsystem = telescopeSubsystem;
        this.clawSubsystem = clawSubsystem;
    }

    public Command getAutoPath(PathPlannerTrajectory Path) {
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("RaiseArmToHigh", new AutoSetArmCommandHigh(Constants.ArmStates.High, armSubsystem, telescopeSubsystem));
        eventMap.put("LowerArmToGround", new AutoSetArmCommandLow(Constants.ArmStates.Ground, armSubsystem, telescopeSubsystem));
        eventMap.put("Collect", new CollectCommand(clawSubsystem));

        SwerveAutoBuilder swerveAutoBuilder = new SwerveAutoBuilder(
                drive::getPose,
                drive::resetOdometry,
                DriveConstants.kDriveKinematics,
                new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X
                                                 // and Y
                                                 // PID controllers)
                new PIDConstants(1, 0.0, 0.0),
                drive::setModuleStates,
                eventMap,
                true,
                drive);

        Command FullAuto = swerveAutoBuilder.fullAuto(Path);

        // Run path following command, then stop at the end.
        // return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0,
        // false));
        return FullAuto;
    }

}
