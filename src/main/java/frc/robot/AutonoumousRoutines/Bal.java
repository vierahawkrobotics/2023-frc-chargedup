package frc.robot.AutonoumousRoutines;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.kinematics.MecanumDriveMotorVoltages;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.BalanceCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;
import frc.robot.subsystems.motorClawSubsystem;

public class Bal extends SequentialCommandGroup {
    PathPlannerTrajectory Bal = PathPlanner.loadPath("Bal", new PathConstraints(2, 1));
    DriveSubsystem drive;
    ArmSubsystem armSubsystem;
    TelescopeSubsystem telescopeSubsystem;
    motorClawSubsystem clawSubsystem;
    Autonomous auto;
    
    public Bal(DriveSubsystem drive, ArmSubsystem armSub, TelescopeSubsystem telescopeSubsystem, motorClawSubsystem clawSubsystem) {
        this.drive = drive;
        auto = new Autonomous(drive, armSubsystem, telescopeSubsystem, clawSubsystem);
        
        addCommands(
            auto.getAutoPath(Bal),
            new BalanceCommand(drive)           
        );
    }
    
}
