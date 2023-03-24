package frc.robot.AutonoumousRoutines;

import javax.sound.midi.Sequence;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;

public class SideAutoRoutine extends SequentialCommandGroup {

    DriveSubsystem drive;
    ArmSubsystem armSub;
    TelescopeSubsystem teleSub;
    ClawSubsystem clawSub;
    Autonomous auto;

    public SideAutoRoutine(DriveSubsystem drive, ArmSubsystem armSub, TelescopeSubsystem telescopeSub, ClawSubsystem clawSubsystem){
        this.drive = drive;
        this.armSub = armSub;
        this.teleSub = telescopeSub;
        this.clawSub = clawSubsystem;
        auto = new Autonomous(drive, armSub, telescopeSub, clawSubsystem);

        PathPlannerTrajectory Sides = PathPlanner.loadPath("Sides", new PathConstraints(.5, .5));

        addCommands(
            auto.getAutoPath(Sides)           
        );
    }


    
}
