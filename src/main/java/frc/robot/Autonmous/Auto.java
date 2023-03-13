package frc.robot.Autonmous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Lemonlight;

public class Auto extends SequentialCommandGroup {
    DriveSubsystem drive;
    Lemonlight lemonlight;
    ArmSubsystem armSubsystem;
    ClawSubsystem clawSubsystem;

    

    public Auto (DriveSubsystem drive, Lemonlight lemonlight, ArmSubsystem armSubsystem, ClawSubsystem clawSubsystem) {
        this.drive = drive;
        this.lemonlight = lemonlight;
        this.armSubsystem = armSubsystem;
        this.clawSubsystem = clawSubsystem;
        
        addCommands(



        );
    }
    
}
