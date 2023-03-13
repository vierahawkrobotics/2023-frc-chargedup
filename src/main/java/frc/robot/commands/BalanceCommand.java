package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.DriveSubsystem;

public class BalanceCommand extends CommandBase {
    public SubsystemBase base;
    public DriveSubsystem drive;
    public static boolean isFinished = false;

    public double adjust = 0;
    PIDController gyroPID = new PIDController(0.5, 0, 0);
    
    private static GenericEntry pitch;
    private static GenericEntry motorpower;
    
    public BalanceCommand(DriveSubsystem drive) {  
        this.drive = drive;
        addRequirements(drive);
    }
    
    @Override
    public void initialize() {
        pitch = Shuffleboard.getTab("Drive").add("Pitch", 0).withPosition(3, 3).getEntry();
        motorpower = Shuffleboard.getTab("Drive").add("Motor Power",0).withSize(3, 4).getEntry();
        System.out.println("Intialized");
    }

    @Override
    public void execute() {
        while(!isFinished()){
            adjust = gyroPID.calculate(-drive.m_gyro.getPitch(), 0);
            if (Math.abs(adjust) < 0.01) adjust = 0;
            drive.drive(adjust, 0, 0, false);
    
            pitch.setDouble(drive.m_gyro.getPitch());
            motorpower.setDouble(adjust);
            System.out.println("executing");

            if (Math.abs(drive.m_gyro.getPitch()) < 3){
                isFinished = true;
            }
        }
    }

    @Override
    public void end(boolean ending) {

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
