package frc.robot.commands;

import javax.naming.spi.ObjectFactory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.DriveSubsystem;

public class BalanceCommand extends CommandBase {
    public SubsystemBase base;
    public DriveSubsystem drive;
    public static boolean isFinished = false;

    public double adjust = 0;
    public static PIDController gyroPID = new PIDController(0.006, 0.001, 0.001);

    public BalanceCommand(DriveSubsystem drive) {
        this.drive = drive;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        adjust = gyroPID.calculate(drive.m_gyro.getRoll(), 0);
        if (Math.abs(drive.m_gyro.getRoll()) < 2)
            adjust = 0;
        drive.setX();
        drive.drive(adjust, 0, 0, true);
    }

    @Override
    public void end(boolean ending) {
        System.out.println("Finsihed");

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
