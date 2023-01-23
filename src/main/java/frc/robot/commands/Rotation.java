package frc.robot.commands;

import java.util.stream.Collector;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Rotation extends CommandBase {
    public SubsystemBase base;

    public Rotation() {
        requires(Collector);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        
        Robot.driveTrain.setMotorSpeed()
    }

    @Override
    public void end(boolean ending) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
