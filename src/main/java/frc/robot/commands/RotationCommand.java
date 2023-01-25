package frc.robot.commands;

import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RotationCommand extends CommandBase {
    public Collector base;
    double speed;

    public RotationCommand(Collector collector, double speed) {
        addRequirements(collector);
        base = collector;
        this.speed = speed;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        
        base.setSpeed(speed);
    }

    @Override
    public void end(boolean ending) {

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
