package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TelescopeSubsystem;

public class SetTelescopePosition extends CommandBase {
    public TelescopeSubsystem telescope_subsystem;
    public double distance;
    public SetTelescopePosition(double m_distance, TelescopeSubsystem t_subsystem) {
        this.telescope_subsystem = t_subsystem;
        addRequirements(t_subsystem);
        distance = m_distance;
    }

    @Override
    public void initialize() {
        telescope_subsystem.setLength(distance);
    }

    @Override
    public void execute() {
        telescope_subsystem.setLength(distance);
    }

    @Override
    public void end(boolean ending) {
        
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
