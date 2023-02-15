package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.TelescopeSubsystem;

public class SetTelescopePosition extends CommandBase {
    public SubsystemBase base;
    public TelescopeSubsystem telescope_subsystem;
    public double distance;
    public SetTelescopePosition(double m_distance, TelescopeSubsystem t_subsystem) {
        this.telescope_subsystem = t_subsystem;
        addRequirements(t_subsystem);
        m_distance = distance;
    }

    @Override
    public void initialize() {
        double setpoint = telescope_subsystem.getLengthToRadians(distance);
        telescope_subsystem.targetLength = setpoint;
    }

    @Override
    public void execute() {
        double setpoint = telescope_subsystem.getLengthToRadians(distance);
        telescope_subsystem.targetLength = setpoint;
    }

    @Override
    public void end(boolean ending) {
        
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
