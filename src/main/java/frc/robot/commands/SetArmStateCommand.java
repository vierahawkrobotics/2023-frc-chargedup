package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
//import edu.wpi.first.wpilibj2.command.Subsystem;
//import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ArmSubsystem;
//import frc.robot.Constants;
//import frc.robot.Robot;
//import frc.robot.RobotContainer;
import frc.robot.subsystems.TelescopeSubsystem;

import frc.robot.subsystems.ArmSubsystem;

public class SetArmStateCommand extends CommandBase {
    public ArmSubsystem arm_subsystem;
    public TelescopeSubsystem tSubsystem;
    public Constants.ArmStates m_state;
    public SetArmStateCommand(Constants.ArmStates state, ArmSubsystem ArmSubsystem, TelescopeSubsystem tSubsystem) {
        this.arm_subsystem = ArmSubsystem;
        this.tSubsystem = tSubsystem;
        addRequirements(arm_subsystem);
        addRequirements(tSubsystem);
        m_state = state;
    }

    @Override
    public void initialize() {
        arm_subsystem.setHeight(m_state);
        tSubsystem.setHeight(m_state);
    }

    @Override
    public void execute() {
        arm_subsystem.setHeight(m_state);
        tSubsystem.setHeight(m_state);
    }

    @Override
    public void end(boolean ending) {
        
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}