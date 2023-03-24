package frc.robot.AutonmousCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;

public class AutoSetArmCommandHigh extends CommandBase {
    ArmSubsystem armSub;
    TelescopeSubsystem teleSub;
    Constants.ArmStates m_state;
    boolean complete;
    
    public AutoSetArmCommandHigh(Constants.ArmStates state, ArmSubsystem ArmSubsystem, TelescopeSubsystem tSubsystem) {
        this.armSub = ArmSubsystem;
        this.teleSub = tSubsystem;
        this.m_state = state;
        complete = false;

        addRequirements(armSub);
        addRequirements(tSubsystem);
    }

    @Override
    public void initialize() {
        armSub.setRadian(m_state);
        teleSub.setLength(m_state);
    }

    @Override
    public void execute() {
        if((m_state == Constants.ArmStates.High) 
        && (armSub.getPosition() >= Constants.RotationArmConstants.highGoalRadian - .15))
        {
            complete = true;
        }
    }

    @Override
    public void end(boolean ending) {
        
    }

    @Override
    public boolean isFinished() {
        return complete;
    }
}
