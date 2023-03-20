package frc.robot.AutonmousCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;

public class AutoSetArmCommand extends CommandBase {
    ArmSubsystem armSub;
    TelescopeSubsystem teleSub;
    Constants.ArmStates m_state;
    boolean complete = false;
    
    public AutoSetArmCommand(Constants.ArmStates state, ArmSubsystem ArmSubsystem, TelescopeSubsystem tSubsystem) {
        this.armSub = ArmSubsystem;
        this.teleSub = tSubsystem;
        this.m_state = state;

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
        && (armSub.getPosition() == Constants.RotationArmConstants.highGoalRadian) 
        && (teleSub.getCurrentArmLength() == Constants.TelescopeArmConstants.highGoalTeleLength))
        {
            complete = true;
        }

        if((m_state == Constants.ArmStates.Middle) 
        && (armSub.getPosition() == Constants.RotationArmConstants.middleGoalRadian) 
        && (teleSub.getCurrentArmLength() == Constants.TelescopeArmConstants.middleGoalTeleLength))
        {
            complete = true;
        }

        if((m_state == Constants.ArmStates.Ground) 
        && (armSub.getPosition() == Constants.RotationArmConstants.groundGoalRadian) 
        && (teleSub.getCurrentArmLength() == Constants.TelescopeArmConstants.groundGoalTeleLength))
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
