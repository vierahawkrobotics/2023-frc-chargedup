package frc.robot.commands;

//import java.net.SocketTimeoutException;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
//import edu.wpi.first.wpilibj2.command.Subsystem;
//import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ArmSubsystem;
//import frc.robot.Constants;
//import frc.robot.Robot;
//import frc.robot.RobotContainer;


public class SetArmStateCommand extends CommandBase {
    public ArmSubsystem arm_subsystem;
    public Constants.ArmStates m_state;
    public SetArmStateCommand(Constants.ArmStates state, ArmSubsystem ArmSubsystem) {
        this.arm_subsystem = ArmSubsystem;
        addRequirements(arm_subsystem);
        m_state = state;
    }

    // @Override
    // public void initialize() {
    //     arm_subsystem.targetRadian = ArmSubsystem.getHeightToRadians(m_height);
    // }

    @Override
    public void execute() {
        arm_subsystem.setHeight(m_state);
    }

    @Override
    public void end(boolean ending) {
        
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}