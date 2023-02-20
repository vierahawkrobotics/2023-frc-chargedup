package frc.robot.commands;

//import java.net.SocketTimeoutException;

import edu.wpi.first.wpilibj2.command.CommandBase;
//import edu.wpi.first.wpilibj2.command.Subsystem;
//import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ArmSubsystem;
//import frc.robot.Constants;
//import frc.robot.Robot;
//import frc.robot.RobotContainer;


public class SetArmPosCommand extends CommandBase {
    public ArmSubsystem arm_subsystem;
    public double m_height;
    public SetArmPosCommand(double height, ArmSubsystem ArmSubsystem) {
        this.arm_subsystem = ArmSubsystem;
        addRequirements(arm_subsystem);
        m_height = height;
    }

    // @Override
    // public void initialize() {
    //     arm_subsystem.targetRadian = ArmSubsystem.getHeightToRadians(m_height);
    // }

    @Override
    public void execute() {
        arm_subsystem.setHeight(m_height);
    }

    @Override
    public void end(boolean ending) {
        
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}