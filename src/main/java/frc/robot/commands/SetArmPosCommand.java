package frc.robot.commands;

import java.net.SocketTimeoutException;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;



public class SetArmPosCommand extends CommandBase {
    public ArmSubsystem arm_subsystem;
    public double m_height;
    public SetArmPosCommand(double height) {
        addRequirements(arm_subsystem);
       m_height = height;
    }

    @Override
    public void initialize() {


    }

    @Override
    public void execute() {
        
        double radians = ArmSubsystem.getHeightToRadians(m_height);
        ArmSubsystem.setTargetRotation(radians);

    }

    @Override
    public void end(boolean ending) {
        ArmSubsystem.motor.set(0.0);
    }

    @Override
    public boolean isFinished() {
        
    }
}