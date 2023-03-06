package frc.robot.commands;

//import java.net.SocketTimeoutException;

import edu.wpi.first.wpilibj2.command.CommandBase;
//import edu.wpi.first.wpilibj2.command.Subsystem;
//import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ArmSubsystem;
//import frc.robot.Constants;
//import frc.robot.Robot;
//import frc.robot.RobotContainer;
import java.util.function.Supplier;


public class SetArmPosCommand extends CommandBase {
    public ArmSubsystem arm_subsystem;
    public Supplier<Double> getRadians;
    public SetArmPosCommand(Supplier<Double> getRadians, ArmSubsystem ArmSubsystem) {
        this.arm_subsystem = ArmSubsystem;
        addRequirements(arm_subsystem);
        this.getRadians = getRadians;
    }

     @Override
     public void initialize() {
        arm_subsystem.setTargetRadianT(getRadians.get());
        //System.out.println(getRadians.get());
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean ending) {
        
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}