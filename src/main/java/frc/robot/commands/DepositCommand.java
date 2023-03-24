package frc.robot.commands;

import java.lang.invoke.ConstantBootstraps;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.motorClawSubsystem;

public class DepositCommand extends CommandBase {
    public SubsystemBase base;
    motorClawSubsystem claw ;

    public DepositCommand(motorClawSubsystem claw) {
        this.claw = claw;
        addRequirements(claw);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        claw.setTargetSpeed(Constants.motorClawSubsystemConstants.depositSpeed);       
    }

    @Override
    public void end(boolean ending) {
        claw.setTargetSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
