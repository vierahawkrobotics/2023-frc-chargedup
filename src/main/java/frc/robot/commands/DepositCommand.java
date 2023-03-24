package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.motorClawSubsystem;

public class DepositCommand extends CommandBase {
    public SubsystemBase base;

    public motorClawSubsystem claw;

    long start;

    public DepositCommand(motorClawSubsystem claw) {
        this.claw = claw;
        addRequirements(claw);
    }

    @Override
    public void initialize() {
        start = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        claw.updatePID(false, true, false, false);      
    }

    @Override
    public void end(boolean ending) {
    }

    @Override
    public boolean isFinished() {
        if (System.currentTimeMillis() - start > 1000){
            return true;
        }
        return false;
    }
}
