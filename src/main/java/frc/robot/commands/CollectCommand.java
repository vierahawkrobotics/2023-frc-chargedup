package frc.robot.commands;

import edu.wpi.first.hal.simulation.ConstBufferCallback;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.motorClawSubsystem;

public class CollectCommand extends CommandBase {
    public SubsystemBase base;
    motorClawSubsystem claw ;

    public CollectCommand(motorClawSubsystem claw) {
        this.claw = claw;
        addRequirements(claw);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        claw.setTargetSpeed(Constants.motorClawSubsystemConstants.collectionSpeed);       
    }

    @Override
    public void end(boolean ending) {
        claw.setTargetSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return claw.isStalled;
    }
}
