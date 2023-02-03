package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.RobotContainer;

public class SetArmPosCommand extends CommandBase {
    public SubsystemBase base;

    public SetArmPosCommand() {
        requires(Robot.ArmSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double rotateSpeed = RobotContainer.driverController.getRawAxis(Constants.DRIVER_CONTROLLER_ROTATE_AXIS);
        motor.set()
    }

    @Override
    public void end(boolean ending) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
