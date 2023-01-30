package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SetArmPosCommand extends CommandBase {
    public SubsystemBase base;

    public SetArmPosCommand() {
        
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double rotateSpeed = RobotContaner.driverController.getRawAxis(Constants.DRIVER_CONTROLLER_ROTATE_AXIS);
    }

    @Override
    public void end(boolean ending) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
