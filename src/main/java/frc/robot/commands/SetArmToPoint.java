package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;

public class SetArmToPoint extends CommandBase {
    public TelescopeSubsystem tele;
    public ArmSubsystem arm;
    public double x;
    public double y;
    public SetArmToPoint(double x, double y, TelescopeSubsystem tele, ArmSubsystem arm) {
        this.tele = tele;
        this.arm = arm;
        addRequirements(tele);
        addRequirements(arm);
        this.x = x;
        this.y = y - Constants.armHeight;
    }

    public SetArmToPoint(Supplier<Double> x, Supplier<Double> y, TelescopeSubsystem tele, ArmSubsystem arm) {
        this.tele = tele;
        this.arm = arm;
        addRequirements(tele);
        addRequirements(arm);
        this.x = x.get();
        this.y = y.get();
    }

    @Override
    public void initialize() {
        if(inRange(x,0.1) && inRange(y,0.1)) {
            arm.setTargetRadianT(Math.PI-Math.atan2(y, x));
            tele.setLength((Math.sqrt(x*x+y*y)-Constants.ArmALength));
        }
    }

    boolean inRange(double value, double range) {
        return value > -range && value < range;
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
