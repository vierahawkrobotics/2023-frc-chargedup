package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;

public class SetArmToPointState extends CommandBase {
    public TelescopeSubsystem tele;
    public ArmSubsystem arm;
    public double x;
    public double y;
    public SetArmToPointState(double x, Constants.ArmStates state, TelescopeSubsystem tele, ArmSubsystem arm) {
        this.tele = tele;
        this.arm = arm;
        addRequirements(tele);
        addRequirements(arm);
        this.x = x;
        setYtoState(state);
    }

    void setYtoState(Constants.ArmStates state) {
        switch(state) {
            case Middle:
                y = Constants.midHeight;
                break;
            case High:
                y = Constants.highHeight;
                break;
            default:
                y =0;
                break;
        }
    }

    @Override
    public void initialize() {
        tele.setLength((Math.sqrt(x*x+y*y)-Constants.ArmALength));
        arm.setTargetRadianT(Math.PI-Math.atan2(y, x));
    }

    @Override
    public void execute() {
        tele.setLength((Math.sqrt(x*x+y*y)-Constants.ArmALength));
        arm.setTargetRadianT(Math.PI-Math.atan2(y, x));
    }

    @Override
    public void end(boolean ending) {
        
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
