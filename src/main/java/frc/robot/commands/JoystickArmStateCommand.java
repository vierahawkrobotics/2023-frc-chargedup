package frc.robot.commands;

import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;

public class JoystickArmStateCommand extends CommandBase {
    public ArmSubsystem arm;
    public TelescopeSubsystem telescope;
    public int state;

    public static Constants.ArmStates currentState = Constants.ArmStates.Ground;

    public JoystickArmStateCommand(int state, ArmSubsystem arm, TelescopeSubsystem telescope ) {
        this.arm = arm;
        this.telescope = telescope;
        this.state = state;
        addRequirements(this.arm);
        addRequirements(this.telescope);
    }

    void armUp(){

        switch(currentState) {
            case High:
                return;
            case Middle:
                currentState = Constants.ArmStates.High;
                return;
            case Low:
                currentState = Constants.ArmStates.Middle;
                return;
            case Collect:
                currentState = Constants.ArmStates.Low;
                return;
            case Ground:
                currentState = Constants.ArmStates.Collect;
                return;
        }      
    }
    void armDown(){
       
        switch(currentState) {
            case Ground:
                return;
            case Collect:
                currentState = Constants.ArmStates.Ground;
                return;
            case Low:
                currentState = Constants.ArmStates.Collect;
                return;
            case Middle:
                currentState = Constants.ArmStates.Low;
                return;
            case High:
                currentState = Constants.ArmStates.Middle;
                return;
        }
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        if (state == 1) {
            armUp();
        }  

        else if (state == -1) {
            armDown();
        }
        
        arm.setRadian(currentState);
        telescope.setLength(currentState);
    }

    @Override
    public void end(boolean ending) {

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
