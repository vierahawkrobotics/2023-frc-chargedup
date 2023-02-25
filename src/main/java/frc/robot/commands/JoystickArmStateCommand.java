package frc.robot.commands;



import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;

public class JoystickArmStateCommand extends CommandBase {
    public ArmSubsystem arm;
    public TelescopeSubsystem claw;
    public int state;

    static Constants.ArmStates currentState = Constants.ArmStates.Low;

    public JoystickArmStateCommand(int state, ArmSubsystem arm, TelescopeSubsystem claw ) {
        this.arm = arm;
        this.claw = claw;
        this.state = state;
        addRequirements(this.arm);
        addRequirements(this.claw);
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
            case Ground:
                currentState = Constants.ArmStates.Low;
                return;
        }      
    }
    void armDown(){
       
        switch(currentState) {
            case Ground:
                return;
            case Middle:
                currentState = Constants.ArmStates.Low;
                return;
            case Low:
                currentState = Constants.ArmStates.Ground;
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

        arm.setHeight(currentState);
        claw.setLength(currentState);
    }

    @Override
    public void end(boolean ending) {

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
