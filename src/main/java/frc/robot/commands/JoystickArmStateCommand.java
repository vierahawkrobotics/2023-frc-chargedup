package frc.robot.commands;



import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

public class JoystickArmStateCommand extends CommandBase {
    public ArmSubsystem arm;
    public Subsystem claw;
    public int state;

    static Constants.ArmStates currentState = Constants.ArmStates.Low;

    public JoystickArmStateCommand(int state, ArmSubsystem arm, Subsystem claw ) {
        this.arm = arm;
        this.claw = claw;
        this.state = state;
        addRequirements(this.arm);
        addRequirements(this.claw);
    }

    void armUp(){

        if(currentState == Constants.ArmStates.High) {
            return;
        }
        
    }

    void armDown(){
       
        if (currentState == Constants.ArmStates.Low) {
            return;
        }  

    }

    @Override
    public void initialize() {

        state = -1;
    }

    @Override
    public void execute() {
        if (state == 1) {
            armUp();
        }  

        else if (state == -1) {
            armDown();
        }
    }

    @Override
    public void end(boolean ending) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
