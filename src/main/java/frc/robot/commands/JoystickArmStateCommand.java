package frc.robot.commands;

import com.fasterxml.jackson.annotation.JsonTypeInfo.As;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

public class JoystickArmStateCommand extends CommandBase {
    public ArmSubsystem arm;
    public ClawSubsystem claw;
    public int state;

    static Constants.ArmStates state = Constants.ArmState.Low;

    public JoystickArmStateCommand(int state, ArmSubsystem arm, ClawSubsystem claw ) {
        this.arm = arm;
        this.claw = claw;
        this.state = state;
        addRequirements(this.arm);
        addRequirements(this.claw);
    }

    void ArmUp(){

        if(state == Constants.ArmState.High) {
            return;
        }
        
    }

    void ArmDown(){
       
        if (state == Constants.ArmState.Low) {
            return;
        }  

    }

    @Override
    public void initialize() {

        state = -1;
    }

    @Override
    public void execute() {
        










        if (state == Constants.ArmState.Low) {
            ArmUp();
        }  

        else if (state == -1) {
            ArmDown();
        }
    }

    private void ArmDown() {
    }

    @Override
    public void end(boolean ending) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
