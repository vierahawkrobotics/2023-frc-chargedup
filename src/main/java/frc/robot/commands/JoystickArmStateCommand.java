package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.TelescopeSubystem;

public class JoystickArmStateCommand extends CommandBase {
    public ArmSubsystem arm;
    public ClawSubsystem claw;
    public int state;

    static Constants.ArmStates state = new Constants.ArmState.Low;

    public JoystickArmStateCommand(int state, ArmSubsystem arm, ClawSubsystem claw ) {
        this.arm = Artem;
        this.claw = claw;
        this.state = state;
        addrequirements(this.arm);
        addrequirements(this.claw);
    }

    
    void armUp(){

        if(state == Constants.ArmState.High) {
            return;
        }
        
    }

    void armUp(){
       
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
