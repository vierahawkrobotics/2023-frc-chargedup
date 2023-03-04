package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawStates;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class ClawSubsystem extends SubsystemBase {
    DoubleSolenoid solenoid1 = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);

    ClawStates clawState;

    public ClawSubsystem() {
        setName("claw");
        solenoid1.set(Value.kReverse);
        clawState = ClawStates.Closed;

        ShuffleboardTab tab = Shuffleboard.getTab("Claw");
        //tab.addNumber(getName(), null)
        tab.addString("State", () -> {return clawState.toString();});
        
    }

    public void OpenClaw(){
        solenoid1.set(Value.kForward);
    }

    public void CloseClaw(){
        solenoid1.set(Value.kReverse);
    }

    public void toggleClaw(){
        solenoid1.toggle();
    }

    @Override
    public void periodic() {

    }

    @Override
    public void simulationPeriodic() {

    }
    
}