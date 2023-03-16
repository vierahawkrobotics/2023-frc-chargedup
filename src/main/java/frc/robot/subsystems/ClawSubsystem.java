package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClawStates;

import javax.management.InstanceAlreadyExistsException;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class ClawSubsystem extends SubsystemBase {
    DoubleSolenoid solenoid1 = new DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1);

    ClawStates clawState;
    GenericEntry clawStateEntry;    

    private static boolean isClawOpen = true;

    public ClawSubsystem() {
        setName("claw");
        solenoid1.set(Value.kReverse);
        clawState = ClawStates.Closed;
        clawStateEntry = Shuffleboard.getTab("Main").add("Claw State", "Closed").withPosition(1, 3) .getEntry();
    }

    public void OpenClaw(){
        solenoid1.set(Value.kForward);
        isClawOpen = true;
    }

    public void CloseClaw(){
        solenoid1.set(Value.kReverse);
        isClawOpen = false;
    }

    public void toggleClaw(){
        solenoid1.toggle();
        isClawOpen = !isClawOpen;
    }

    @Override
    public void periodic() {
        String updateMessage = isClawOpen ? "Open" : "Closed";
        clawStateEntry.setString(updateMessage);   
    }

    @Override
    public void simulationPeriodic() {

    }
    
}