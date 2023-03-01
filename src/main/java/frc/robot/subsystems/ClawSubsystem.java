package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class ClawSubsystem extends SubsystemBase {
    DoubleSolenoid solenoid1 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);

    public ClawSubsystem() {
        setName("claw");
        solenoid1.set(Value.kReverse);
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