package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Subsystem extends SubsystemBase {

    NEO suckingMotor = new NEO()

    public Subsystem() {
        setName("name");
    }
    
    public void setMotorSpeed(double speed){
        suckingMotor.set(ControlMode.PercentOutput, speed)
    
    
    }
    

    @Override
    public void periodic() {

    }

    @Override
    public void simulationPeriodic() {

    }
}
