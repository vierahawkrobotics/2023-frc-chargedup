package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Collector extends SubsystemBase {

    CANSparkMax leftSuckingMotor = new CANSparkMotor()
    CANSparkMax rightSuckingMotor = new CANSparkMotor()
    

    public Collector() {
        setName("Rotation");
    }
    
    public void setSpeed(double speed){
        

    
    }
    

    @Override
    public void periodic() {

    }

    @Override
    public void simulationPeriodic() {

    }
}
