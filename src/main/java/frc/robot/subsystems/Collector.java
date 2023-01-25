package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Collector extends SubsystemBase {

    CANSparkMax leftSuckingMotor = new CANSparkMax(0, MotorType.kBrushless);
    CANSparkMax rightSuckingMotor = new CANSparkMax(1, MotorType.kBrushless);
    

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
