package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import java.lang.Math;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
    //function armhight to radians
    //function radians to armhight
    
    //sparkmax motor + absolute encode
        //Set speed
        //Encoder Position
//------------------------------------
    //pidcontroller
    //function to set target position

    //command to set target position
    //PIDController pid = new PIDController(kP, kI, kD);
    //state machine for different hights

    //motor.set(pidController.calculate(currentRotation, target));

    final CANSparkMax motor = new CANSparkMax(Constants.armMotorID, MotorType.kBrushless);
    final SparkMaxAbsoluteEncoder encoder = motor.getAbsoluteEncoder(Type.kDutyCycle);

    public double getHeightToRadians(double height){
        return Math.acos((height-Constants.ArmALength)/Constants.ArmBLength);
    }
    
    public double getRadiansToHeight(double radians){
        return Constants.ArmALength -Math.cos(radians) * Constants.ArmBLength;
    }
    public void setSpeed(double speed){
        motor.set(speed);
    }
    public double getPosition(){
        return getPosition();
    }

    public ArmSubsystem() {
        setName("name");
    }

    @Override
    public void periodic() {

    }

    @Override
    public void simulationPeriodic() {

    }
}
