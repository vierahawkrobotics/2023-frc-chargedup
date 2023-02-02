package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import frc.robot.Constants;

 // Left and Right movement pull/release back a string attached to a coil
 // mOTOR SET POSITION
 // CONVERT ENCODER POSITIONS TO RADIANS, VICE VERSA
public class TelescopeSubsystem {
   
    final CANSparkMax motor = new CANSparkMax(Constants.armMotorID, MotorType.kBrushless);
    final SparkMaxAbsoluteEncoder encoder = motor.getAbsoluteEncoder(Type.kDutyCycle);
 
    public double getLengthToRadians(double length){
        return (2 * Math.PI * encoder.getPosition());
    }
    
    public double getRadiansToLength(double radians){
        return Constants.ArmALength-Math.cos(radians) * Constants.ArmBLength;
    }
    public void setSpeed(double speed){
        motor.set(speed);
    }
    public double getPosition(){
        return getPosition();
    }




    
}

