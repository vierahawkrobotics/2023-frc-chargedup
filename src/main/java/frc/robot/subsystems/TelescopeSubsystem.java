package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import edu.wpi.first.math.controller.PIDController;

import frc.robot.Constants;


public class TelescopeSubsystem {
   
    
    /** Create Variables: Motor, Encoder, PID */
    final CANSparkMax motor = new CANSparkMax(Constants.armMotorID, MotorType.kBrushless);
    final SparkMaxAbsoluteEncoder encoder = motor.getAbsoluteEncoder(Type.kDutyCycle);
    final PIDController pid = new PIDController(Constants.ScopeP,Constants.ScopeI,Constants.ScopeD);
    
    /** Using Radius, Convert Length To Radians (length/radius = radians) */
    public double getLengthToRadians(double length){
        return (length / Constants.ArmWinchRadius);
    }
    /** Convert Radians Back To Length (radians * radius = length) */
    public double getRadiansToLength(double radians){
        return (radians * Constants.ArmWinchRadius);
    }
    /** Convert Encoder Position Into Radians (encoder_position * 2 * pi) */
    public double getEncoderInRadians() {
        return encoder.getPosition() * 2 * Math.PI;
    }
    /** The Equation Uses "getEncoderInRadians" As The Input For The Equation "getRadiansToLength" */
    public double getCurrentArmLength(){
        return getRadiansToLength(getEncoderInRadians());
    }
    /** Using A PID, Calculate The Encoder Position And Create A Certain Setpoint */
    public void setpid (double setpoint){
        motor.set(pid.calculate(encoder.getPosition(), setpoint));
    }
    
}

