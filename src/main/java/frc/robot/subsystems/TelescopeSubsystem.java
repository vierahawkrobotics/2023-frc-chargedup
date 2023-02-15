package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class TelescopeSubsystem extends SubsystemBase {
   
    
    /** Create Variables: Motor, Encoder, PID */
    final CANSparkMax motor = new CANSparkMax(Constants.armMotorID, MotorType.kBrushless);
    final SparkMaxAbsoluteEncoder encoder = motor.getAbsoluteEncoder(Type.kDutyCycle);
    final PIDController pid = new PIDController(Constants.ScopeP,Constants.ScopeI,Constants.ScopeD);
    
    public double targetLength = 0;

    public TelescopeSubsystem() {
        setName("name");
    }

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
    
    public void setHeight(Constants.ArmStates mState) {
        double target = 0;
        switch (mState) {
            case Low:
                target = getLengthToRadians(Constants.lowGoalTeleLength);
                break;
            case Middle:
                target = getLengthToRadians(Constants.middleGoalTeleLength);
                break;
            case High:
                target = getLengthToRadians(Constants.highGoalTeleLength);
                break;
        }
        targetLength = target;
    }

    @Override
    public void periodic() {
        setpid(getLengthToRadians(targetLength));
    }

    @Override
    public void simulationPeriodic() {

    }
}

