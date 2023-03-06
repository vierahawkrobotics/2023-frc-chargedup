package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TelescopeSubsystem extends SubsystemBase {
   
    
    /** Create Variables: Motor, Encoder, PID */
    final CANSparkMax motor = new CANSparkMax(Constants.scopeMotorID, MotorType.kBrushless);
    final RelativeEncoder encoder = motor.getEncoder();
    final PIDController pid = new PIDController(Constants.ScopeP,Constants.ScopeI,Constants.ScopeD);
    
    static double targetLength = 0;

    public TelescopeSubsystem() {
        setName("name");
        encoder.setPosition(0);
        ShuffleboardTab tab = Shuffleboard.getTab("Telescope Arm");
        //tab.addNumber(getName(), null)
        tab.addNumber("Motor position:", () -> {return encoder.getPosition();});
        tab.addNumber("Motor Height:", () -> {return ArmSubsystem.getTotalHeightFromSecondaryArm(getRadiansToLength(getEncoderInRadians()));});
        tab.addNumber("Length of Arm:", () -> {return getRadiansToLength(getEncoderInRadians());});
        tab.addNumber("Motor intput:", () -> {return motor.get();});
    
    }

    /** Using Radius, Convert Length To Radians (length/radius = radians) */
    static double getLengthToRadians(double length){
        return (length / Constants.ArmWinchRadius);
    }
    /** Convert Radians Back To Length (radians * radius = length) */
    static double getRadiansToLength(double radians){
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
    void setpid (double setpoint){
        motor.set(pid.calculate(getEncoderInRadians(), setpoint));
    }

    double MaxLengthValue() {
        if(ArmSubsystem.getTargetRadian() > 1.4) return Constants.ArmBLength;

        return Constants.ArmBLength * Math.min(Math.max(
            ((Constants.clawLength+Constants.armHeight)/Math.cos(ArmSubsystem.getTargetRadian())-Constants.ArmALength)/Constants.ArmBLength
        ,0), 1);
    }

    public void setLength(double v) {
        targetLength = Math.min(Math.max(v,0),MaxLengthValue());
    }
    
    public void setLength(Constants.ArmStates mState) {
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
            case Ground:
                target = getLengthToRadians(Constants.groundGoalTeleLength);
                break;
            default:
                break;
        }
        targetLength = target;
    }
    //motor position(rotation)--Done
    //motor height
    //length of arm--Done
    //output of pid

    @Override
    public void periodic() {
        setpid(getLengthToRadians(targetLength));
    }

    @Override
    public void simulationPeriodic() {

    }
}

