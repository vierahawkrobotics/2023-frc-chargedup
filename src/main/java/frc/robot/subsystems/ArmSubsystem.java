package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import java.lang.Math;
import java.util.Map;

import frc.robot.Constants;
import frc.robot.commands.JoystickArmStateCommand;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {

    static double targetRadian;
    
    /***
     * Defines the CANS spark max motor, and the motor type
     */
    final static CANSparkMax motor = new CANSparkMax(Constants.RotationArmConstants.armMotorID, MotorType.kBrushless);
    /***
     * Defines the spark max encoder, and gets the absolute encoder
     */
    final static SparkMaxAbsoluteEncoder encoder = motor.getAbsoluteEncoder(Type.kDutyCycle);
    /***
     * Defines the PID controller in variables ArmP, ArmI, ArmD
     */
    public final static PIDController armPID = new PIDController(Constants.RotationArmConstants.ArmP, Constants.RotationArmConstants.ArmI, Constants.RotationArmConstants.ArmD);

    public ArmSubsystem() {

        armPID.setIntegratorRange(-1, 1);
       setName("name");
        ShuffleboardTab tab = Shuffleboard.getTab("Arm Rotation");
        //tab.addNumber(getName(), null)
        tab.addNumber("Motor Position:", () -> {return getPosition();});
        tab.addNumber("Motor Height (M):", () -> {return getHeightPosition();});
        //tab.addNumber("Length of Arm:", () -> {return tSubsystem.getRadiansToLength(encoder.getPosition());});
        tab.addNumber("Motor Input:", () -> {return motor.get();});
        tab.addNumber("Motor RPM:", () -> {return motor.getEncoder().getVelocity();});

        Shuffleboard.getTab("Main").addString("Arm State", () -> {return getState();}).withPosition(1, 2);
        Shuffleboard.getTab("Main").getLayout("Arm PID", BuiltInLayouts.kList).withPosition(8, 2) .withProperties(Map.of("Label Position", "HIDDEN")).withSize(2, 2).add(armPID).withWidget(BuiltInWidgets.kPIDController);
    }

    private String getState() {
        Constants.ArmStates state = JoystickArmStateCommand.currentState;

        if (state == Constants.ArmStates.Ground){
            return "Ground";
        }

        else if (state == Constants.ArmStates.Collect){
            return "Collect";
        }

        else if (state == Constants.ArmStates.Low){
            return "Low";
        }

        else if (state == Constants.ArmStates.Middle){
            return "Middle";
        }

        else{
            return "High";
        }
    }
/**
     * Sets the target of the pid
     * 
     * @param target target in radians that the position of the arm should be in.
     */
    void setPID(double target) {
        motor.set(armPID.calculate(getPosition(), target));
    }

    /***
     * The arms height to radians
     * 
     * @param height The height of the arm
     * @return Returns the height - arm A length / arm B length
     */

    static double getHeightToRadians(double height) {
        return Math.acos((height - Constants.RotationArmConstants.ArmALength) / Constants.TelescopeArmConstants.ArmBLength);
    }

    /***
     * The arms radians to height
     * 
     * @param radians The radians of the arm
     * @return Returns the arm A length - radians * arm B length
     */

    static double getRadiansToHeight(double radians) {
        return Constants.armHeight - Math.cos(radians) * Constants.RotationArmConstants.ArmALength;
    }

    /***
     * The arms radians to height
     * 
     * @param radians The radians of the arm
     * @return Returns the arm A length - radians * arm B length
     */

     static double getTotalHeightFromSecondaryArm(double armBlength) {
        return Constants.armHeight - Math.cos(targetRadian) * (Constants.RotationArmConstants.ArmALength+armBlength);
    }

    /***
     * Gets the position of the arm
     * 
     * @return Returns the position of the arm
     */
    public double getPosition() {
        double v= encoder.getPosition() * Math.PI * 2;
        if (v > 0.8 * Math.PI * 2) {
            return v - Math.PI * 2;
        }
        return v;
    }

    public double getHeightPosition() {
        return getRadiansToHeight(getPosition());
    }

    /***
     * Sets the target height
     * 
     * @param height The height and the position of the arm(decimal)
     */
    void setTargetHeight(double height) {
        motor.set(armPID.calculate(getPosition(), getHeightToRadians(height)));
    }

    /***
     * Sets the target rotation
     * 
     * @param radians Calculates the armPID, radians, and gets position
     */
    void setTargetRotation(double radians) {
        double v = armPID.calculate(getPosition(), radians);
        if(v < 0.01 && v > -0.01) v = 0;
        if(v > 0.8) v = 0.8;
        if(v < -0.8) v = -0.8;
        motor.set(v);
    }

    /***
     * Sets the arsm height in contants
     * 
     * @param mState The new state to use in the arm
     */

    public void setRadian(Constants.ArmStates mState) {
        double target = 0;
        switch (mState) {
            case Low:
                target = Constants.RotationArmConstants.lowGoalRadian;
                break;
            case Middle:
                target = Constants.RotationArmConstants.middleGoalRadian;
                break;
            case High:
                target = Constants.RotationArmConstants.highGoalRadian;
                break;
            case Collect:
                target = Constants.RotationArmConstants.lowGoalRadian;
            case Ground:
                target = Constants.RotationArmConstants.groundGoalRadian;
                break;
            default:
                break;
        }

        targetRadian = target;
    }

    public void setHeight(double height) {
        targetRadian = getHeightToRadians(height);
    }
    
    public static void setTargetRadian(double v) {
        targetRadian = Math.min(Math.max(v,0),135.0/180.0*Math.PI);
    }

    public static double getTargetRadian() {
        return targetRadian;
    }

    public void setTargetRadianT(double v) {
        setTargetRadian(v);
    }

    public double getTargetRadianT() {
        return targetRadian;
    }

    @Override
    public void periodic() {
        // set to target rotation value
        setTargetRotation(targetRadian);       
    }

    @Override
    public void simulationPeriodic() {

    }

}