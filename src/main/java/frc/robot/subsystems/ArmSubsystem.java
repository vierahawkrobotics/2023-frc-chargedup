package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import java.lang.Math;
import frc.robot.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
    static double targetRadian;
    
    /***
     * Defines the CANS spark max motor, and the motor type
     */
    final static CANSparkMax motor = new CANSparkMax(Constants.armMotorID, MotorType.kBrushless);
    /***
     * Defines the spark max encoder, and gets the absolute encoder
     */
    final static SparkMaxAbsoluteEncoder encoder = motor.getAbsoluteEncoder(Type.kDutyCycle);
    /***
     * Defines the PID controller in variables ArmP, ArmI, ArmD
     */
    final PIDController pidController = new PIDController(Constants.ArmP, Constants.ArmI, Constants.ArmD);

    public ArmSubsystem() {
        setName("name");

        ShuffleboardTab tab = Shuffleboard.getTab("Arm Rotation");
        //tab.addNumber(getName(), null)
        tab.addNumber("Motor Position:", () -> {return encoder.getPosition();});
        tab.addNumber("Motor Height:", () -> {return ArmSubsystem.getRadiansToHeight(encoder.getPosition());});
        //tab.addNumber("Length of Arm:", () -> {return tSubsystem.getRadiansToLength(encoder.getPosition());});
        tab.addNumber("Motor Input:", () -> {return motor.get();});
    }

    /**
     * Sets the target of the pid
     * 
     * @param target target in radians that the position of the arm should be in.
     */
    void setPID(double target) {
        motor.set(pidController.calculate(encoder.getPosition(), target));
    }

    /***
     * The arms height to radians
     * 
     * @param height The height of the arm
     * @return Returns the height - arm A length / arm B length
     */

    static double getHeightToRadians(double height) {
        return Math.acos((height - Constants.ArmALength) / Constants.ArmBLength);
    }

    /***
     * The arms radians to height
     * 
     * @param radians The radians of the arm
     * @return Returns the arm A length - radians * arm B length
     */

    static double getRadiansToHeight(double radians) {
        return Constants.ArmALength - Math.cos(radians) * Constants.ArmBLength;
    }

    /***
     * Gets the position of the arm
     * 
     * @return Returns the position of the arm
     */
    public double getPosition() {
        return encoder.getPosition();
    }

    public double getHeightPosition() {
        return getRadiansToHeight(encoder.getPosition());
    }

    /***
     * Sets the target height
     * 
     * @param height The height and the position of the arm(decimal)
     */
    void setTargetHeight(double height) {
        motor.set(pidController.calculate(encoder.getPosition(), getHeightToRadians(height)));
    }

    /***
     * Sets the target rotation
     * 
     * @param radians Calculates the pidController, radians, and gets position
     */
    void setTargetRotation(double radians) {
        motor.set(pidController.calculate(encoder.getPosition(), radians));
    }

    /***
     * Sets the arsm height in contants
     * 
     * @param mState The new state to use in the arm
     */

    public void setHeight(Constants.ArmStates mState) {
        double target = 0;
        switch (mState) {
            case Low:
                target = getHeightToRadians(Constants.lowGoalHeight);
                break;
            case Middle:
                target = getHeightToRadians(Constants.middleGoalHeight);
                break;
            case High:
                target = getHeightToRadians(Constants.highGoalHeight);
                break;
            case Ground:
                target = getHeightToRadians(Constants.groundGoalHeight);
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
        targetRadian = Math.min(Math.max(v,0),135/180*Math.PI);
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