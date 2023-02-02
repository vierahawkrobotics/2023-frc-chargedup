package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import java.lang.Math;
import frc.robot.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
    public double targetRadian;

    // function armhight to radians; Done
    // function radians to armhight; Done

    // sparkmax motor + absolute encode Done
    // Set speed Done
    // Encoder Position Done
    // ------------------------------------
    // pidcontroller Done
    // function to set target position DOne

    // command to set target position
    // PIDController pid = new PIDController(kP, kI, kD); Done
    // state machine for different hights Done

    // motor.set(pidController.calculate(currentRotation, target)); Done
    ///////////////////////////////////////////////////////////////////////////////////////////////////
    /***
     * Defines the CANS spark max motor, and the motor type
     */
    final static CANSparkMax motor = new CANSparkMax(Constants.armMotorID, MotorType.kBrushless);
    /***
     * Defines the spark max encoder, and gets the absolut encoder
     */
    final static SparkMaxAbsoluteEncoder encoder = motor.getAbsoluteEncoder(Type.kDutyCycle);
    /***
     * Defines the PID controller in variables ArmP, ArmI, ArmD
     */
    final PIDController pidController = new PIDController(Constants.ArmP, Constants.ArmI, Constants.ArmD);

    public ArmSubsystem() {
        setName("name");
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

    public static double getHeightToRadians(double height) {
        return Math.acos((height - Constants.ArmALength) / Constants.ArmBLength);
    }

    /***
     * The arms radians to height
     * 
     * @param radians The radians of the arm
     * @return Returns the arm A length - radians * arm B length
     */

    public static double getRadiansToHeight(double radians) {
        return Constants.ArmALength - Math.cos(radians) * Constants.ArmBLength;
    }

    /***
     * Sets the speed of the motor
     * 
     * @param speed The speed of the motor(decimal)
     * @deprecated
     */

    static void setSpeed(double speed) {
        motor.set(speed);
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
        }

        targetRadian = target;
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