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

    // sparkmax motor + absolute encode
    // Set speed
    // Encoder Position
    // ------------------------------------
    // pidcontroller
    // function to set target position DOne

    // command to set target position
    // PIDController pid = new PIDController(kP, kI, kD); Done
    // state machine for different hights

    // motor.set(pidController.calculate(currentRotation, target)); Done

    final CANSparkMax motor = new CANSparkMax(Constants.armMotorID, MotorType.kBrushless);
    final SparkMaxAbsoluteEncoder encoder = motor.getAbsoluteEncoder(Type.kDutyCycle);
    final PIDController pidController = new PIDController(Constants.ArmP, Constants.ArmI, Constants.ArmD);

    /*
     * haleelfk
     */
    public void setPID(double target) {
        motor.set(pidController.calculate(encoder.getPosition(), target));
    }

    public static double getHeightToRadians(double height) {
        return Math.acos((height - Constants.ArmALength) / Constants.ArmBLength);
    }

    public static double getRadiansToHeight(double radians) {
        return Constants.ArmALength - Math.cos(radians) * Constants.ArmBLength;
    }

    public void setSpeed(double speed) {
        motor.set(speed);
    }

    public double getPosition() {
        return getPosition();
    }

    public void setTarget() {
    }

    public void setTargetHeight(double height) {

        motor.set(pidController.calculate(encoder.getPosition(), getHeightToRadians(height)));
    }

    public void setTargetRotation(double radians) {

        motor.set(pidController.calculate(encoder.getPosition(), radians));
    }

    public ArmSubsystem() {
        setName("name");
    }

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

    private void armRotation() {

    }
}