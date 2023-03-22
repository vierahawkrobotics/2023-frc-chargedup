package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class motorClawSubsystem extends SubsystemBase {

    // Find motor id and add it to constants class
    final CANSparkMax clawMotor = new CANSparkMax(13, MotorType.kBrushless);
    // Add to constants
    public final PIDController clawPID = new PIDController(0, 0, 0);

    private double targetRotationalSpeed = 0;

    public boolean isStalled = false;

    public motorClawSubsystem() {
        // clawMotor.getEncoder().setVelocityConversionFactor(2 * Math.PI);
        clawMotor.setIdleMode(IdleMode.kBrake);
    }

    public double getMotorVelocity() {
        return clawMotor.getEncoder().getVelocity();
    }

    public void updatePID() {
        double input = clawPID.calculate(getMotorVelocity(), targetRotationalSpeed);
        if (Math.abs(input) < 0.05)
            input = 0;
            targetRotationalSpeed = 0;

        if (clawMotor.getOutputCurrent() > 10){
            input = 0;
            isStalled = true;
            targetRotationalSpeed = 0;
        }

        if (clawMotor.getMotorTemperature() > 100)
            input = 0;
            targetRotationalSpeed = 0;

        clawMotor.set(input);
    }

    public void setTargetSpeed(double rotationalSpeed) {
        targetRotationalSpeed = rotationalSpeed;
    }

    @Override
    public void periodic() {

    }

    @Override
    public void simulationPeriodic() {

    }
}
