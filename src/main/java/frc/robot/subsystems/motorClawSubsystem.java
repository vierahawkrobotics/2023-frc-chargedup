package frc.robot.subsystems;

import java.util.Map;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class motorClawSubsystem extends SubsystemBase {

    final CANSparkMax clawMotor = new CANSparkMax(Constants.motorClawSubsystemConstants.clawMotorID,
            MotorType.kBrushless);
    // Add to constants
    public final PIDController clawPID = new PIDController(Constants.motorClawSubsystemConstants.clawP,
            Constants.motorClawSubsystemConstants.clawI, Constants.motorClawSubsystemConstants.clawD);

    private double targetRotationalSpeed = 0;

    public boolean isStalled = false;

    private static GenericEntry collectionSpeedWidget;
    private static GenericEntry depositSpeedWidget;

    public motorClawSubsystem() {
        // clawMotor.getEncoder().setVelocityConversionFactor(2 * Math.PI);
        clawMotor.setIdleMode(IdleMode.kBrake);

        clawMotor.setSmartCurrentLimit(35);

        collectionSpeedWidget = Shuffleboard.getTab("Main").getLayout("Claw Speed", BuiltInLayouts.kList).withPosition(1, 2).withSize(1, 2).withProperties(Map.of("Label Position", "TOP")).add("Collection RPM", Constants.motorClawSubsystemConstants.collectionSpeed).withSize(1, 2).getEntry();
        depositSpeedWidget = Shuffleboard.getTab("Main").getLayout("Claw Speed", BuiltInLayouts.kList).add("Deposit RPM", Constants.motorClawSubsystemConstants.depositSpeed).withSize(1, 2).getEntry();
    }

    public double getMotorVelocity() {
        return clawMotor.getEncoder().getVelocity();
    }

    public void updatePID(boolean collectCube, boolean depositCube, boolean collectCone, boolean depositCone) {

        double input = -0.05;
        
        if(collectCube){
            input = -0.3;
        }

        if(depositCube){
            input = 0.2;
        }

        if (collectCone){
            input = -0.5;
        }

        if (depositCone){
            input = 0.3;
        }
       
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
