package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ComplexWidget;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import java.lang.Math;
import java.util.Map;
import java.util.function.DoubleSupplier;

import frc.robot.Constants;
import frc.robot.Constants.ArmStates;
import frc.robot.commands.JoystickArmStateCommand;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {

    private boolean isAdjustable;

    private final static CANSparkMax elbowMotor = new CANSparkMax(Constants.RotationArmConstants.armMotorID, MotorType.kBrushless);
    private final static SparkMaxAbsoluteEncoder encoder = elbowMotor.getAbsoluteEncoder(Type.kDutyCycle);
    public final static PIDController elbowPID = new PIDController(Constants.RotationArmConstants.ArmP, Constants.RotationArmConstants.ArmI, Constants.RotationArmConstants.ArmD);

    private double targetRadian = 0;

    public ArmSubsystem() {

        // Config
        elbowPID.setIntegratorRange(-1, 1);


              




        // Shuffleboard
        ShuffleboardTab tab = Shuffleboard.getTab("Arm Rotation");
        //tab.addNumber(getName(), null)
        tab.addNumber("elbowMotor Position:", () -> {return getPosition();});
        // tab.addNumber("elbowMotor Height (M):", () -> {return getHeightPosition();});
        //tab.addNumber("Length of Arm:", () -> {return tSubsystem.getRadiansToLength(encoder.getPosition());});
        tab.addNumber("elbowMotor Input:", () -> {return elbowMotor.get();});
        tab.addNumber("elbowMotor RPM:", () -> {return elbowMotor.getEncoder().getVelocity();});

        Shuffleboard.getTab("Arm Rotation").addString("Arm State", () -> {return getState();}).withPosition(1, 2);
        Shuffleboard.getTab("Arm Rotation").getLayout("Arm PID", BuiltInLayouts.kList).withPosition(8, 2) .withProperties(Map.of("Label Position", "HIDDEN")).withSize(2, 2).add(elbowPID).withWidget(BuiltInWidgets.kPIDController);
    }


        /***
     * Gets the position of the arm
     * 
     * @return Returns the position of the arm
     */
    public double getPosition() {
        double v = encoder.getPosition() * Math.PI * 2;
        if (v > 0.8 * Math.PI * 2) {
            return v - Math.PI * 2;
        }
        return v;
    }
    
    public double getTargetRadian() {
        return targetRadian;
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


    /***
     * Sets the target rotation
     * 
     * @param radians Calculates the elbowPID, radians, and gets position
     */
    public void updatePID(double adjust) {
        double setpoint = targetRadian - adjust;
        System.out.println(setpoint);
        double v = elbowPID.calculate(getPosition(), setpoint);
        if(v < 0.01 && v > -0.01) v = 0;
        if(v > 0.8) v = 0.8;
        if(v < -0.8) v = -0.8;
        elbowMotor.set(v);
    }
    
    /***
     * Sets the arm height in contants
     * 
     * @param mState The new state to use in the arm
     */

    public void setTargetRadianUsingState(Constants.ArmStates mState) {
        double target = 0;
        isAdjustable = false;
        switch (mState) {
            case Low:
                target = Constants.RotationArmConstants.lowGoalRadian;
                break;
            case Middle:
                target = Constants.RotationArmConstants.middleGoalRadian;
                isAdjustable = true;
                break;
            case High:
                target = Constants.RotationArmConstants.highGoalRadian;
                isAdjustable = true;
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
    
    public void setTargetRadian(double v) {
        targetRadian = Math.min(Math.max(v,0),135.0/180.0*Math.PI);
    }

    @Override
    public void periodic() {}

    @Override
    public void simulationPeriodic() {}

}