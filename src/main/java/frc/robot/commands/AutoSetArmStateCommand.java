package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmStates;
import frc.robot.Constants.RotationArmConstants;
//import edu.wpi.first.wpilibj2.command.Subsystem;
//import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ArmSubsystem;
//import frc.robot.Constants;
//import frc.robot.Robot;
//import frc.robot.RobotContainer;
import frc.robot.subsystems.TelescopeSubsystem;
import frc.robot.subsystems.motorClawSubsystem;

public class AutoSetArmStateCommand extends CommandBase {
    public ArmSubsystem arm_subsystem;
    public TelescopeSubsystem tSubsystem;
    public Constants.ArmStates m_state;
    public motorClawSubsystem motorClaw;
    boolean complete;


    public AutoSetArmStateCommand(Constants.ArmStates state, ArmSubsystem ArmSubsystem, TelescopeSubsystem tSubsystem, motorClawSubsystem motorClawSub) {
        this.arm_subsystem = ArmSubsystem;
        this.tSubsystem = tSubsystem;
        this.motorClaw = motorClawSub;
        complete = false;



        m_state = state;

        addRequirements(arm_subsystem);
        addRequirements(tSubsystem);
        addRequirements(motorClaw);
    }

    @Override
    public void initialize() {

        arm_subsystem.setTargetRadianUsingState(m_state);
        tSubsystem.setLength(m_state);
        System.out.println("Initalizing");

    }

    @Override
    public void execute() {
        System.out.println("Executing");
        if((m_state == ArmStates.Middle) && (arm_subsystem.getPosition() > RotationArmConstants.middleGoalRadian - .2)){
            complete = true;
            System.out.println("Arm state Middle reached");
        }


        if((m_state == ArmStates.Ground) && (arm_subsystem.getPosition() < RotationArmConstants.groundGoalRadian + .2)){
            complete = true;
            System.out.println("Arm State Ground Reached");
        }

        if((m_state == ArmStates.Low) && (arm_subsystem.getPosition() > RotationArmConstants.lowGoalRadian - .1)){
            complete = true;
            System.out.println("Arm state Low reached");
        }

        
         


    }

    @Override
    public void end(boolean ending) {
        
    }

    @Override
    public boolean isFinished() {
        System.out.print("Finished");
        return complete;
    }
}