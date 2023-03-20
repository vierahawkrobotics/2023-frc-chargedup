package frc.robot.AutonoumousRoutines;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.AutonmousCommands.AutoSetArmCommand;
import frc.robot.Constants.ClawStates;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.SetClawCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.TelescopeSubsystem;

public class Autonomous {
    DriveSubsystem drive;
    ArmSubsystem armSubsystem;
    TelescopeSubsystem telescopeSubsystem;
    ClawSubsystem clawSubsystem;

    public Autonomous(DriveSubsystem drive, ArmSubsystem armSubsystem, TelescopeSubsystem telescopeSubsystem, ClawSubsystem clawSubsystem){
        this.drive = drive;
        this.armSubsystem = armSubsystem;
        this.telescopeSubsystem = telescopeSubsystem;
        this.clawSubsystem = clawSubsystem;
    }

    public Command getAutoPath(PathPlannerTrajectory Path) {
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("RaiseArmToHigh", new AutoSetArmCommand(Constants.ArmStates.Middle, armSubsystem, telescopeSubsystem));
        eventMap.put("RaiseArmToLow", new AutoSetArmCommand(Constants.ArmStates.Low, armSubsystem, telescopeSubsystem));
        eventMap.put("RaiseArmToMid", new AutoSetArmCommand(Constants.ArmStates.Middle, armSubsystem, telescopeSubsystem));
        eventMap.put("LowerArmToGround", new AutoSetArmCommand(Constants.ArmStates.Ground, armSubsystem, telescopeSubsystem));
        eventMap.put("OpenClaw", new SetClawCommand(ClawStates.Open, clawSubsystem));
        eventMap.put("CloseClaw", new SetClawCommand(ClawStates.Closed, clawSubsystem));
        eventMap.put("ToggleClaw", new SetClawCommand(ClawStates.Toggle, clawSubsystem));
       // eventMap.put("Wait", new WaitCommand(.5));
        //eventMap.put("Balance", new BalanceCommand(m_robotDrive));
    
        PathPlannerTrajectory TopBalance = PathPlanner.loadPath("TopBalance", new PathConstraints(4, 3));
        HashMap<String, Command> topEventMap = new HashMap<>();
        // topEventMap.put(null, getAutonomousCommand());
    
        PathPlannerTrajectory TopPlace = PathPlanner.loadPath("TopPlace", new PathConstraints(1, .5));
        HashMap<String, Command> topPlaceEventMap = new HashMap<>();
    
        PathPlannerTrajectory MiddleBalance = PathPlanner.loadPath("MiddleBalance", new PathConstraints(.5, .5));
        HashMap<String, Command> middleBalanceEventMap = new HashMap<>();
    
        PathPlannerTrajectory BottomBalance = PathPlanner.loadPath("BottomBalance", new PathConstraints(1, .5));
        HashMap<String, Command> bottomBalanceEventMap = new HashMap<>();
    
        PathPlannerTrajectory BottomPlace = PathPlanner.loadPath("BottomPlace", new PathConstraints(1, .5));
        HashMap<String, Command> bottomPlaceEventMap = new HashMap<>();
    
        PathPlannerTrajectory Straight = PathPlanner.loadPath("Straight", new PathConstraints(1, .5));
    
        PathPlannerTrajectory Simple = PathPlanner.loadPath("Simple", new PathConstraints(2, .5));
    
        PathPlannerTrajectory Middle = PathPlanner.loadPath("Middle", new PathConstraints(2, 1));
    
        PathPlannerTrajectory Sides = PathPlanner.loadPath("Sides", new PathConstraints(.5, .5));
    
        PathPlannerTrajectory Bal = PathPlanner.loadPath("Bal", new PathConstraints(2,.5));
    
    
        //PathPlannerTrajectory Fun = PathPlanner.loadPath("Fun", new PathConstraints(2,.5));
    
        SwerveAutoBuilder swerveAutoBuilder = new SwerveAutoBuilder(
            drive::getPose,
            drive::resetOdometry,
            DriveConstants.kDriveKinematics,
            new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y
                                             // PID controllers)
            new PIDConstants(8, 0.0, 0.0),
            drive::setModuleStates,
            eventMap,
            true,
            drive);
    
        Command FullAuto = swerveAutoBuilder.fullAuto(Path);
    
        // Run path following command, then stop at the end.
        // return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0,
        // false));
        return FullAuto;
      }

      
    
}
