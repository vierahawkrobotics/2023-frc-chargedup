package frc.robot.commands;

import java.util.ArrayList;

import com.pathplanner.lib.PathPoint;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Lemonlight;
import frc.robot.subsystems.TelescopeSubsystem;

public class TeleOpPlacement extends SequentialCommandGroup {
    private DriveSubsystem drive;
    private ArmSubsystem armSub;
    private TelescopeSubsystem telescopeSubsystem;
    private Lemonlight lemonLight;
    private int Path;    

    private PathPoint chosePathPoint;
    private Command JoystickArmStateCommand;


    FieldConstants fieldConstants = new FieldConstants();
    AutoConstants autoConstants = new AutoConstants();
    private ArrayList<PathPoint> aprilTagPoseList = fieldConstants.getUpperPathPoints();
    
    //Initialize Arm Subsystem
    //Intialize Telescoping Subsyem

    public PathPoint getRoutine(int Path){
        if(lemonLight.getAprilTagID() == 1){
            aprilTagPoseList = fieldConstants.getUpperPathPoints();
        } else if (lemonLight.getAprilTagID() == 2){
            aprilTagPoseList = fieldConstants.getMiddPoints();
        } else if (lemonLight.getAprilTagID() == 3){
            aprilTagPoseList = fieldConstants.getBottomPoints();
        }

        if (Path == 1 || Path == 2) {
            chosePathPoint = aprilTagPoseList.get(0);
        } else if (Path == 3 || Path == 4) {
            chosePathPoint = aprilTagPoseList.get(1);
        } else if (Path == 5 || Path == 6) {
            chosePathPoint = aprilTagPoseList.get(2);
        }

        return chosePathPoint;
    }

    //fix this
    public Command getArmSubCommand(int Path){
        if(Path == 1 || Path == 2 || Path == 3){
            JoystickArmStateCommand = new SetArmStateCommand(Constants.ArmStates.High, armSub, telescopeSubsystem);

        } else {
            JoystickArmStateCommand = new SetArmStateCommand(Constants.ArmStates.Middle, armSub, telescopeSubsystem);
        }

        return JoystickArmStateCommand;
    }

    

    public TeleOpPlacement (DriveSubsystem drive, ArmSubsystem armSub, TelescopeSubsystem tele, int Path){
        this.drive = drive;
        this.armSub = armSub;
        this.telescopeSubsystem = tele;
        this.Path = Path;


        addCommands(
            new OnTheFlyPathGen(drive, lemonLight, getRoutine(Path)),
            getArmSubCommand(Path)
        );
        
    }


}

