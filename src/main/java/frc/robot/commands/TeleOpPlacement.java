package frc.robot.commands;

import java.util.ArrayList;

import com.pathplanner.lib.PathPoint;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Lemonlight;

public class TeleOpPlacement extends SequentialCommandGroup {
    DriveSubsystem drive;
    int Path;    
    private PathPoint chosePathPoint;
    private Lemonlight lemonLight;
    private Command JoystickArmStateCommand;


    Lemonlight lemonlight = new Lemonlight();
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
            JoystickArmStateCommand = new JoystickArmStateCommand(Path, null, null);

        } else {
            JoystickArmStateCommand = new JoystickArmStateCommand(Path, null, null);
        }

        return JoystickArmStateCommand;
    }

    

    public TeleOpPlacement (DriveSubsystem drive, int Path){
        this.drive = drive;
        this.
        Path = Path;

        addCommands(
            new OnTheFlyPathGen(drive, lemonlight, null),
            getArmSubCommand(Path)
        );
        
    }


}

