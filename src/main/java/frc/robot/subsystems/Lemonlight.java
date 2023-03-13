package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class Lemonlight{
    
    NetworkTableEntry tx;
    NetworkTableEntry ty;
    NetworkTableEntry ta;
    NetworkTableEntry tid;
    DoubleArraySubscriber posesub;
    NetworkTable table;

    private Translation3d tran3d;
    private Rotation3d r3d;
    private Pose3d p3d;

    public Lemonlight(){    
        this.table = NetworkTableInstance.getDefault().getTable("limelight");
        this.tx = table.getEntry("tx");
        this.ty = table.getEntry("ty");
        this.ta = table.getEntry("ta");
        this.tid = table.getEntry("tid");
        this.posesub = table.getDoubleArrayTopic("botpose").subscribe(new double[] {});   
    }

    public double[] getRobotPose() {
        double[] result = posesub.get();

        if(result.length > 0){
            tran3d = new Translation3d(result[0], result[1], result[2]);
            r3d = new Rotation3d(result[3], result[4], result[5]);
            p3d = new Pose3d(tran3d, r3d);
            System.out.println(p3d);
        }

        return result;
    }

    public double getAprilTagID(){
        double id = tid.getDouble(-1);
        return id;
    }

    public void lemonLightPeriodic(){
        double x = tx.getDouble(0.0);
        double y = ty.getDouble(0.0);   
        System.out.println(getRobotPose());
    }   

    public Pose2d getPose(){
        return new Pose2d(new Translation2d(getRobotPose()[0], getRobotPose()[1]), new Rotation2d(getRobotPose()[2]));
    }
}