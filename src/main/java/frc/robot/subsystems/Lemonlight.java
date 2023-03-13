package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.LemonlightConstants;

public class Lemonlight{
    
    NetworkTableEntry tx;
    NetworkTableEntry ty;
    NetworkTableEntry ta;
    NetworkTableEntry tid;
    DoubleArraySubscriber posesub;
    static NetworkTable table;

    private Translation3d tran3d;
    private Rotation3d r3d;
    private Pose3d p3d;

    GenericEntry lemonlightCoorX;
    GenericEntry lemonlightCoorY;
    GenericEntry lemonlightCoorZ;

    LemonlightConstants lemonlightConstants = new LemonlightConstants();
    ShuffleboardTab tab;

    public Lemonlight(){    
        this.table = NetworkTableInstance.getDefault().getTable("limelight");
        this.tx = table.getEntry("tx");
        this.ty = table.getEntry("ty");
        this.ta = table.getEntry("ta");
        this.tid = table.getEntry("tid");
        this.posesub = table.getDoubleArrayTopic("botpose").subscribe(new double[] {});

        tab = Shuffleboard.getTab("Limelight");
        lemonlightCoorX = tab.add("Limelight X Coordinate",0).withPosition(3,3).getEntry();
        lemonlightCoorY = tab.add("Limelight Y Coordinate",0).withPosition(3,3).getEntry();
        lemonlightCoorZ = tab.add("Limelight Z Coordinate",0).withPosition(3,3).getEntry();
    }
    
    public double[] getRobotPose() {
        double[] result = posesub.get();
        p3d = new Pose3d(tran3d, r3d);
        tran3d = new Translation3d(result[0], result[1], result[2]);
        r3d = new Rotation3d(result[3], result[4], result[5]);
        
        return result;
    }

    public Pose3d getposition() {
        double[] temp = { 0, 0, 0 ,0,0,0};//default for getEntry
        NetworkTableEntry value = table.getEntry("botpose");
        double[] result = value.getDoubleArray(temp);
        tran3d = new Translation3d(result[0], result[1], result[2]);
        r3d = new Rotation3d(result[3], result[4], result[5]);
        p3d = new Pose3d(tran3d, r3d);
        return p3d;
    }

    public double getAprilTagID(){
        double id = tid.getDouble(-1);
        return id;
    }

    public static double getArea(){
        NetworkTableEntry ta = table.getEntry("ta");
        double c = ta.getDouble(0.0);
        return c;
    }

    public Translation2d getTranslation2D(){
        return new Translation2d(getRobotPose()[0], getRobotPose()[1]);
    }

    public Rotation2d getHolonomicRotation2d() {
        return new Rotation2d(getRobotPose()[2]);
    }

    public Pose2d getPose2d () {
        return new Pose2d(getTranslation2D(), getHolonomicRotation2d());
    } 

    public boolean aprilTag(){
        double[] result = posesub.get();
        if(result[0] != 0){
            return true;
        }
        else{
            return false;
        }
    }

    public void lemonLightPeriodic(){
        double x = tx.getDouble(0.0);
        double y = ty.getDouble(0.0);

        double[] result = posesub.get();

        if(result.length > 0){
            tran3d = new Translation3d(result[0], result[1], result[2]);
            r3d = new Rotation3d(result[3], result[4], result[5]);
            p3d = new Pose3d(tran3d, r3d);
            System.out.println(p3d);
        }

       // lemonlightCoorX = getAsDouble(getRobotPose().getTranslation().getX());
      //  lemonlightCoorY = getAsDouble(getRobotPose().getTranslation().getY());
       // lemonlightCoorZ = getAsDouble(getRobotPose().getTranslation().getZ());


        System.out.println(getAprilTagID());
        System.out.println(getTranslation2D());
    }   

    private GenericEntry getAsDouble(double x) {
        return null;
    }

    public void initTheLemon() {
   }

    
}