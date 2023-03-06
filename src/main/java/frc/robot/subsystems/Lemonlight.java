package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Lemonlight{
    
    NetworkTableEntry tx;
    NetworkTableEntry ty;
    NetworkTableEntry ta;
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
        this.posesub = table.getDoubleArrayTopic("botpose").subscribe(new double[] {});
    }

    public Pose3d getRobotPose() {
        double[] result = posesub.get();
        tran3d = new Translation3d(result[0], result[1], result[2]);
        r3d = new Rotation3d(result[3], result[4], result[5]);
        p3d = new Pose3d(tran3d, r3d);
        return p3d;
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

        // tran3d = new Translation3d(result[0], result[1], result[2]);
        // r3d = new Rotation3d(result[3], result[4], result[5]);
        // p3d = new Pose3d(tran3d, r3d);


        // double area = ta.getDouble(0.0);
        System.out.println("LimelightX: "+ x);
        System.out.println("LimelightY: "+ y);
        // SmartDashboard.putNumber("LimelightArea", area);
    }   

    public void initTheLemon() {
   }
}