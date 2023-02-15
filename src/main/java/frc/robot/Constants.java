package frc.robot;

public class Constants {
    public final static double ArmP = 0;
    public final static double ArmI = 0;
    public final static double ArmD = 0;
    public final static int armMotorID = 0;
    public final static double ArmALength = 0;
    public final static double ArmBLength = 0;
    public final static double lowGoalHeight = 0;
    public final static double middleGoalHeight = 0;
    public final static double highGoalHeight = 0;
    public final static int extensionMotorID = 0;
    public static final String DRIVER_CONTROLLER_ROTATE_AXIS = null;
    public enum ArmStates{
        Low,
        Middle,
        High;
    }
    public enum ClawStates{
        Open,
        Closed,
        Toggle;
    }   
}
