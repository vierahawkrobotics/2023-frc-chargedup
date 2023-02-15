package frc.robot;

public class Constants {
    //Rotational arm constants
    public final static double ArmP = 0;
    public final static double ArmI = 0;
    public final static double ArmD = 0;
    public final static int armMotorID = 0;
    public final static double ArmALength = 0;
    //Telescope constants
    public final static double ArmWinchRadius = 0;
    public final static double ScopeP = 0;
    public final static double ScopeI = 0;
    public final static double ScopeD = 0;
    public final static int scopeMotorID = 0;
    public final static double ArmBLength = 0;
    //States
    public final static double lowGoalHeight = 0;
    public final static double middleGoalHeight = 0;
    public final static double highGoalHeight = 0;
    public final static double lowGoalTeleLength = 0;
    public final static double middleGoalTeleLength = 0;
    public final static double highGoalTeleLength = 0;
    public final static int extensionMotorID = 0;
    public static final String DRIVER_CONTROLLER_ROTATE_AXIS = null;
    public final static int buttonNumber0 = 0;
    public final static int buttonNumber1 = 1;
    public final static int buttonNumber2 = 2;
    public final static int buttonNumber3 = 3;
    public enum ArmStates{
        Low,
        Middle,
        High;
    }
}
