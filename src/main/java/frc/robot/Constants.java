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
    public enum ArmStates{
        Low,
        Middle,
        High;
    }
}
