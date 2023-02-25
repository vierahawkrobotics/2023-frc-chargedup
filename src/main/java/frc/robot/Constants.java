package frc.robot;

public class Constants {
    //Rotational arm constants
    public final static double ArmP = 0.2;
    public final static double ArmI = 0.00;
    public final static double ArmD = 0;
    public final static int armMotorID = 0;
    public final static double ArmALength = 0.6477;
    //Telescope constants
    public final static double ArmWinchRadius = 0;
    public final static double ScopeP = 0;
    public final static double ScopeI = 0;
    public final static double ScopeD = 0;
    public final static int scopeMotorID = 0;
    public final static double ArmBLength = 0.4572;
    //States
    public final static double lowGoalHeight = 0;
    public final static double middleGoalHeight = 0;
    public final static double highGoalHeight = 0;
    public final static double groundGoalHeight = 0;
    public final static double lowGoalTeleLength = 0;
    public final static double middleGoalTeleLength = 0;
    public final static double highGoalTeleLength = 0;
    public final static double groundGoalTeleLength = 0;
    public enum ArmStates{
        Ground,
        Low,
        Middle,
        High;
    }
    public enum ClawStates {
        Open,
        Closed,
        Toggle;
    }

    public final static double clawLength = 0.305;
    public final static double armHeight = 1.05;
}


//~37 distance between fully extended arm touching ground and robot front
//64.77 cm arm A length
//71.12 cm - 10in. arm B length
//12 in. claw pads to armB
//Claw <2lb
//Whole arm ~15 lb