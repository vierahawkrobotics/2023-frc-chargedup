package frc.robot;

public class Constants {
    //Rotational arm constants
    public final static double ArmP = 1.3;
    public final static double ArmI = 0.00;
    public final static double ArmD = 0.0000;
    public final static int armMotorID = 9;
    public final static double ArmALength = 0.6477;
    //Telescope constants
    public final static double ArmWinchRadius = 0.5;
    public final static double ScopeP = 0.2;
    public final static double ScopeI = 0;
    public final static double ScopeD = 0;
    public final static int scopeMotorID = 10;
    public final static double ArmBLength = 0.4572;
    //States
    public final static double lowGoalRadian = 0.1;
    public final static double middleGoalRadian = 0.87;
    public final static double highGoalRadian = 1.55;
    public final static double groundGoalRadian = 0;
    public final static double lowGoalTeleLength = .26;
    public final static double middleGoalTeleLength = 0.04;
    public final static double highGoalTeleLength = .88;
    public final static double groundGoalTeleLength = 0.26;

    public final static double midHeight = .87;
    public final static double highHeight = 1.17;

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