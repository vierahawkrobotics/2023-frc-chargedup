// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.ClawSubsystem;
// import frc.robot.Constants;

// public class SetClawCommand extends CommandBase {
//     public ClawSubsystem claw_subsystem;
//     public Constants.ClawStates m_state;
//     public SetClawCommand(Constants.ClawStates state, ClawSubsystem ClawSubsystem) {
//         this.claw_subsystem = ClawSubsystem;
//         addRequirements(claw_subsystem);
//         m_state = state;
//     }

//     @Override
//     public void initialize() {
//     }

//     @Override
//     public void execute() {
//         switch(m_state) {
//             case Open:
//                 claw_subsystem.OpenClaw();
//                 break;
//             case Closed:
//                 claw_subsystem.CloseClaw();
//                 break;
//             case Toggle:
//                 claw_subsystem.toggleClaw();
//                 break;
//         }
//     }

//     @Override
//     public void end(boolean ending) {   
//     }

//     @Override
//     public boolean isFinished() {
//         return true;
//     }
// }