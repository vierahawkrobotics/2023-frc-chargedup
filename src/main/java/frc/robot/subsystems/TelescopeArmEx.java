package frc.robot.subsystems;


import frc.robot.Constants;


public class TelescopeArmEx {

    CANSparkMax rotation = new CANSparkMax(Constants.extensionMotorID, MotorType.kBrushless);
    
        public Subsystem() {
            setName("name");
        }
    
        @Override
        public void periodic() {
    
        }
    
        @Override
        public void simulationPeriodic() {
    
        }
    
}
