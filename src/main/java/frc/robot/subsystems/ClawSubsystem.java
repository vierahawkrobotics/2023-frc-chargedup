package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClawSubsystem extends SubsystemBase {
    // Compressor pcmCompressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
    // Compressor phCompressor = new Compressor(1, PneumaticsModuleType.REVPH);

    // pcmCompressor.enableDigital();
    // pcmCompressor.disable();

    // boolean enabled = pcmCompressor.enabled();
    // boolean pressureSwitch = pcmCompressor.getPressureSwitchValue();
    // double current = pcmCompressor.getCompressorCurrent();
    public ClawSubsystem() {
        setName("claw");
    }
    @Override
    public void periodic() {

    }

    @Override
    public void simulationPeriodic() {

    }

    public void OpenClaw(){


    }

    public void CloseClaw(){
            
    }




    public void initialize(){

    }

    public void execute(){
        
    }

    public void end(){

    }

    public void isFinished(){

    }
}