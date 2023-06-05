package frc.robot.simWrapper.motors;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.simulation.EncoderSim;

public class CANSparkMaxWrapper {

    CANSparkMax motor;
    // just use fake ones
    Encoder m_encoder;
	EncoderSim m_encoderSim;

    public CANSparkMaxWrapper(CANSparkMax motor){
        this.motor = motor;
    }
    
}
