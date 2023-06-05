package frc.robot.simWrapper.motors;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.simulation.EncoderSim;

public class CANSparkMaxWrapper {

    CANSparkMax motor;

    public CANSparkMaxWrapper(CANSparkMax motor){
        this.motor = motor;
    }
    
}
