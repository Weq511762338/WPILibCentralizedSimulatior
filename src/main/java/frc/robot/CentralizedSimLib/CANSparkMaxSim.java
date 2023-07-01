package frc.robot.CentralizedSimLib;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;

public class CANSparkMaxSim {

    // actual motor
    CANSparkMax m_motor;
    
    // simulation objects, all fake
    Encoder m_encoder;
	EncoderSim m_encoderSim;

	PIDController m_PIDController = new PIDController(ModuleConstants.kPModuleDriveController, 0, 0);

    FlywheelSim m_motorSim = new FlywheelSim(
		LinearSystemId.identifyVelocitySystem(Constants.DriveConstants.kvVoltSecondsPerMeter, Constants.DriveConstants.kaVoltSecondsSquaredPerMeter),
		ModuleConstants.kDriveMotorGearbox,
		ModuleConstants.kDriveGearRatio
	);

    public CANSparkMaxSim(CANSparkMax motor){
        m_motor = motor;

        m_encoder = new Encoder(0, 1);
		m_encoderSim = new EncoderSim(m_encoder);	
			
		
        // setup of all sim obj
        m_encoder.setDistancePerPulse(ModuleConstants.kDriveEncoderDistancePerPulse);
    }
    
    public void set(double speedMetersPerSecond){
		double driveOutput = m_PIDController.calculate(m_encoder.getRate(), speedMetersPerSecond);
		// set the output on actual devices
        m_motor.set(driveOutput);

		m_motorSim.setInputVoltage(driveOutput / Constants.DriveConstants.kMaxSpeedMetersPerSecond * RobotController.getBatteryVoltage());
	}

    public void simulationPeriodic(){
		m_motorSim.update(0.2);
		m_encoderSim.setRate(m_motorSim.getAngularVelocityRadPerSec());
		SmartDashboard.putNumber("sim rate", m_encoderSim.getRate());
		SmartDashboard.putNumber("rate", m_encoder.getRate());
		SmartDashboard.putNumber("sim angular vel", m_motorSim.getAngularVelocityRPM());
		SmartDashboard.putNumber("sim current draw amps", m_motorSim.getCurrentDrawAmps());
	}

    
}
