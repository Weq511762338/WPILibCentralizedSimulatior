// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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

/** Add your docs here. */
public class CANSparkMaxSim extends CANSparkMax{

    Encoder m_encoder;
	EncoderSim m_encoderSim;

    // TODO: is this good? the built-in PID Controller will be used for advanced control directly without calling set(), such as in TrapezoidalProfilingControl: setReference()f'vfv
    PIDController m_PIDController = new PIDController(ModuleConstants.kPModuleDriveController, 0, 0);

    FlywheelSim m_motorSim = new FlywheelSim(
		LinearSystemId.identifyVelocitySystem(Constants.DriveConstants.kvVoltSecondsPerMeter, Constants.DriveConstants.kaVoltSecondsSquaredPerMeter),
		ModuleConstants.kDriveMotorGearbox,
		ModuleConstants.kDriveGearRatio
	);

    public CANSparkMaxSim(int deviceId, MotorType type) {
        super(deviceId, type);

        m_encoder = new Encoder(0, 1);
		m_encoderSim = new EncoderSim(m_encoder);
    }

    public CANSparkMaxSim(int deviceId, MotorType type, double encoderDistancePerPulse) {
        this(deviceId, type);

        m_encoder.setDistancePerPulse(encoderDistancePerPulse);
    }

    @Override
    public void set(double speed) {
        double driveOutput = m_PIDController.calculate(m_encoder.getRate(), speed);
		// set the output on actual devices
        super.set(driveOutput);

		m_motorSim.setInputVoltage(driveOutput / Constants.DriveConstants.kMaxSpeedMetersPerSecond * RobotController.getBatteryVoltage());
    }

    public void simulationPeriodic(){
        m_motorSim.update(0.2);
		m_encoderSim.setDistance(m_encoderSim.getDistance() + 0.2*m_encoderSim.getRate());

        m_encoderSim.setRate(m_motorSim.getAngularVelocityRadPerSec());
        
		SmartDashboard.putNumber("sim rate", m_encoderSim.getRate());
		SmartDashboard.putNumber("rate", m_encoder.getRate());
		SmartDashboard.putNumber("sim angular rad per sec", m_motorSim.getAngularVelocityRadPerSec());
		SmartDashboard.putNumber("distance", m_encoder.getDistance());
        SmartDashboard.putNumber("sim distance", m_encoderSim.getDistance());
    }
}
