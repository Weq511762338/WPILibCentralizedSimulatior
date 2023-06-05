// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;

public class Motor extends SubsystemBase {
	CANSparkMax m_motor;
	Encoder m_encoder;
	EncoderSim m_encoderSim;

	PIDController m_PIDController = new PIDController(ModuleConstants.kPModuleDriveController, 0, 0);
	
	double m_driveOutput;

	private final FlywheelSim m_driveMotorSim = new FlywheelSim(
		LinearSystemId.identifyVelocitySystem(Constants.DriveConstants.kvVoltSecondsPerMeter, Constants.DriveConstants.kaVoltSecondsSquaredPerMeter),
		ModuleConstants.kDriveMotorGearbox,
		ModuleConstants.kDriveGearRatio
	);

	/** Creates a new Motor. */
	public Motor() {
		m_motor = new CANSparkMax(0, MotorType.kBrushless);
		m_encoder = new Encoder(0, 1);
		m_encoderSim = new EncoderSim(m_encoder);		    

		m_encoder.setDistancePerPulse(ModuleConstants.kDriveEncoderDistancePerPulse);

		// m_encoder.setReverseDirection(driveEncoderReversed);

	}

	public void set(double speedMetersPerSecond){
		// System.out.println(speedMetersPerSecond);
		m_driveOutput = m_PIDController.calculate(m_encoder.getRate(), speedMetersPerSecond);
		// m_motor.set(m_driveOutput);
		m_driveMotorSim.setInputVoltage(m_driveOutput / Constants.DriveConstants.kMaxSpeedMetersPerSecond * RobotController.getBatteryVoltage());
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}

	@Override
	public void simulationPeriodic(){
		m_driveMotorSim.update(0.2);
		SmartDashboard.putNumber("distance", m_encoderSim.getDistance());
		SmartDashboard.putNumber("output", m_driveOutput);
		m_encoderSim.setRate(m_driveMotorSim.getAngularVelocityRadPerSec());
	}
}
