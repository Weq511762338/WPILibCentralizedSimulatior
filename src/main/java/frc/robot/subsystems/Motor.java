// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.CentralizedSimLib.CANSparkMaxSim;

public class Motor extends SubsystemBase {
	CANSparkMax m_motor;
	CANSparkMaxSim m_simWrapper = null;

	/** Creates a new Motor. */
	public Motor() {
			m_motor = new CANSparkMax(0, MotorType.kBrushless);

			SparkMaxRelativeEncoder m_encoder = (SparkMaxRelativeEncoder) m_motor.getEncoder();
	}


	public void set(double d){
		// this is also going to set the speed of acutal motor
		m_simWrapper.set(d);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}

	@Override
	public void simulationPeriodic(){
		m_simWrapper.simulationPeriodic();
	}
}
