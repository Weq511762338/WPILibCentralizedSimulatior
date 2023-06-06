// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.simWrapper.motors.CANSparkMaxWrapper;

public class Motor extends SubsystemBase {
	CANSparkMax m_motor;
	CANSparkMaxWrapper m_simWrapper;

	/** Creates a new Motor. */
	public Motor() {
		m_motor = new CANSparkMax(0, MotorType.kBrushless);
		m_simWrapper = new CANSparkMaxWrapper(m_motor, false);
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
