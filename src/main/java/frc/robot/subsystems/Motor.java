// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.constant.Constable;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.CentralizedSimLib.CANSparkMaxSim;

public class Motor extends SubsystemBase {
	CANSparkMax m_motor;

	/** Creates a new Motor. */
	public Motor() {
		
		if(Robot.isSimulation())
			m_motor = new CANSparkMaxSim(0, MotorType.kBrushless, Constants.ModuleConstants.kDriveEncoderDistancePerPulse);
		else
			m_motor = new CANSparkMax(0, MotorType.kBrushless);
	}


	public void set(double d){
		m_motor.set(d);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
	}

	@Override
	public void simulationPeriodic(){
		CANSparkMaxSim sparkSim = (CANSparkMaxSim) m_motor;
		sparkSim.simulationPeriodic();
	}
}
