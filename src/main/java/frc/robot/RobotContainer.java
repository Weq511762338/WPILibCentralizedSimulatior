// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Motor;

public class RobotContainer {
	XboxController m_driverController = new XboxController(0);

	Motor m_motor = new Motor();

	public RobotContainer() {
		configureBindings();

		var driveCommand = new RunCommand(
				() -> m_motor.set(m_driverController.getLeftX() * DriveConstants.kMaxSpeedMetersPerSecond));

		driveCommand.addRequirements(m_motor);
		m_motor.setDefaultCommand(driveCommand);
	}

	private void configureBindings() {
	}

	public Command getAutonomousCommand() {
		return Commands.print("No autonomous command configured");
	}

}
