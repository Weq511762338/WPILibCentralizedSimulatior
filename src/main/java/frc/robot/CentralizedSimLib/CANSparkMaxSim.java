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

        // TODO: check if this should be called by encoder or encoderSim. Change the disPerPulse and see if anything changes?
        m_encoder.setDistancePerPulse(encoderDistancePerPulse);
    }


    @Override
    public double get() {
        // TODO Auto-generated method stub
        return super.get();
    }

    @Override
    public void set(double speed) {

        double driveOutput = m_PIDController.calculate(m_encoder.getRate(), speed);
		// set the output on actual devices
        super.set(driveOutput);

		m_motorSim.setInputVoltage(driveOutput / Constants.DriveConstants.kMaxSpeedMetersPerSecond * RobotController.getBatteryVoltage());
    }

    @Override
    public void stopMotor() {
        // TODO Auto-generated method stub
        super.stopMotor();
    }

    public void simulationPeriodic(){
        
    }


}
