// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.churrolib;

import com.revrobotics.spark.SparkBase;
// TODO: stop using PatchedSparkSim once RevLib fixes their SparkSim bugs
import com.revrobotics.spark.PatchedSparkSim;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/**
 * Simulation for a single MAXSwerve module, inspired by the "flywheel-and-arm"
 * approach taken by Quixilver:
 * https://github.com/frc604/2023-public/blob/main/FRC-2023/src/main/java/frc/quixlib/swerve/QuixSwerveModule.java#L88
 */
public class RevMAXSwerveModuleSim implements ChurroSim.SimulationEntity {

  // see inspiration here:
  // https://github.com/frc604/2023-public/blob/main/FRC-2023/src/main/java/frc/quixlib/swerve/QuixSwerveModule.java#L88

  final SparkBase m_drivingMotorController;
  final SparkBase m_turningMotorController;
  final PatchedSparkSim m_drivingMotorControllerSim;
  final PatchedSparkSim m_turningMotorControllerSim;
  final FlywheelSim m_drivingPhysicsSim;
  final SingleJointedArmSim m_turningPhysicsSim;
  final double m_wheelCircumferenceInConvertedUnits;

  public RevMAXSwerveModuleSim(
      SparkBase drivingMotorController,
      SparkBase turningMotorController,
      double turningMotorReduction,
      double drivingMotorReduction,
      double wheelCircumferenceInConvertedUnits) {

    // Start with the motor controller sims.
    m_drivingMotorController = drivingMotorController;
    m_turningMotorController = turningMotorController;
    m_drivingMotorControllerSim = new PatchedSparkSim(
        m_drivingMotorController, DCMotor.getNEO(1));
    m_turningMotorControllerSim = new PatchedSparkSim(
        m_turningMotorController, DCMotor.getNeo550(1));

    // Add the physics sims.
    // TODO: VecBuilder was weird, figure out if we need to add noise back in
    // https://github.com/frc604/2023-public/blob/main/FRC-2023/src/main/java/frc/quixlib/swerve/QuixSwerveModule.java#L88
    m_drivingPhysicsSim = new FlywheelSim(
        LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), 0.01, drivingMotorReduction),
        DCMotor.getNEO(1));
    m_turningPhysicsSim = new SingleJointedArmSim(
        DCMotor.getNeo550(1),
        turningMotorReduction,
        0.001, // MOI
        0.0, // Length (m)
        Double.NEGATIVE_INFINITY, // Min angle
        Double.POSITIVE_INFINITY, // Max angle
        false, // Simulate gravity
        Math.random() * 2 * Math.PI // random starting angle for the wheels, never know
    );

    m_wheelCircumferenceInConvertedUnits = wheelCircumferenceInConvertedUnits;
  }

  public void iterate(double timeDeltaInSeconds) {

    // Apply the motor controller's voltage to the driving wheel physics sim for
    // the number of seconds given.
    double appliedDriveVoltagePercentage = m_drivingMotorController.getAppliedOutput();
    double appliedDriveVoltage = appliedDriveVoltagePercentage * RobotController.getBatteryVoltage();
    m_drivingPhysicsSim.setInput(appliedDriveVoltage);
    m_drivingPhysicsSim.update(timeDeltaInSeconds);

    // Then we can ask the driving physics sim what the resulting velocity is, so we
    // can apply that velocity to the driving motor controller sim, so that the
    // motor controller can update its simulated sensors for distance and velocity.
    // TODO: shouldn't this be using the gearbox reduction? rather than the
    // circumference of the wheel?
    double driveVelocityInRotationsPerSecond = m_drivingPhysicsSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);
    double driveVelocityInConvertedUnits = driveVelocityInRotationsPerSecond * m_wheelCircumferenceInConvertedUnits;
    m_drivingMotorControllerSim.iterate(driveVelocityInConvertedUnits, RobotController.getBatteryVoltage(),
        timeDeltaInSeconds);

    // Now same thing for the turning motor. Apply the voltage for the number of
    // seconds given.
    double appliedTurningVoltagePercentage = m_turningMotorController.getAppliedOutput();
    double appliedTurningVoltage = appliedTurningVoltagePercentage * RobotController.getBatteryVoltage();
    m_turningPhysicsSim.setInput(appliedTurningVoltage);
    m_turningPhysicsSim.update(timeDeltaInSeconds);

    // Then we can ask the turning physics sim what the resulting velocity is, so we
    // can apply that velocity to the turning motor controller sim, so that the
    // motor controller can update its simulated sensors for distance and velocity
    // TODO: assert that the motor controller is configured for radians/sec, might
    // have to do that in the constructor
    double turningVelocityInConvertedUnits = m_turningPhysicsSim.getVelocityRadPerSec();
    m_turningMotorControllerSim.iterate(turningVelocityInConvertedUnits, RobotController.getBatteryVoltage(),
        timeDeltaInSeconds);
  }
}
