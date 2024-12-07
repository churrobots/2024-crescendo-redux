// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.churrolib;

import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Robot;

/**
 * Simulation for a generic swerve drive, inspired by Quixilver's approach.
 * https://github.com/frc604/2023-public/blob/main/FRC-2023/src/main/java/frc/quixlib/swerve/QuixSwerveModule.java#L88
 */
public class GenericSwerveSim implements SimulationEntity {
  final Pigeon2 m_gyro;
  final Supplier<ChassisSpeeds> m_chassisSpeedsSupplier;
  final Field2d m_fieldViz;
  Pose2d m_simPose;

  public GenericSwerveSim(
      Pigeon2 gyro,
      Supplier<ChassisSpeeds> chassisSpeedsSupplier,
      Field2d fieldViz) {
    m_gyro = gyro;
    m_chassisSpeedsSupplier = chassisSpeedsSupplier;
    m_fieldViz = fieldViz;
    m_simPose = new Pose2d();
  }

  /**
   * Call this whenever you want the sim to assume an initial pose,
   * for example if your autonomous command sets its pose. You should
   * definitely NOT call this when recalibrating drivetrain mid-match.
   * 
   * @param newPose - the new pose
   */
  public void assumeRealWorldPose(Pose2d newPose) {
    m_simPose = newPose;
  }

  public void iterate(double timeDeltaInSeconds) {
    // Update pose by integrating ChassisSpeeds.
    final ChassisSpeeds currentChassisSpeeds = m_chassisSpeedsSupplier.get();
    m_simPose = m_simPose.transformBy(
        new Transform2d(
            new Translation2d(
                currentChassisSpeeds.vxMetersPerSecond * timeDeltaInSeconds,
                currentChassisSpeeds.vyMetersPerSecond * timeDeltaInSeconds),
            new Rotation2d(currentChassisSpeeds.omegaRadiansPerSecond * timeDeltaInSeconds)));

    // Update gyro based on sim pose.
    Pigeon2SimState gyroSimState = m_gyro.getSimState();
    gyroSimState.setRawYaw(m_simPose.getRotation().getDegrees());

    // Finally, update the visualization.
    m_fieldViz.setRobotPose(m_simPose);
  }
}
