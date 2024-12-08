// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {

  static final class Constants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.1;
  }

  final XboxController driverController = new XboxController(Constants.kDriverControllerPort);

  Drivetrain drivetrain = new Drivetrain();

  final Command fastDrive = new RunCommand(
      () -> drivetrain.drive(
          -MathUtil.applyDeadband(driverController.getLeftY(),
              Constants.kDriveDeadband),
          -MathUtil.applyDeadband(driverController.getLeftX(),
              Constants.kDriveDeadband),
          -MathUtil.applyDeadband(driverController.getRightX(),
              Constants.kDriveDeadband),
          true, false),
      drivetrain);

  public RobotContainer() {
    drivetrain.setDefaultCommand(fastDrive);
  }

  public Command getAutonomousCommand() {
    var doNothing = new InstantCommand();
    return doNothing;
  }

}