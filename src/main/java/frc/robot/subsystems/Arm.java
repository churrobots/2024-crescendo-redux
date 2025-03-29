// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CANMapping;

public class Arm extends SubsystemBase {

  final SparkMax m_leftArmSparkMax = new SparkMax(CANMapping.leftArmMotor, MotorType.kBrushless);
  final SparkMax m_rightArmSparkMax = new SparkMax(CANMapping.rightArmMotor, MotorType.kBrushless);
  final SparkAbsoluteEncoder m_absoluteEncoder = m_rightArmSparkMax.getAbsoluteEncoder();
  // TODO: look at robot to get the actual reduction. It might be closer to 120?
  final double kGearboxReduction = 85;
  // TODO: calculate or estimate the actual MOI and CG somehow?
  final double kArmMomentOfInertia = 0.2; // kgMetersSquared
  final double kArmCenterOfGravityLength = 0.3; // meters
  final double kArmStartingAngle = Math.toRadians(0);
  final double kArmMinAngle = Math.toRadians(0);
  final double kArmMaxAngle = Math.toRadians(90);

  public Arm() {
  }

  @Override
  public void periodic() {
  }

  //////////////////////////////////////
  // BEGIN SIMULATION

  class Simulation {
    // TODO: mirrored motors potentially can use a single SparkMaxSim (read docs)
    final public SparkMaxSim leftArmMotor = new SparkMaxSim(m_leftArmSparkMax, DCMotor.getNEO(1));
    final public SparkMaxSim rightArmMotor = new SparkMaxSim(m_rightArmSparkMax, DCMotor.getNEO(1));
    // TODO: how do we simulate two motors through TWO gearboxes?
    final public SingleJointedArmSim armPhysics = new SingleJointedArmSim(DCMotor.getNEO(2), kGearboxReduction,
        kArmMomentOfInertia, kArmCenterOfGravityLength, kArmMinAngle, kArmMaxAngle, true, kArmStartingAngle);

    public void iterate(double deltaInSeconds) {

      double leftVoltage = sim.leftArmMotor.getAppliedOutput() * sim.leftArmMotor.getBusVoltage();
      double rightVoltage = sim.rightArmMotor.getAppliedOutput() * sim.rightArmMotor.getBusVoltage();
      // TODO: should simulation assert that the voltage is opposite each other since
      // they are mirrored mounting?
      sim.armPhysics.setInput(leftVoltage, rightVoltage);
      sim.armPhysics.update(deltaInSeconds);
      double armRotationsPerSecond = sim.armPhysics.getVelocityRadPerSec() / (2.0 * Math.PI);
      double motorRotationsPerSecond = armRotationsPerSecond * kGearboxReduction;
      // TODO: since mounting is mirrored, should we be feeding mirrored inputs too?
      sim.leftArmMotor.iterate(motorRotationsPerSecond, sim.leftArmMotor.getBusVoltage(), deltaInSeconds);
      sim.rightArmMotor.iterate(motorRotationsPerSecond, sim.rightArmMotor.getBusVoltage(), deltaInSeconds);
      // TODO: is iterate() better than raw setting position/velocity?
      // double radiansFromResting = sim.armPhysics.getAngleRads() -
      // kArmStartingAngle;
      // double radiansAfterGearing = radiansFromResting * kGearboxReduction;
      // double rotationsAppliedToMotor = radiansAfterGearing * 2.0 * Math.PI;
      // sim.leftArmMotor.setPosition(rotationsAppliedToMotor);
      // sim.leftArmMotor.setVelocity(motorRotationsPerSecond);
      // sim.rightArmMotor.setPosition(rotationsAppliedToMotor);
      // sim.rightArmMotor.setVelocity(motorRotationsPerSecond);
      // sim.rightArmMotor.setPosition(rotationsAppliedToMotor);
    }
  }

  final Simulation sim = new Simulation();

  @Override
  public void simulationPeriodic() {
    // TODO: is it better to run simulation in separate thread at higher frequency?
    sim.iterate(TimedRobot.kDefaultPeriod);
  }
  // END SIMULATION
  //////////////////////////////////////
}
