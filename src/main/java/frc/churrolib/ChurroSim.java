// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.churrolib;

/**
 * Base interface for all simulations, so that clients can simply call
 * iterate() without knowing about the concrete implementation.
 */
public interface ChurroSim {
  public void iterate(double timeDeltaInSeconds);
}
