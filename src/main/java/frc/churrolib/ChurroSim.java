// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.churrolib;

import java.util.ArrayList;
import java.util.List;

/**
 * This singleton collects all simulation objects in a single place so that we
 * can update them on whatever frequency we like. Instead of overriding
 * `simulationPeriodic()` in your subsystems, you just use `ChurroSim.register`.
 */
public class ChurroSim {

  /**
   * Base interface for all simulations, so that clients can simply call
   * iterate() without knowing about the concrete implementation.
   */
  public interface SimulationEntity {
    public void iterate(double timeDeltaInSeconds);
  }

  static ChurroSim instance;
  final List<SimulationEntity> m_entities;

  private ChurroSim() {
    m_entities = new ArrayList<>();
  }

  private static ChurroSim getInstance() {
    if (instance == null) {
      instance = new ChurroSim();
    }
    return instance;
  }

  private static List<SimulationEntity> getEntities() {
    return getInstance().m_entities;
  }

  public static void register(SimulationEntity entity) {
    getEntities().add(entity);
  }

  public static void iterate(double timeDeltaInSeconds) {
    for (SimulationEntity entity : getEntities()) {
      entity.iterate(timeDeltaInSeconds);
    }
  }
}
