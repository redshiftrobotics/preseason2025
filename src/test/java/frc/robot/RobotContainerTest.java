package frc.robot;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

/** Tests for RobotContainer */
public class RobotContainerTest {

  @Test
  @DisplayName("Create robot container")
  public void createRobotContainer() {
    // Instantiate RobotContainer
    new RobotContainer();
  }

  /**
   * @author Aceius E.
   */
  @Test
  @Deprecated
  @DisplayName("Ensure Big Brother is not watching")
  public void orwellianSecurityTest() {
    Assertions.assertNotEquals(2 + 2, 5);
  }
}
