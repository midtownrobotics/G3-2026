package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import java.io.IOException;
import java.nio.file.Path;

import org.junit.jupiter.api.Test;

import com.fasterxml.jackson.databind.ObjectMapper;
import frc.robot.commands.BoardCalibrationPlan.Action;

class BoardCalibrationPlanTest {
  private static final ObjectMapper kMapper = new ObjectMapper();

  @Test
  void deployedPlanHasOrderedActions() throws IOException {
    var plan = BoardCalibrationPlan.load(Path.of("src/main/deploy/board_calibration.json"));
    assertEquals(16, plan.steps().size());
    assertEquals(Action.MOVE, plan.steps().get(0).action());
    assertEquals(1.5, plan.steps().get(6).awayMeters());
    assertEquals(0.35, plan.steps().get(6).leftMeters());
    assertEquals(-0.35, plan.steps().get(12).leftMeters());
    assertEquals(Action.WAIT, plan.steps().get(15).action());
  }

  @Test
  void preservesConfiguredRotationOrder() throws IOException {
    var plan = BoardCalibrationPlan.parse(kMapper.readTree("""
        {"steps":[{"type":"rotate","headingDegrees":360},
                  {"type":"wait","seconds":5},
                  {"type":"rotate","headingDegrees":-30}]}
        """));
    assertEquals(360, plan.steps().get(0).headingDegrees());
    assertEquals(-30, plan.steps().get(2).headingDegrees());
  }

  @Test
  void rejectsMissingFieldsAndUnsafePositions() throws IOException {
    assertThrows(IllegalArgumentException.class,
        () -> BoardCalibrationPlan.parse(kMapper.readTree("""
            {"steps":[{"type":"move","awayMeters":3,"leftMeters":0}]}
            """)));
    assertThrows(IllegalArgumentException.class,
        () -> BoardCalibrationPlan.parse(kMapper.readTree("""
            {"steps":[{"type":"move","awayMeters":1}]}
            """)));
    assertThrows(IllegalArgumentException.class,
        () -> BoardCalibrationPlan.parse(kMapper.readTree("""
            {"steps":[{"type":"wait","seconds":5,"headingDegrees":180}]}
            """)));
    assertThrows(IllegalArgumentException.class,
        () -> BoardCalibrationPlan.parse(kMapper.readTree("{\"steps\":[]}")));
  }
}
