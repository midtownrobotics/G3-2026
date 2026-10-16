package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

/** An ordered, start-relative calibration itinerary loaded afresh for each run. */
public record BoardCalibrationPlan(List<Step> steps) {
  public enum Action { MOVE, ROTATE, WAIT }

  public record Step(Action action, double awayMeters, double leftMeters,
      double headingDegrees, double seconds) {
  }

  public static BoardCalibrationPlan load(Path path) throws IOException {
    return parse(new ObjectMapper().readTree(path.toFile()));
  }

  static BoardCalibrationPlan parse(JsonNode root) {
    if (root == null || !root.isObject() || root.size() != 1
        || !root.has("steps") || !root.get("steps").isArray()
        || root.get("steps").isEmpty() || root.get("steps").size() > 120) {
      throw new IllegalArgumentException("expected an array of 1-120 steps");
    }

    List<Step> steps = new ArrayList<>();
    for (JsonNode step : root.get("steps")) {
      if (!step.isObject() || !step.hasNonNull("type") || !step.get("type").isTextual()) {
        throw new IllegalArgumentException("step requires a type");
      }
      switch (step.get("type").asText()) {
        case "move" -> {
          requireFields(step, "awayMeters", "leftMeters");
          double away = number(step, "awayMeters", 0, 2.75);
          double left = number(step, "leftMeters", -1.25, 1.25);
          steps.add(new Step(Action.MOVE, away, left, 0, 0));
        }
        case "rotate" -> {
          requireFields(step, "headingDegrees");
          steps.add(new Step(Action.ROTATE, 0, 0,
              number(step, "headingDegrees", -720, 720), 0));
        }
        case "wait" -> {
          requireFields(step, "seconds");
          steps.add(new Step(Action.WAIT, 0, 0, 0, number(step, "seconds", 0.5, 10)));
        }
        default -> throw new IllegalArgumentException("unknown step type: " + step.get("type").asText());
      }
    }
    return new BoardCalibrationPlan(List.copyOf(steps));
  }

  private static void requireFields(JsonNode step, String... fields) {
    if (step.size() != fields.length + 1) {
      throw new IllegalArgumentException("unexpected or missing step fields");
    }
    for (String field : fields) {
      if (!step.has(field)) {
        throw new IllegalArgumentException("missing " + field);
      }
    }
  }

  private static double number(JsonNode step, String field, double minimum, double maximum) {
    JsonNode value = step.get(field);
    if (!value.isNumber() || !Double.isFinite(value.asDouble())
        || value.asDouble() < minimum || value.asDouble() > maximum) {
      throw new IllegalArgumentException(field + " must be between " + minimum + " and " + maximum);
    }
    return value.asDouble();
  }
}
