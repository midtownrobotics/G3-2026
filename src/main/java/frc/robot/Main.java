package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

public final class Main {
  private Main() {
  }

  public static void main(String... args) {
    RobotBase.startRobot(Robot::new);
  }

  // public static void main(String[] args) {
  //   // Seed motor position from CANcoder absolute position
  //   Angle angle = Degrees.of(30);
  //   EasyCRTConfig easyCRTConfig = new EasyCRTConfig(() -> angle, () -> angle.times(20d / 21))
  //       .withAbsoluteEncoder1GearingStages(82, 10)
  //       .withAbsoluteEncoder2GearingStages(82, 10, 20, 21)
  //       .withMechanismRange(Degrees.of(-45), Degrees.of(315));

  //   EasyCRT easyCRT = new EasyCRT(easyCRTConfig);

  //   System.out.println(easyCRT.getAngleOptional().orElse(Degrees.zero()).in(Degrees));

  // }
}
