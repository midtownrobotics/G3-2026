package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Represents radial coordinates (r, theta) and their derivatives.
 * Used for hub-centric tracking calculations.
 */
public class RadialCoordinates {
  /** Radial distance from the hub (meters) */
  public final double r;

  /** Angle from the hub to the position (radians, counter-clockwise from +X axis) */
  public final Rotation2d theta;

  /** Radial velocity (meters/second, positive = away from hub) */
  public final double rDot;

  /** Tangential velocity (meters/second, positive = counter-clockwise) */
  public final double thetaDot;

  /** Radial acceleration (meters/second^2, positive = away from hub) */
  public final double rDotDot;

  /** Tangential acceleration (meters/second^2, positive = counter-clockwise) */
  public final double thetaDotDot;

  public RadialCoordinates(double r, Rotation2d theta) {
    this(r, theta, 0.0, 0.0, 0.0, 0.0);
  }

  public RadialCoordinates(double r, Rotation2d theta, double rDot, double thetaDot) {
    this(r, theta, rDot, thetaDot, 0.0, 0.0);
  }

  public RadialCoordinates(double r, Rotation2d theta, double rDot, double thetaDot,
                          double rDotDot, double thetaDotDot) {
    this.r = r;
    this.theta = theta;
    this.rDot = rDot;
    this.thetaDot = thetaDot;
    this.rDotDot = rDotDot;
    this.thetaDotDot = thetaDotDot;
  }

  @Override
  public String toString() {
    return String.format("RadialCoordinates(r=%.3f, theta=%.3f°, rDot=%.3f, thetaDot=%.3f, rDotDot=%.3f, thetaDotDot=%.3f)",
                        r, theta.getDegrees(), rDot, thetaDot, rDotDot, thetaDotDot);
  }
}
