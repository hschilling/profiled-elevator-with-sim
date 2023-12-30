// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final int NEO_CURRENT_LIMIT = 60;

  /**
   * Example of an inner class. One can "import static [...].Constants.OIConstants.*" to gain access
   * to the constants contained within without having to preface the names with the class, greatly
   * reducing the amount of text required.
   */
  public static final class OIConstants {
    // Example: the port of the driver's controller
    public static final int kDriverControllerPort = 0;
  }
  public static final class ElevatorConstants {
    public static final double kpPos = 5.0;
    public static final double kiPos = 0.0;
    public static final double kdPos = 1.0;

    public static final double gravityCompensation = 0.05;

    public static final double simMeasurementStdDev = 0.0;  // Meters

    public static final int ELEVATOR_MOTOR_ID = 7;
    public static final int ELEVATOR_SLEW = 5;

    //elevator min and max heights in meters
    public static final double MIN_ELEVATOR_HEIGHT = 0.0;
    public static final double MAX_ELEVATOR_HEIGHT = 0.75;

    //27 inches per 41.951946 encoder counts
    public static final double METERS_PER_REVOLUTION = Units.inchesToMeters(27) / 41.951946;

    //can be 1 inch off from goal setpoints and still considered at goal; made higher so placeConeOnNode cmd in auton will execute
    public static final double HEIGHT_TOLERANCE = Units.inchesToMeters(0.5);
}
}
