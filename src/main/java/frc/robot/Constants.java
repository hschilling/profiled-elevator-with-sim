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
    // public static final double kpPos = 5.0;
    // public static final double kpPos = 1.0; // works but ringing and overshoot for velocity
    public static final double kpPos = 10.0; 
    public static final double kiPos = 0.0;
    public static final double kdPos = 1.0; // If you make this too big you eventually get oscillations. For the sim, with P=10.0, 2.0 caused oscillations! But one was great!
// just a small overshoot of 1/200.0 of the set point and then straight to the correct value 
    // for PIDProfiled System
    // PIDF tuned values:
    // feedforward does not make sense for PID alone, but does for ProfiledPID since the 
    // profile sets the desired speeds. There is an approximate linear relationship between the
    // motor controller input (-1 to 1) or (-12 to 12) and the velocity. 
    // To get that value, set the speed of the motor (-1 to 1) and then see what the velocity is for multiple values
    // elevator velocity (m/s)    motor speed
    // 0.238                     0.2
    // 0.745                     0.5
    // 1.589                     1.0
    //  fitting a line using desmos to that gives slope of m=0.617753 ( r squared of 0.997 and b = 0.027785)
    public static final double feedForward = 0.617753;

    // this helps maintain the elevator at the location desired
    public static final double gravityCompensation = 0.059;  // for the sim, 0.059 works really well.

    // Trapezoidal profile constants and variables
    // this max velocity of 0.25 results in a trapezoidal profile in sim for common position values
    // if you set it to something like 1.25, just get triangular
    public static final double max_vel = 0.25; // m/s  - 
    //
    // public static final double max_accel = 2.50; // m/s/s //0.4
    public static final double max_accel = 2.50; // m/s/s //0.4
    // end for PIDProfiled System

    // - just to give some randomness to the data, as there would be in real life
    // can set it to zero. Just makes the plots easier to understand
    public static final double simMeasurementStdDev = 0.0;  // Meters 

    public static final int ELEVATOR_MOTOR_ID = 7;

    //elevator min and max heights in meters
    public static final double MIN_ELEVATOR_HEIGHT = 0.0;
    public static final double MAX_ELEVATOR_HEIGHT = 0.75;

    //27 inches per 41.951946 encoder counts
    public static final double METERS_PER_REVOLUTION = Units.inchesToMeters(27) / 41.951946;

    //can be 1 inch off from goal setpoints and still considered at goal; made higher so placeConeOnNode cmd in auton will execute
    public static final double HEIGHT_TOLERANCE = Units.inchesToMeters(0.5);
    public static final double VELOCITY_TOLERANCE = max_vel / 50.0;
}
}
