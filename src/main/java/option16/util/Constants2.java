/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package option16.util;

/**
 * Add your docs here.
 */
public class Constants2 {

    //ALL UNITS ARE IN FEET
    public static double wheelRadius = 0.25;
    public static double maxVelocity = 1.2;
    public static double maxAcceleration = 2;
    public static double wheelBaseWidth = (double)20 / 12;
    public static double kP = 1;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 0;

    public static double kV = 1; //needs tuning
    public static int ticksPerRevolution = 4096;

    public static int lookaheadDistance = 6;
}
