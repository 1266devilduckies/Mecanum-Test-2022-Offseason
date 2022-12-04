// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
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
    public static final int CANID_motorBR = 0;
    public static final int CANID_motorFR = 1;
    public static final int CANID_motorFL = 2;
    public static final int CANID_motorBL = 3;

    //vector offset from center of bot
    public static final Translation2d BRWheelOffsetMeters = new Translation2d(Units.inchesToMeters(-11),Units.inchesToMeters(-4.7));
    public static final Translation2d BLWheelOffsetMeters = new Translation2d(Units.inchesToMeters(11),Units.inchesToMeters(-4.7));
    public static final Translation2d FRWheelOffsetMeters = new Translation2d(Units.inchesToMeters(11),Units.inchesToMeters(4.7));
    public static final Translation2d FLWheelOffsetMeters = new Translation2d(Units.inchesToMeters(-11),Units.inchesToMeters(4.7));

    //uncollected data
    public static final double kS_drivetrain = 0.1;
    public static final double kV_drivetrain = 0.05;
    public static final double kA_drivetrain = 0.02;
}
