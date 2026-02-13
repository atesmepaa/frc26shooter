// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import java.util.List;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
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
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class ModuleConstants {
    public static final double wheelDiameterMeters = Units.inchesToMeters(3.0);
    public static final double wheelRadius = Units.inchesToMeters(wheelDiameterMeters);

    public static final double drivingMotorReduction = 5.14; 
        
    public static final double turningMotorReduction = 41.25; 

    public static final double driveWheelFreeSpeedRps = (5676.0 / 60.0) / drivingMotorReduction;

    public static final boolean turningEncoderInverted = true;
        
    public static final int drivingMotorCurrentLimit = 40;
    public static final int turningMotorCurrentLimit = 20;
  }

  public static class DriveContants {
    public static final int flDriveCanID = 3;
    public static final int flAngleCanID = 4;

    public static final int frDriveCanID = 1;
    public static final int frAngleCanID = 2;

    public static final int blDriveCanID = 5;
    public static final int blAngleCanID = 6;

    public static final int brDriveCanID = 7;
    public static final int brAngleCanID = 8;

    public static final double mass = 13.0;
    public static final double MOI = 5.0;

    public static final double moduleOffsetMeters = Units.inchesToMeters(26.06) / 2.0;

// PathPlanner'ın beklediği liste (Sıralama: FL, FR, BL, BR)
    public static final List<Translation2d> moduleOffsets = List.of(
        new Translation2d(moduleOffsetMeters, moduleOffsetMeters),   // Front Left (+X, +Y)
        new Translation2d(moduleOffsetMeters, -moduleOffsetMeters),  // Front Right (+X, -Y)
        new Translation2d(-moduleOffsetMeters, moduleOffsetMeters),  // Back Left (-X, +Y)
        new Translation2d(-moduleOffsetMeters, -moduleOffsetMeters)  // Back Right (-X, -Y)
    );




    
    public static final double flChassisAngularOffset = Math.toRadians(274);
    public static final double frChassisAngularOffset = Math.toRadians(250);
    public static final double blChassisAngularOffset = Math.toRadians(65);
    public static final double brChassisAngularOffset = Math.toRadians(72);

    public static final double trackWidth = Units.inchesToMeters(26.06); // Sağ-sol tekerlek arası
    public static final double wheelBase = Units.inchesToMeters(21.34);  // Ön-arka tekerlek arası

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2, trackWidth / 2),   // Front Left
            new Translation2d(wheelBase / 2, -trackWidth / 2),  // Front Right
            new Translation2d(-wheelBase / 2, trackWidth / 2),  // Rear Left
            new Translation2d(-wheelBase / 2, -trackWidth / 2)  // Rear Right
        );

    public static final double maxSpeedMetersPerSecond = 10; // Yaklaşık 14-15 ft/s
    public static final double maxAngularSpeed = (10 * Math.PI);  // Saniyede 1 tam tur (radyan)


        // --- Gyro Ayarı ---
        // NavX montaj yönüne göre gerekirse true yapın
    public static final boolean gyroReversed = false;
  }

  public static final class OIConstants {
    // Kumandanın takılı olduğu USB portu (Driver Station'da görülür)
    public static final int primaryPort = 0;

    // Joystick Ölü Bölgesi (Deadband)
    // 0.05 ile 0.1 arası idealdir. 
    // Joystick eski ve gevşekse bu değeri biraz daha artırabilirsin.
    public static final double driveDeadband = 0.1;

  }
}
