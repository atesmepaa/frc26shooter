// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import java.util.List;

import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
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

  // ----------------------------
  // SHOOTER / HOOD constants
  // ----------------------------
  public static final class ShooterConstants {
    // --- CAN IDs (NEO SparkMax) ---
    public static final int kShooterNeo1ID = 11;
    public static final int kShooterNeo2ID = 12;
    public static final int kFeederNeoID   = 13;

    // --- PWM ports (VictorSP) ---
    public static final int kFeederCimPort   = 0;
    public static final int kTransferCimPort = 1;
    public static final int kIntakeCimPort   = 2;

    // --- Hood PWM + Analog ---
    public static final int kHoodMotorPort = 3;
    public static final int kPotentiometerPort = 0; // AnalogInput port (roboRIO)

    // Flywheel velocity PID
    public static final double kFlywheelP = 0.0002;
    public static final double kFlywheelI = 0.0;
    public static final double kFlywheelD = 0.0;

    // Shoot speeds
    public static final double kFeederShootSpeed = 0.65;
    public static final double kIntakeShootSpeed = 0.75;

    // Distance -> RPM table
    public static final InterpolatingDoubleTreeMap kRpmTable = new InterpolatingDoubleTreeMap();
    static {
      kRpmTable.put(1.5, 3200.0);
      kRpmTable.put(3.0, 4200.0);
      kRpmTable.put(5.0, 5200.0);
    }

    // SparkMax idle mode enum’ını buradan yönetmek istersen:
    public static final SparkMaxConfig.IdleMode kFlywheelIdleMode = SparkMaxConfig.IdleMode.kCoast;
    public static final SparkMaxConfig.IdleMode kFeederIdleMode   = SparkMaxConfig.IdleMode.kBrake;

    // Hood PID
    public static final double kHoodP = 0.15;
    public static final double kHoodI = 0.0;
    public static final double kHoodD = 0.0;

    public static final double kHoodToleranceDeg = 2.0; // daha hassas: 1.0 veya 0.7

// Pot kalibrasyonu (robot üzerinde ölç!)
    public static final double kPotMinV = 0.55;   // hood en aşağıdayken okunan voltaj
    public static final double kPotMaxV = 4.35;   // hood en yukarıdayken okunan voltaj

// Hood mekanik açı sınırları
    public static final double kHoodMinDeg = 5.0;
    public static final double kHoodMaxDeg = 85.0;

// PID output shaping
    public static final double kHoodMaxOutput = 0.6;      // çok hızlı vurmasın
    public static final double kHoodOutputDeadband = 0.02; // titremeyi keser
    public static final double kHoodKs = 0.06;            // stiction (0.04–0.10 arası)

// Table clamp + fallback
    public static final double kHoodMinTableM = 1.5;
    public static final double kHoodMaxTableM = 5.0;
    public static final double kHoodSafeDeg = 20.0;

    public static final int kFlywheelCurrentLimit = 60;
    public static final int kFeederCurrentLimit = 40;
    public static final double kFlywheelRamp = 0.2;

    public static final double kMaxFlywheelRPM = 6000;
    public static final double kFlywheelReadyToleranceRPM = 150;

    public static final boolean kFeederCimInverted = false;
    public static final boolean kTransferInverted = false;
    public static final boolean kIntakeInverted = false;

    public static final double kRpmMinTableM = 1.5;
    public static final double kRpmMaxTableM = 5.0;
    public static final double kSafeRpm = 2500; // tablo patlarsa bile güvenli RPM
  }

  // ----------------------------
  // CLIMB PNEUMATIC (single solenoid)
  // ----------------------------
  public static final class ClimbConstants {
    // REV PH mi CTRE PCM mi? Sizde hangisiyse onu seç
    public static final PneumaticsModuleType kModuleType = PneumaticsModuleType.REVPH;
    public static final int kSolenoidChannel = 0;

    // Eğer kompresör kontrol edecekseniz:
    public static final int kModuleId = 1; // PH module id (emin değilsen 1/0 kontrol edin)
    public static final double kMinPressurePSI = 80.0;
  }

  // ----------------------------
  // LED
  // ----------------------------
    public static final class LedConstants {
    public static final int kPwmPort = 9;
    public static final int kLength = 60;
  }
}
