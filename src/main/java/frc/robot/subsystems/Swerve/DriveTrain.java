
package frc.robot.subsystems.Swerve;



import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveContants;

public class DriveTrain extends SubsystemBase{

    private final Field2d m_field = new Field2d();
    
    private final SwerveModule frontLeft = new SwerveModule(DriveContants.flDriveCanID, DriveContants.flAngleCanID, DriveContants.flChassisAngularOffset, "fl",0.01958944275, 3.3008210659);
    private final SwerveModule frontRight = new SwerveModule(DriveContants.frDriveCanID, DriveContants.frAngleCanID, DriveContants.frChassisAngularOffset, "fr", 0.01469208206,3.29592370986);
    private final SwerveModule backLeft = new SwerveModule(DriveContants.blDriveCanID, DriveContants.blAngleCanID, DriveContants.blChassisAngularOffset, "bl", 0.01958944275,3.3008210659);
    private final SwerveModule backRight = new SwerveModule(DriveContants.brDriveCanID, DriveContants.brAngleCanID, DriveContants.brChassisAngularOffset, "br",0.01469208206,3.29102635383);

    private final AHRS navx = new AHRS(NavXComType.kMXP_SPI);

    private final PIDController turnPID= new PIDController(0.01,0,0);

    // private final SwerveDriveOdometry odometry;

    private final StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault()
        .getStructTopic("RobotPoseStruct", Pose2d.struct)
        .publish();

    private final StructArrayPublisher<SwerveModuleState> m_publisher = 
        NetworkTableInstance.getDefault()
            .getStructArrayTopic("SwerveStates", SwerveModuleState.struct)
            .publish();

    private final SwerveDrivePoseEstimator poseEstimator;

    public DriveTrain(){

        zeroHeading();

        poseEstimator =
            new SwerveDrivePoseEstimator(
                DriveContants.kDriveKinematics,     // SwerveDriveKinematics
                getRotation2d(),              // Rotation2d
                getModulePositions(),           // SwerveModulePosition[]
                new Pose2d()                    // başlangıç pozu
            );

        
        poseEstimator.setVisionMeasurementStdDevs(
            VecBuilder.fill(0.5, 0.5, 9999)
        );

        turnPID.enableContinuousInput(-180, 180);
        /* 
        odometry = new SwerveDriveOdometry(
        DriveContants.kDriveKinematics,
        getRotation2d(),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        });
        */
        SmartDashboard.putData("Field", m_field);


        // 1. Değişkeni tanımla
        RobotConfig config;

        try {
            // 2. GUI ayarlarını yüklemeyi dene
            config = RobotConfig.fromGUISettings();

            // 3. EĞER ayarlar başarıyla yüklendiyse AutoBuilder'ı yapılandır
            AutoBuilder.configure(
                this::getPose, //Robot pose supplier
                this::resetOdometry, //Method to reset odometry
                this::getRobotRelativeSpeeds, //ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> driveRobotRelative(speeds), //Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                new PPHolonomicDriveController(
                    new PIDConstants(5.65, 0.0, 0.0), //Translation PID constants
                    new PIDConstants(5.65, 0.0, 0.0) //Rotation PID constants
                ),
                config, //The robot configuration
                () -> {
                //Boolean supplier that controls when the path will be mirrored for the red alliance
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
                },
                this //Reference to this subsystem to set requirements
            );
            } catch (Exception e) {
                // Ayarlar yüklenemezse buraya düşer, hata basar ama robot çökmez
                DriverStation.reportError("PathPlanner config yüklenemedi usta!: " + e.getMessage(), true);
            }
        
    }

    public void addVisionMeasurement(
            Pose2d visionPose,
            double timestamp,
            Matrix<N3, N1> stdDevs
    ) {
        poseEstimator.addVisionMeasurement(
            visionPose,
            timestamp,
            stdDevs
        );
    }

    

    private SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        };
    }

    

    public void configAuto() {
        RobotConfig config;
        try {
          config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
          e.printStackTrace();
          return;
        }
    
        //Configure AutoBuilder last
        AutoBuilder.configure(
            this::getPose, //Robot pose supplier
            this::resetOdometry, //Method to reset odometry
            this::getRobotRelativeSpeeds, //ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> driveRobotRelative(speeds), //Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new PPHolonomicDriveController(
                new PIDConstants(5.65, 0.0, 0.0), //Translation PID constants
                new PIDConstants(5.65
                
                
                , 0.0, 0.0) //Rotation PID constants
            ),
            config, //The robot configuration
            () -> {
              //Boolean supplier that controls when the path will be mirrored for the red alliance
              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this //Reference to this subsystem to set requirements
        );
      }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        // Şasi hızlarını modül durumlarına çeviriyoruz
        SwerveModuleState[] states = DriveContants.kDriveKinematics.toSwerveModuleStates(speeds);
        setModuleStates(states);
    }
    
    // Mevcut hızı ChassisSpeeds olarak döndüren metod (PathPlanner geri bildirim için kullanır)
    public ChassisSpeeds getRobotRelativeSpeeds() {
        return DriveContants.kDriveKinematics.toChassisSpeeds(
            new SwerveModuleState[] {
                frontLeft.getState(), frontRight.getState(),
                backLeft.getState(), backRight.getState()
            }
        );
    }

    @Override
    public void periodic() {
        // Odometry güncelleme (Robotun sahadaki yerini hesaplar)
        /*

        odometry.update(
            getRotation2d(),
            new SwerveModulePosition[] {
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
            });
        
        */

        poseEstimator.update(getRotation2d(), getModulePositions());
        
        posePublisher.set(getPose());

        frontLeft.logCalibrationData();
        frontLeft.updateCalibration();

        frontRight.logCalibrationData();    
        frontRight.updateCalibration();

        backLeft.logCalibrationData();
        backLeft.updateCalibration();

        backRight.logCalibrationData();
        backRight.updateCalibration();

        m_publisher.set(new SwerveModuleState[] {
            frontLeft.getState(),
            frontRight.getState(),
            backLeft.getState(),
            backRight.getState()
        });
    
        
        SmartDashboard.putNumber("fl velocity", frontLeft.getVelocity());
        SmartDashboard.putNumber("fr velocity", frontRight.getVelocity());
        SmartDashboard.putNumber("bl velocity", backLeft.getVelocity());
        SmartDashboard.putNumber("br velocity", backRight.getVelocity());

        
        // SmartDashboard üzerinden tekerlek açılarını ve gyro bilgisini takip et
        SmartDashboard.putNumber("FL Angle Deg", Math.toDegrees(frontLeft.getAngle()));
        SmartDashboard.putNumber("FR Angle Deg", Math.toDegrees(frontRight.getAngle()));
        SmartDashboard.putNumber("RL Angle Deg", Math.toDegrees(backLeft.getAngle()));
        SmartDashboard.putNumber("RR Angle Deg", Math.toDegrees(backRight.getAngle()));
        SmartDashboard.putNumber("Robot Heading", getHeading());
    }

    public void drive(double xSpeed, double ySpeed, double rotation, boolean fieldRelative) {
        double xSpeedDelivered = xSpeed * DriveContants.maxSpeedMetersPerSecond;
        double ySpeedDelivered = ySpeed * DriveContants.maxSpeedMetersPerSecond;
        double rotDelivered = rotation * DriveContants.maxAngularSpeed;

        var swerveModuleStates = DriveContants.kDriveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, getRotation2d())
                : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
        
        setModuleStates(swerveModuleStates);
    }

    public void updateTelemetry() {
        double[] swerveStates = new double[] {
            frontLeft.getAngle(),  frontLeft.getVelocity(),
            frontRight.getAngle(), frontRight.getVelocity(),
            backLeft.getAngle(),   backLeft.getVelocity(),
            backRight.getAngle(),  backRight.getVelocity()
        };
    
        // Bu isimle (SwerveStates) gönderiyoruz
        SmartDashboard.putNumberArray("SwerveStates", swerveStates);
    }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    // Tekerlek hızlarını maksimum hıza oranla (Desaturate)
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveContants.maxSpeedMetersPerSecond);
    
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
  }

    public Rotation2d getRotation2d() {
        //return Rotation2d.fromDegrees(DriveContants.gyroReversed ? navx.getAngle() : -navx.getAngle());

        return navx.getRotation2d();
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void resetOdometry(Pose2d pose) {
        /*
        odometry.resetPosition(getRotation2d(), 
            new SwerveModulePosition[] {
                frontLeft.getPosition(), frontRight.getPosition(),
                backLeft.getPosition(), backRight.getPosition()
            }, pose);
        */
        poseEstimator.resetPosition(getRotation2d(), getModulePositions(), pose);
    }


    public void zeroHeading() {
        navx.reset();
        
    }

    public double getHeading() {
        //return Math.IEEEremainder(getRotation2d().getDegrees(), 360);
        return getRotation2d().getDegrees();
    }

    public void resetEncoders() {
        frontLeft.resetEncoders();
        frontRight.resetEncoders();
        backLeft.resetEncoders();
        backRight.resetEncoders();
    }
    
      /** Tekerlekleri X şeklinde kilitler (Defans modu) */
      public void setX() {
        frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
      }


    public void driveAtTarget(double xSpeed, double ySpeed) {
    // 1. Hedef Koordinatlar
    double targetX = 4.625594;
    double targetY = 4.034536;

    Pose2d currentPose = getPose();

    // 2. Hedef vektörü (field-relative)
    double dx = targetX - currentPose.getX();
    double dy = targetY - currentPose.getY();

    // 3. Robot frame'ine çevir (field -> robot)
    Rotation2d robotRot = currentPose.getRotation();

    double localX =  dx * robotRot.getCos() + dy * robotRot.getSin();
    double localY = -dx * robotRot.getSin() + dy * robotRot.getCos();

    // -------------------------------------------------
    // 🔥 AIM OFFSET HESABI
    // -------------------------------------------------

    // (A) Konuma bağlı küçük stabil ofset
    double kPosAim = 2.0; // derece / metre
    double positionalOffset =
        MathUtil.clamp(localY * kPosAim, -3.0, 3.0);

    // (B) Hıza bağlı lead (ASIL OLAY)
    double lateralSpeed =
        ySpeed * DriveContants.maxSpeedMetersPerSecond; // m/s

    double kVelocityAim = 3.0; // derece / (m/s)
    double velocityOffset =
        MathUtil.clamp(lateralSpeed * kVelocityAim, -6.0, 6.0);

    // (C) Toplam ofset
    double angleOffset = positionalOffset + velocityOffset;

    // Hedef arkadaysa saçmalamasın
    if (localX < 0.0) {
        angleOffset = 0.0;
    }

    // 4. Hedef açısı + ofset
    double angleToTarget =
        Math.toDegrees(Math.atan2(dy, dx)) + angleOffset;

    // 5. PID ile dönüş
    double angleError =
        Math.IEEEremainder(getHeading() - angleToTarget, 360);

    double rotationOutput = turnPID.calculate(angleError, 0);
    rotationOutput = MathUtil.clamp(rotationOutput, -0.5, 0.5);

    // Deadband (titreşim kesici)
    if (Math.abs(angleError) < 0.5) {
        rotationOutput = 0;
    }

    // 6. Şasi hızları
    double xVel = xSpeed * DriveContants.maxSpeedMetersPerSecond;
    double yVel = ySpeed * DriveContants.maxSpeedMetersPerSecond;
    double rVel = rotationOutput * DriveContants.maxAngularSpeed;

    // 7. Field-relative sürüş
    ChassisSpeeds chassisSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            xVel,
            yVel,
            rVel,
            getRotation2d()
        );

    // 8. Modüllere gönder
    var states =
        DriveContants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(
        states,
        DriveContants.maxSpeedMetersPerSecond
    );

    setModuleStates(states);
}


    public void lockFront(double xSpeed, double ySpeed) {
        double error = Math.IEEEremainder(getHeading(), 360);
        double rotationOutput = turnPID.calculate(error, 0);
    
        // 1. SARSINTI ENGELLEYİCİ: 
        // Eğer robot hareket halindeyse (joystick basılıysa), dönüş gücünü kısıtla.
        // Bu sayede "ileri fırlama" hissi azalır.
        double moveThreshold = 0.1;
        if (Math.abs(xSpeed) > moveThreshold || Math.abs(ySpeed) > moveThreshold) {
            rotationOutput = MathUtil.clamp(rotationOutput, -0.25, 0.25); // Daha nazik dönüş
        }
    
        if (Math.abs(error) < 2.0) rotationOutput = 0;
    
        double xVel = xSpeed * DriveContants.maxSpeedMetersPerSecond;
        double yVel = ySpeed * DriveContants.maxSpeedMetersPerSecond;
        double rVel = rotationOutput * DriveContants.maxAngularSpeed;
    
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xVel, yVel, rVel, getRotation2d()
        );
    
        var states = DriveContants.kDriveKinematics.toSwerveModuleStates(speeds);
    
        // 2. HIZLARI DENGELE (Sarsıntıyı bitiren asıl satır)
        SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveContants.maxSpeedMetersPerSecond);
    
        setModuleStates(states);
    }
    
    // Kilitleyeceğimiz açıyı set etmek için yardımcı bir fonksiyon
    public void lockBack(double xSpeed, double ySpeed) {
        double error = Math.IEEEremainder(getHeading(), 360);
        double rotationOutput = turnPID.calculate(error, 180);
    
        // 1. SARSINTI ENGELLEYİCİ: 
        // Eğer robot hareket halindeyse (joystick basılıysa), dönüş gücünü kısıtla.
        // Bu sayede "ileri fırlama" hissi azalır.
        double moveThreshold = 0.1;
        if (Math.abs(xSpeed) > moveThreshold || Math.abs(ySpeed) > moveThreshold) {
            rotationOutput = MathUtil.clamp(rotationOutput, -0.25, 0.25); // Daha nazik dönüş
        }
    
        if (Math.abs(error) < 2.0) rotationOutput = 0;
    
        double xVel = xSpeed * DriveContants.maxSpeedMetersPerSecond;
        double yVel = ySpeed * DriveContants.maxSpeedMetersPerSecond;
        double rVel = rotationOutput * DriveContants.maxAngularSpeed;
    
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xVel, yVel, rVel, getRotation2d()
        );
    
        var states = DriveContants.kDriveKinematics.toSwerveModuleStates(speeds);
    
        // 2. HIZLARI DENGELE (Sarsıntıyı bitiren asıl satır)
        SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveContants.maxSpeedMetersPerSecond);
    
        setModuleStates(states);
    }
}

