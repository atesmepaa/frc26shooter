package frc.robot.subsystems.Swerve;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;

import com.revrobotics.spark.SparkAnalogSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;


import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule extends SubsystemBase{
    private final SparkMax driveMotor;
    private final SparkMax angleMotor;

    private final RelativeEncoder driveEncoder;
    private final SparkAnalogSensor angleEncoder;

    private final SparkClosedLoopController  drivePID;
    private final SparkClosedLoopController  anglePID;

    double kV = 1/ ModuleConstants.driveWheelFreeSpeedRps;

    SimpleMotorFeedforward ff =
    new SimpleMotorFeedforward(0.0, kV, 0.0);
    
    private double CAO = 0;

    // Başlangıçta min'i en yüksek, max'ı en düşük değere ayarla ki ilk okumada güncellensinler
    private double calibratedMinVolt = 3.3;
    private double calibratedMaxVolt = 0.0;

    private final String moduleID;

    public SwerveModule(int driveCanID, int angleCanID, double chassisAngularOffset, String m_moduleID,double minV, double maxV){
        this.moduleID = m_moduleID;
        driveMotor = new SparkMax(driveCanID, MotorType.kBrushless);
        angleMotor = new SparkMax(angleCanID, MotorType.kBrushed);

        driveEncoder = driveMotor.getEncoder();

        angleEncoder = angleMotor.getAnalog();

        drivePID = driveMotor.getClosedLoopController();
        anglePID = angleMotor.getClosedLoopController();
        
        
        driveMotor.configure(Configs.SwerveXS.driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        SparkMaxConfig customAngleConfig = new SparkMaxConfig();
        customAngleConfig.apply(Configs.SwerveXS.angleConfig);

        double customAngleFactor = (2 * Math.PI) / (maxV - minV);
        
        customAngleConfig.analogSensor
            .positionConversionFactor(customAngleFactor)
            .velocityConversionFactor(customAngleFactor / 60.0);
            

        customAngleConfig.closedLoop
            .positionWrappingEnabled(true) // Artık güvenle açabilirsin
            .positionWrappingInputRange(0, 2 * Math.PI);

        
        angleMotor.configure(customAngleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        
        CAO = chassisAngularOffset;
        driveEncoder.setPosition(0);

        this.calibratedMinVolt = minV;
        this.calibratedMaxVolt = maxV;
    }

    

    public SwerveModuleState getState() {
    return new SwerveModuleState(driveEncoder.getVelocity(),
        Rotation2d.fromRadians(getAngle() - CAO));
    }

    public SwerveModulePosition getPosition() {
      return new SwerveModulePosition(
          driveEncoder.getPosition(),
          Rotation2d.fromRadians(getAngle() - CAO));
    }

    public double getPerfectAngleRadians() {
      double volt = angleEncoder.getVoltage();
      // Kalibrasyon değerleri arasında sıkışmayı (clamp) sağlar
      if (volt < calibratedMinVolt) volt = calibratedMinVolt;
      if (volt > calibratedMaxVolt) volt = calibratedMaxVolt;

      // Voltajı 0 ile 2PI arasına oranlıyoruz
      return ((volt - calibratedMinVolt) / (calibratedMaxVolt - calibratedMinVolt)) * 2 * Math.PI;
  }

    @Override
    public void periodic() {
        // Her döngüde kalibrasyonu güncellemek istersen buraya ekleyebilirsin
        updateCalibration(); 
        logCalibrationData();

    }

    public double getVelocity() {
      return driveEncoder.getVelocity(); // m/s cinsinden hızı döndürür
  }

    

    public void logCalibrationData() {
        // Artık moduleID hata vermeyecek
        SmartDashboard.putNumber(moduleID + "_MinVolt", calibratedMinVolt);
        SmartDashboard.putNumber(moduleID + "_MaxVolt", calibratedMaxVolt);
        SmartDashboard.putNumber(moduleID + "_CurrentVolt", angleEncoder.getVoltage());
        SmartDashboard.putNumber(moduleID + "_AdjustedAngleDeg", Math.toDegrees(getPerfectAngleRadians()));
    }

    public void setDesiredState(SwerveModuleState desiredState) {
      SwerveModuleState correctedDesiredState = new SwerveModuleState();
      correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
      correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(CAO));

      // Thrifty encoder'dan gelen radyan değerine göre optimizasyon
      correctedDesiredState.optimize(Rotation2d.fromRadians(getAngle()));

      

      drivePID.setSetpoint(correctedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
      
      // Turning için SparkMax PID'si analog sensörü feedback olarak kullanır
      anglePID.setSetpoint(correctedDesiredState.angle.getRadians(), ControlType.kPosition);
    }

    public void resetEncoders() {
      driveEncoder.setPosition(0);
    }

    public double getAngle() {
      double rawVolt = angleEncoder.getVoltage();
      // Voltajdan minV çıkarıp çarpanla çarpıyoruz. Bu SparkMax'in yapamadığı offsettir.
      double correctedAngle = (rawVolt - calibratedMinVolt) * ((2 * Math.PI) / (calibratedMaxVolt - calibratedMinVolt));
      return correctedAngle;
    }

    public void updateCalibration() {
      // Conversion Factor 1.0 iken ham voltajı alıyoruz
      double currentVolt = angleEncoder.getVoltage(); 

      // Eğer okunan değer mevcut min'den küçükse güncelle (0.001 gibi gürültüleri engellemek için 0'dan büyükse)
      if (currentVolt < calibratedMinVolt && currentVolt > 0.0001) {
          calibratedMinVolt = currentVolt;
      }

      // Eğer okunan değer mevcut max'tan büyükse güncelle (3.3'ten büyük okumaları engellemek için)
      if (currentVolt > calibratedMaxVolt && currentVolt < 3.301) {
          calibratedMaxVolt = currentVolt;
      }
    }

    
    
}
