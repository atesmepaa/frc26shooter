package frc.robot.subsystems.Swerve;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;


import com.revrobotics.spark.FeedbackSensor;

import frc.robot.Constants.ModuleConstants;

public class Configs {
    public static class SwerveXS {
        public static final SparkMaxConfig driveConfig = new SparkMaxConfig();
        public static final SparkMaxConfig angleConfig = new SparkMaxConfig();

        

        static {
            // --- Drive Ayarları (NEO) ---
            double driveFactor = ModuleConstants.wheelDiameterMeters * Math.PI
                                   / ModuleConstants.drivingMotorReduction;
            double driveVelocityFeedForward = 1 / ModuleConstants.driveWheelFreeSpeedRps;

            

            driveConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(ModuleConstants.drivingMotorCurrentLimit)
                    .openLoopRampRate(0.25)   // Joystick ile sürerken ivmelenme
                    .closedLoopRampRate(0.35);

            driveConfig.encoder
                    .positionConversionFactor(driveFactor) // metre
                    .velocityConversionFactor(driveFactor / 60.0); // metre/saniye

            driveConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    .pid(0.04, 0, 0)
                    .velocityFF(driveVelocityFeedForward)
                    .outputRange(-1, 1);
            
         

            double angleFactor = (2 * Math.PI)/3.4;

            angleConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(ModuleConstants.turningMotorCurrentLimit)   // Joystick ile sürerken ivmelenme
                    .closedLoopRampRate(0.1);

            angleConfig.analogSensor
                    .positionConversionFactor(angleFactor)
                    .velocityConversionFactor((angleFactor) / 60.0);

            angleConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kAnalogSensor)
                    .pid(0.6, 0, 0)
                    .outputRange(-1, 1)
                    .positionWrappingEnabled(true)
                    .positionWrappingInputRange(0, (angleFactor+0.5));
        }
    }
}
