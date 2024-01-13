package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
// import com.ctre.phoenix.sensors.SensorInitializationStrategy;
// import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;

public final class CTREConfigs {

    public CANSparkMax swerveAngleMotor;
    public CANSparkMax swerveDriveMotor;

    public SparkPIDController swerveAnglePidController;
    public SparkPIDController swerveDrivePidController; 

    public TalonFXConfiguration swerveAngleFXConfig;
    public TalonFXConfiguration swerveDriveFXConfig;
    public CANcoderConfiguration swerveCanCoderConfig;

    public CTREConfigs(){
        // swerveAngleMotor = angleSpark;
        // swerveDriveMotor = driveSpark;

        // swerveAnglePidController = swerveAngleMotor.getPIDController();
        // swerveDrivePidController = swerveDriveMotor.getPIDController();

        // swerveAngleFXConfig = new TalonFXConfiguration();
        // swerveDriveFXConfig = new TalonFXConfiguration();
        swerveCanCoderConfig = new CANcoderConfiguration();

        // /* Swerve Angle Motor Configurations */
        // SupplyCurrentLimitConfiguration angleSupplyLimit = new SupplyCurrentLimitConfiguration(
        //     Constants.Swerve.angleEnableCurrentLimit, 
        //     Constants.Swerve.angleContinuousCurrentLimit, 
        //     Constants.Swerve.anglePeakCurrentLimit, 
        //     Constants.Swerve.anglePeakCurrentDuration);

        // swerveAngleMotor.setSmartCurrentLimit(Constants.Swerve.angleContinuousCurrentLimit, Constants.Swerve.anglePeakCurrentLimit);    

        // swerveAngleFXConfig.slot0.kP = Constants.Swerve.angleKP;
        // swerveAnglePidController.setP(Constants.Swerve.angleKP, 0);
        // swerveAngleFXConfig.slot0.kI = Constants.Swerve.angleKI;
        // swerveAnglePidController.setI(Constants.Swerve.angleKI, 0);
        // swerveAngleFXConfig.slot0.kD = Constants.Swerve.angleKD;
        // swerveAnglePidController.setD(Constants.Swerve.angleKD, 0);
        // swerveAngleFXConfig.slot0.kF = Constants.Swerve.angleKF;
        // swerveAnglePidController.setFF(Constants.Swerve.angleKF, 0);
        // swerveAngleFXConfig.supplyCurrLimit = angleSupplyLimit;

        // /* Swerve Drive Motor Configuration */
        // SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
        //     Constants.Swerve.driveEnableCurrentLimit, 
        //     Constants.Swerve.driveContinuousCurrentLimit, 
        //     Constants.Swerve.drivePeakCurrentLimit, 
        //     Constants.Swerve.drivePeakCurrentDuration);

        // swerveDriveMotor.setSmartCurrentLimit(Constants.Swerve.driveContinuousCurrentLimit, Constants.Swerve.drivePeakCurrentLimit);

        // swerveDriveFXConfig.slot0.kP = Constants.Swerve.driveKP;
        // swerveDrivePidController.setP(Constants.Swerve.driveKP, 0);
        // swerveDriveFXConfig.slot0.kI = Constants.Swerve.driveKI;
        // swerveDrivePidController.setI(Constants.Swerve.driveKI, 0);
        // swerveDriveFXConfig.slot0.kD = Constants.Swerve.driveKD;
        // swerveDrivePidController.setD(Constants.Swerve.driveKD, 0);
        // swerveDriveFXConfig.slot0.kF = Constants.Swerve.driveKF; 
        // swerveDrivePidController.setFF(Constants.Swerve.driveKF, 0);       
        // swerveDriveFXConfig.supplyCurrLimit = driveSupplyLimit;
        // swerveDriveFXConfig.openloopRamp = Constants.Swerve.openLoopRamp;
        // swerveDriveMotor.setOpenLoopRampRate(Constants.Swerve.openLoopRamp);
        // swerveDriveFXConfig.closedloopRamp = Constants.Swerve.closedLoopRamp;
        // swerveDriveMotor.setClosedLoopRampRate(Constants.Swerve.closedLoopRamp);
            
        /* Swerve CANCoder Configuration */
        swerveCanCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        swerveCanCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        // swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        // swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
    }

    public void configAngle(CANSparkMax angleMotor){
        swerveAngleMotor = angleMotor;
        swerveAnglePidController = swerveAngleMotor.getPIDController();

        /* Swerve Angle Motor Configurations */
        swerveAngleMotor.setSmartCurrentLimit(Constants.Swerve.angleContinuousCurrentLimit, Constants.Swerve.anglePeakCurrentLimit);    

        swerveAnglePidController.setP(Constants.Swerve.angleKP, 0);
        swerveAnglePidController.setI(Constants.Swerve.angleKI, 0);
        swerveAnglePidController.setD(Constants.Swerve.angleKD, 0);
        swerveAnglePidController.setFF(Constants.Swerve.angleKF, 0);

        swerveAngleMotor.burnFlash();
    }
    public void configDrive(CANSparkMax driveMotor){
        swerveDriveMotor = driveMotor;
        swerveDrivePidController = swerveDriveMotor.getPIDController();

        /* Swerve Angle Motor Configurations */
        swerveDriveMotor.setSmartCurrentLimit(Constants.Swerve.driveContinuousCurrentLimit, Constants.Swerve.drivePeakCurrentLimit);    

        swerveDrivePidController.setP(Constants.Swerve.driveKP, 0);
        swerveDrivePidController.setI(Constants.Swerve.driveKI, 0);
        swerveDrivePidController.setD(Constants.Swerve.driveKD, 0);
        swerveDrivePidController.setFF(Constants.Swerve.driveKF, 0);
        
        swerveDriveMotor.burnFlash();
    }

    public CANcoderConfiguration getCANConfig(){

        CANcoderConfiguration swerveCanCoderConfig = new CANcoderConfiguration();

        swerveCanCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        swerveCanCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        // swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        // swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;


        return swerveCanCoderConfig;
    }
}