// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

  private CANSparkMax frontRightMotor, frontLeftMotor, backRightMotor, backLeftMotor, leftArticulatorMotor, rightArticulatorMotor, feedMotor;
  private IdleMode shooterIdleMode, articulatorIdleMode, feedIdleMode;
  private DigitalInput shooterBeamBreak, limitSwitch;
  private RelativeEncoder shooterEncoder;

  private final double autoPosition = 0;


  /** Creates a new Shooter. */
  public Shooter() {

    frontLeftMotor = new CANSparkMax(Constants.Shooter.frontLeftMotorID, MotorType.kBrushless);
    frontRightMotor = new CANSparkMax(Constants.Shooter.frontRightMotorID, MotorType.kBrushless);
    backLeftMotor = new CANSparkMax(Constants.Shooter.backLeftMotorID, MotorType.kBrushless);
    backRightMotor = new CANSparkMax(Constants.Shooter.backRightMotorID, MotorType.kBrushless);
    leftArticulatorMotor = new CANSparkMax(Constants.Shooter.leftArticulatorID, MotorType.kBrushless);
    rightArticulatorMotor = new CANSparkMax(Constants.Shooter.rightArticulatorID, MotorType.kBrushless);
    feedMotor = new CANSparkMax(Constants.Shooter.feedID, MotorType.kBrushless);

    // Configure Idle Modes:
    shooterIdleMode = IdleMode.kCoast;
    articulatorIdleMode = IdleMode.kBrake;
    feedIdleMode = IdleMode.kCoast;

    shooterBeamBreak = new DigitalInput(2);
    limitSwitch = new DigitalInput(6);

    shooterEncoder = leftArticulatorMotor.getEncoder();

  }

  public void configureMotors(){
    // Restore Factory Defaults:
    frontRightMotor.restoreFactoryDefaults();
    frontLeftMotor.restoreFactoryDefaults();
    backRightMotor.restoreFactoryDefaults();
    backLeftMotor.restoreFactoryDefaults();

    leftArticulatorMotor.restoreFactoryDefaults();
    rightArticulatorMotor.restoreFactoryDefaults();

    feedMotor.restoreFactoryDefaults();

    // Set Idle Modes:
    frontRightMotor.setIdleMode(shooterIdleMode);
    frontLeftMotor.setIdleMode(shooterIdleMode);
    backRightMotor.setIdleMode(shooterIdleMode);
    backLeftMotor.setIdleMode(shooterIdleMode);

    leftArticulatorMotor.setIdleMode(articulatorIdleMode);
    rightArticulatorMotor.setIdleMode(articulatorIdleMode);

    feedMotor.setIdleMode(feedIdleMode);
  }

  public double getShooterEncoder(){
    return shooterEncoder.getPosition();
  }

  public void resetEncoders(){
    shooterEncoder.setPosition(0);
  }

  public boolean getShooterBeamBreak(){
    return shooterBeamBreak.get();
  }
  public void shooterOn(){
    frontLeftMotor.set(-1);
    frontRightMotor.set(1);
    backLeftMotor.set(-1);
    backRightMotor.set(1);
  }

  public void shooterSlow(){
    frontLeftMotor.set(-.2);
    frontRightMotor.set(.2);
    backLeftMotor.set(-.2);
    backRightMotor.set(.2);
  }

  public void shooterTrap(){
    frontLeftMotor.set(-.65);
    frontRightMotor.set(.65);
    backLeftMotor.set(-.65);
    backRightMotor.set(.65);
  }

  public void shooterOff(){
    frontRightMotor.set(0);
    frontLeftMotor.set(0);
    backRightMotor.set(0);
    backLeftMotor.set(0);

    rightArticulatorMotor.set(0);
    leftArticulatorMotor.set(0);
  }

  public void articulateUp(){
    rightArticulatorMotor.set(.7);
    leftArticulatorMotor.set(-.7);
  }

  public void articulateDown(){
    rightArticulatorMotor.set(-.7);
    leftArticulatorMotor.set(.7);
  }

  public void articulateSlow(double speed){
    rightArticulatorMotor.set(speed);
    leftArticulatorMotor.set(-1*speed);
  }

  public void articulateOff(){
    rightArticulatorMotor.set(0);
    leftArticulatorMotor.set(0);
  }

  public void feed(){
    feedMotor.set(.2);
  }

  public void feedSlow(){
    feedMotor.set(.1);
  }

  public void feedBack(){
    feedMotor.set(-.05);
  }
  public void feedOff(){
    feedMotor.set(0);
  }

  public boolean getLimitSwitch(){
    return limitSwitch.get();
  }

  public double getDistanceSensor(){
    return 0;
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Beam Break 2", getShooterBeamBreak());
    SmartDashboard.putBoolean("ShooterLimitSwitch", getLimitSwitch());
    SmartDashboard.putNumber("Shooter encoder", getShooterEncoder());
    // SmartDashboard.putNumber("Difference", Math.abs(getDistanceSensor() - Constants.Shooter.speakerPosition));
    // This method will be called once per scheduler run
  }
}
