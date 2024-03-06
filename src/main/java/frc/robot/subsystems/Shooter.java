// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {

  private CANSparkMax frontRightMotor, frontLeftMotor, backRightMotor, backLeftMotor, leftArticulatorMotor, rightArticulatorMotor, feedMotor;
  private IdleMode shooterIdleMode, articulatorIdleMode, feedIdleMode;


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

    // Set Inverted:
    frontLeftMotor.setInverted(true);
    backLeftMotor.setInverted(true);
    leftArticulatorMotor.setInverted(true);

    // Set Idle Modes:
    frontRightMotor.setIdleMode(shooterIdleMode);
    frontLeftMotor.setIdleMode(shooterIdleMode);
    backRightMotor.setIdleMode(shooterIdleMode);
    backLeftMotor.setIdleMode(shooterIdleMode);

    leftArticulatorMotor.setIdleMode(articulatorIdleMode);
    rightArticulatorMotor.setIdleMode(articulatorIdleMode);

    feedMotor.setIdleMode(feedIdleMode);
  }

  public void shooterOn(){
    frontLeftMotor.set(1);
    frontRightMotor.set(1);
    backLeftMotor.set(1);
    backRightMotor.set(1);
  }

  public void shooterOff(){
    frontRightMotor.set(0);
    frontLeftMotor.set(0);
    backRightMotor.set(0);
    backRightMotor.set(0);
  }

  public void articulateUp(){
    rightArticulatorMotor.set(.1);
    leftArticulatorMotor.set(.1);
  }

  public void articulateDown(){
    rightArticulatorMotor.set(-.1);
    leftArticulatorMotor.set(-.1);
  }

  public void articulateOff(){
    rightArticulatorMotor.set(0);
    leftArticulatorMotor.set(0);
  }

  public void feed(){
    feedMotor.set(1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
