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

  private CANSparkMax frontRightMotor, frontLeftMotor, backRightMotor, backLeftMotor, articulatorMotor;
  private IdleMode shooterIdleMode, articulatorIdleMode;


  /** Creates a new Shooter. */
  public Shooter() {

    frontLeftMotor = new CANSparkMax(Constants.Shooter.frontLeftMotorID, MotorType.kBrushless);
    frontRightMotor = new CANSparkMax(Constants.Shooter.frontRightMotorID, MotorType.kBrushless);
    backLeftMotor = new CANSparkMax(Constants.Shooter.backLeftMotorID, MotorType.kBrushless);
    backRightMotor = new CANSparkMax(Constants.Shooter.backRightMotorID, MotorType.kBrushless);

    // Configure Idle Modes:
    shooterIdleMode = IdleMode.kCoast;
    articulatorIdleMode = IdleMode.kBrake;

  }

  public void configureMotors(){
    // Restore Factory Defaults:
    frontRightMotor.restoreFactoryDefaults();
    frontLeftMotor.restoreFactoryDefaults();
    backRightMotor.restoreFactoryDefaults();
    backLeftMotor.restoreFactoryDefaults();
    articulatorMotor.restoreFactoryDefaults();

    // Set Inverted:
    frontLeftMotor.setInverted(true);
    backLeftMotor.setInverted(true);

    // Set Idle Modes:
    frontRightMotor.setIdleMode(shooterIdleMode);
    frontLeftMotor.setIdleMode(shooterIdleMode);
    backRightMotor.setIdleMode(shooterIdleMode);
    backLeftMotor.setIdleMode(shooterIdleMode);
    articulatorMotor.setIdleMode(articulatorIdleMode);
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
