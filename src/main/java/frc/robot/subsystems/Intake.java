// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  private CANSparkMax leftIntakeMotor, rightIntakeMotor, intakeArticulatorMotor;
  private IdleMode intakeIdleMode, intakeArticulatorIdleMode;


  /** Creates a new Intake. */
  public Intake() {

    leftIntakeMotor = new CANSparkMax(Constants.Intake.leftIntakeID, MotorType.kBrushless);
    rightIntakeMotor = new CANSparkMax(Constants.Intake.rightIntakeID, MotorType.kBrushless);
    intakeArticulatorMotor = new CANSparkMax(Constants.Intake.articulateIntakeID, MotorType.kBrushless);

    // Configure Idle Modes:
    intakeIdleMode = IdleMode.kBrake;
    intakeArticulatorIdleMode = IdleMode.kBrake;

  }

  public void configureMotors(){
    // Restore Factory Defaults:
    leftIntakeMotor.restoreFactoryDefaults();
    rightIntakeMotor.restoreFactoryDefaults();
    intakeArticulatorMotor.restoreFactoryDefaults();

    // Set Idle Modes:
    rightIntakeMotor.setIdleMode(intakeIdleMode);
    leftIntakeMotor.setIdleMode(intakeIdleMode);
    intakeArticulatorMotor.setIdleMode(intakeArticulatorIdleMode);

    leftIntakeMotor.setInverted(true);
  }

  public void intakeOn(){
    leftIntakeMotor.set(1);
    rightIntakeMotor.set(1);
  }

  public void intakeOff(){
    intakeArticulatorMotor.set(0);
  }

  public void intakeReverse(){
    leftIntakeMotor.set(-1);
    rightIntakeMotor.set(-1);
  }

  public void intakeUp(){
    intakeArticulatorMotor.set(.1);
  }
  
  public void intakeDown(){
    intakeArticulatorMotor.set(-.1);
  }

  public void intakeStop(){
    leftIntakeMotor.set(0);
    rightIntakeMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
