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

  private CANSparkMax frontIntakeMotor, backIntakeMotor, intakeArticulatorMotor;
  private IdleMode intakeIdleMode, intakeArticulatorIdleMode;


  /** Creates a new Intake. */
  public Intake() {

    frontIntakeMotor = new CANSparkMax(Constants.Intake.frontIntakeID, MotorType.kBrushless);
    backIntakeMotor = new CANSparkMax(Constants.Intake.backIntakeID, MotorType.kBrushless);
    intakeArticulatorMotor = new CANSparkMax(Constants.Intake.articulateIntakeID, MotorType.kBrushless);

    // Configure Idle Modes:
    intakeIdleMode = IdleMode.kBrake;
    intakeArticulatorIdleMode = IdleMode.kBrake;

  }

  public void configureMotors(){
    // Restore Factory Defaults:
    frontIntakeMotor.restoreFactoryDefaults();
    backIntakeMotor.restoreFactoryDefaults();
    intakeArticulatorMotor.restoreFactoryDefaults();

    // Set Idle Modes:
    backIntakeMotor.setIdleMode(intakeIdleMode);
    frontIntakeMotor.setIdleMode(intakeIdleMode);
    intakeArticulatorMotor.setIdleMode(intakeArticulatorIdleMode);
  }

  public void intakeOn(){
    frontIntakeMotor.set(1);
    backIntakeMotor.set(1);
  }

  public void intakeOff(){
    frontIntakeMotor.set(0);
    backIntakeMotor.set(0);
  }

  public void intakeReverse(){
    frontIntakeMotor.set(-1);
    backIntakeMotor.set(-1);
  }

  public void intakeUp(){
    intakeArticulatorMotor.set(1);
  }
  
  public void intakeDown(){
    intakeArticulatorMotor.set(-1);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
