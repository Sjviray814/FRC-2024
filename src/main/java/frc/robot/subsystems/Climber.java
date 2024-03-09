// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {

  private CANSparkMax leftClimberMotor, rightClimberMotor;
  private IdleMode climberIdleMode;


  /** Creates a new Climber. */
  public Climber() {

    leftClimberMotor = new CANSparkMax(Constants.Climber.leftWinchID, MotorType.kBrushless);
    rightClimberMotor = new CANSparkMax(Constants.Climber.rightWinchID, MotorType.kBrushless);

    // Configure Idle Modes:
    climberIdleMode = IdleMode.kBrake;

  }

  public void configureMotors(){
    // Restore Factory Defaults:
    leftClimberMotor.restoreFactoryDefaults();
    rightClimberMotor.restoreFactoryDefaults();

    // Set Idle Modes:
    rightClimberMotor.setIdleMode(climberIdleMode);
    leftClimberMotor.setIdleMode(climberIdleMode);

    leftClimberMotor.setInverted(true);
  }

  public void climberOn(){
    leftClimberMotor.set(-.25);
    rightClimberMotor.set(-.25);
  }


  public void climberReverse(){
    leftClimberMotor.set(.25);
    rightClimberMotor.set(.25);
  }

  public void climberStop(){
    leftClimberMotor.set(0);
    rightClimberMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
