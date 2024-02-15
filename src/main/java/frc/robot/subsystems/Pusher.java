// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Pusher extends SubsystemBase {

  // private final DoubleSolenoid tongue;
  private final DoubleSolenoid push;

  
  /** Creates a new Climber. */
  public Pusher() {
    // tongue = new DoubleSolenoid(1, PneumaticsModuleType.CTREPCM, Constants.Snake.boopID1, Constants.Snake.boopID2); 
    push = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.Snake.boopID1, Constants.Snake.boopID2);
    push.set(DoubleSolenoid.Value.kForward);
  }


  public void push(){
    push.toggle();
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}