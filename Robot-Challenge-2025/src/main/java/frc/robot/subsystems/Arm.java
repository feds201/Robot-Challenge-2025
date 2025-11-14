// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.LimelightThreeBase.LEDState;

public class Arm extends SubsystemBase {


  public enum armState{
    //Straight out
    ZERO(0),
    TARGETING(90),
    //Straight up
    STOW(90);
    private double angle;
    armState(double angle){
      this.angle = angle;
    }
    public double getAngle(){
      return angle;
    }
  }
  private armState state = null;
  private TalonFX motor = new TalonFX(6); 
  private CANcoder encoder = new CANcoder(7);
  private TalonFXConfiguration tConfig = new TalonFXConfiguration();
  private CANcoderConfiguration eConfig = new CANcoderConfiguration();
  private TurretLimelight ll = new TurretLimelight("limelight-num", LEDState.OFF);

  /** Creates a new Arm. */
  public Arm() {
    eConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = -.25;

    tConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    tConfig.Slot0.kP = 0.0;
    tConfig.Slot0.kD = 0.0;
    tConfig.Slot0.kG = 0.0;
    tConfig.Slot0.kS = 0.0;
    tConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    tConfig.CurrentLimits.StatorCurrentLimit = 40; //40 Amp
  

  }
 
  @Override
  public void periodic() {

    switch(state){
      case STOW:

      break;
      case TARGETING:

      break;
      case ZERO:

      break;
      default:

      break;
    }

  }

  
}
