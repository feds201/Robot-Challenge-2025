// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.SubsystemStatusManager;
import frc.robot.utils.LimelightThreeBase.LEDState;

public class Arm extends SubsystemBase {


  public enum armState{
    //Straight out
    ZERO(Radians.of(0)),
    TARGETING(Degrees.of(90)),
    //Straight up
    STOW(Rotations.of(.25));
    private Angle angle;
    armState(Angle angle){
      this.angle = angle;
    }
    public Angle getAngle(){
      return angle;
    }
  }
  private armState state = armState.ZERO;
  private TalonFX turretArmMotor = new TalonFX(6); 
  private MotionMagicVoltage motionMagic = new MotionMagicVoltage(0);
  private CANcoder turretArmEncoder = new CANcoder(7);
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
    tConfig.MotionMagic.MotionMagicAcceleration = 0.0;
    tConfig.MotionMagic.MotionMagicCruiseVelocity = 0.0;
  
    for (int j = 0; j < 2; ++j){
      var status = turretArmMotor.getConfigurator().apply(tConfig);
      if(status.isOK()) break;
    }

    for (int j = 0; j < 2; ++j){
      var status = turretArmEncoder.getConfigurator().apply(eConfig);
      if(status.isOK()) break;
    }

    SubsystemStatusManager.addSubsystem(getName(), turretArmMotor, turretArmEncoder);
  }
 
  @Override
  public void periodic() {

    switch(state){
      case STOW:
      case ZERO:
        break;
      case TARGETING:
        if (ll.getNetworkTableEntry("ty").getDouble(0) == 0){
          setPosition(state.getAngle());
        } else {
          setPosition(ll.calculateAngle());
        }
      
        break;
    }

  }

  public Angle getAngle(){
    return turretArmEncoder.getAbsolutePosition().getValue();
  }

  public void setState(armState state){
    this.state = state;
    setPosition(state.getAngle());
  }

  /*
   * Set the position of the motor, and command the motor to track to that position
   */
  public void setPosition(Angle setpoint){
    turretArmMotor.setControl(motionMagic.withPosition(setpoint));
  }

  public boolean atSetPoint(){
    Angle angle = getAngle();
    Angle desiredAngle = state.getAngle();
    Angle tolerance = Degrees.of(1);
    if (Math.abs(desiredAngle.minus(angle).abs(Degrees)) < tolerance.abs(Degrees)){
      return true;
    } else {
      return false;
    }
  }

  public Command commandState(armState state){
    return new InstantCommand(()-> setState(state));
  }
}


