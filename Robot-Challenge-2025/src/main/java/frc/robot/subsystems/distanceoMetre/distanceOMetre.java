// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems.distanceoMetre;

// import edu.wpi.first.units.measure.Angle;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.utils.LimelightHelpers;
// import com.ctre.phoenix6.hardware.TalonFX;

// import static edu.wpi.first.units.Units.Degree;
// import static edu.wpi.first.units.Units.Radian;
// import static edu.wpi.first.units.Units.Rotation;

// import com.ctre.phoenix6.hardware.CANcoder;

// public class distanceOMetre extends SubsystemBase {
//   /** Creates a new distanceOMetre. */
//   private TalonFX motor;
//   private CANcoder sensor; 
//   private double tx; 
//   public distanceOMetre() {
//     motor  = new TalonFX(5); 
//     sensor = new CANcoder(4);
//     tx = LimelightHelpers.getTX("limelight2");  
//   }

//   /*
//    * 
//    * must get tx updated in periodic 
//    * cancoder sensor update 
//    * METHOD: create a method with if statments which use the enum states created to move the arm 
//    * start motor
//    * end motor 
//    * get angle 
//    * get distance 
//    * PID controls 
//    *  enum : zero, d5, d10
//    * 
//    */

//   @Override
//   public void periodic() {
    
//   }

//   public void startMotor(double velocity) {
//     motor.set(velocity);
//   } 

//   public void stopMotor() {
//     motor.stopMotor();
//   }

//   public double getAngle() {
//     return sensor.getPosition().getValue().in(Degree); 
//   } 


//   public enum distanceOMetreStates {

//     ZERO(180), 
//     D5(90), 
//     D10(0);

//     private final Angle angle; 

//     distanceOMetreStates(double angle) {
//       this.angle = Degree.of(angle);
//     }

//     public double getTargetAngle() {
//       return angle.in(Rotation); 
//     }


//   }

// }
