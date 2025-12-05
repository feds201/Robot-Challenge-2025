// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.units.measure.Angle;
// import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.LimelightThreeBase;

/** Add your docs here. */
public class TurretLimelight extends LimelightThreeBase {
    private final double aprilTagHeight = 3; //feet
    private final double groundToLimelightHeight = 2; //feet

    //Angle between the center of limelight FOV and the imaginary side connecting the origin of the limelight to the origin of the arm.
    private final double constAngle = 0; //Radians
    private final double originToLimelightAngle = 0; //Radians
    private final double goalHeight = 0; //feet
    private final double armAORHeight = 0; //feet
  
    public TurretLimelight(String llName, LimelightThreeBase.LEDState defaultLEDState){
        super(llName, defaultLEDState);
    }

    //calculates distance between limelight and wall of april tag
    public double calculateDistance()
  {
    double distance = (aprilTagHeight-groundToLimelightHeight)/Math.tan(getNetworkTableEntry("ty").getDouble(0));
    return distance; 
  }

  //calculates angle between arm origin and april tag itself
  public Angle calculateAngle()
  {
    double fullDistance = distanceTarget();

    return Radians.of(Math.atan2(goalHeight-armAORHeight, fullDistance));
}

//calculates distance between arm origin and wall of april tag
public double distanceTarget()
{

double distance = calculateDistance();
double angleLimelight = constAngle - getNetworkTableEntry("tx").getDouble(0);
double distanceTarget = (Math.sin(angleLimelight)*distance)/(Math.sin(originToLimelightAngle));
return distanceTarget;
}

}
