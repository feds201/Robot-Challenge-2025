// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.LimelightThreeBase;

/** Add your docs here. */
public class TurretLimelight extends LimelightThreeBase {
    private final double aprilTagHeight = 3; //feet
    private final double constAngle = 0; //Radians
    private final double originToLimelightAngle = 0; //Radians
    private final double goalHeight = 0; //feet
    private final double armAORHeight = 0; //feet
  
    public TurretLimelight(String llName, LimelightThreeBase.LEDState defaultLEDState){
        super(llName, defaultLEDState);
    }

    public double calculateDistance()
  {
    double distance = Math.tan(aprilTagHeight)/getNetworkTableEntry("ty").getDouble(0);
    return distance; 
  }

  public double calculateAngle()
  {
    double fullDistance = calculateDistance();

    return Math.atan2(goalHeight-armAORHeight, fullDistance);
}

public double distanceTarget()
{
double distance = calculateDistance();
double angleLimelight = constAngle - getNetworkTableEntry("tx").getDouble(0);
double distanceTarget = (Math.sin(angleLimelight)*distance)/(Math.sin(originToLimelightAngle));
return distanceTarget;
}

}
