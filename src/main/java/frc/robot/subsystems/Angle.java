/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Angle extends SubsystemBase {
  /**
   * Creates a new Angle.
   */
  AHRS navX;
  double prevA;
 // double prevA;
  private static Angle instance;
  public Angle() {
    navX = new AHRS(SPI.Port.kMXP);
    prevA = navX.getYaw();
  }
  public double getAngle() {
    double angle = navX.getYaw() - prevA;
    if (angle < 0) {
        angle += 360;
    }
    return angle;
  }
  public static Angle getInstance() {
    if (instance == null)
      instance = new Angle();
    return instance;
  } 
  public boolean isRotating() {
    return navX.isRotating();
  }
   @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
