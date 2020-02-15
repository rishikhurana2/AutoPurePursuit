/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import option16.util.Units;

public class DriveTrain extends SubsystemBase {
  /**
   * Creates a new DriveTrain.
   */
// SerialPort port = new SerialPort(57600, SerialPort.Port.kUSB);
  byte refreshRate = 60;
  private static DriveTrain instance;
   TalonSRX left;
   TalonSRX right;
   TalonSRX leftFollow;
   TalonSRX rightFollow;
  double prevD = 0;
  double x, y = 0;
  public DriveTrain() {
    left = new TalonSRX(2);
    right = new TalonSRX(3);
    leftFollow = new TalonSRX(4);
    rightFollow = new TalonSRX(5);

    
   
    configTalon(left);
    configTalon(right);
    configTalon(leftFollow);
    configTalon(rightFollow);
    
    left.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    right.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    leftFollow.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
    rightFollow.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

    left.setInverted(false);
    right.setInverted(true);
    left.setSensorPhase(true);
    right.setSensorPhase(true);

    leftFollow.setInverted(false);
    rightFollow.setInverted(true);

    prevD = (left.getSelectedSensorPosition() + right.getSelectedSensorPosition())/2;

    //leftFollow.setInverted(true);
  }

  public static DriveTrain getInstance(){
    if (instance == null)
      instance = new DriveTrain();
    return instance;
  }
  private void configTalon(TalonSRX talon){
    talon.configFactoryDefault();
    talon.setNeutralMode(NeutralMode.Brake);
    talon.configClosedloopRamp(0);
    talon.configOpenloopRamp(0);
    talon.config_kP(0, 1);
    talon.config_kI(0, 0);
    talon.config_kD(0, 0);
    
    talon.configPeakOutputForward(1);
    talon.configPeakOutputReverse(-1);
  }

  public void tankDrive(double lPow, double rPow) {
    
    left.set(ControlMode.PercentOutput, lPow);
    right.set(ControlMode.PercentOutput, rPow);
    leftFollow.set(ControlMode.Follower, 2);
    rightFollow.set(ControlMode.Follower, 3);
  }
  public double getAngle() {
    System.out.println("is rotating: " + Angle.getInstance().isRotating());
      return Angle.getInstance().getAngle();
  }
  public void resetAngle() {
    //Angle.getInstance().zeroYaw();
  }
  public void resetDistance(){
    left.setSelectedSensorPosition(0);
    right.setSelectedSensorPosition(0);
  }
  public double[] getDistance(){
    double leftD = left.getSelectedSensorPosition();
    double rightD = right.getSelectedSensorPosition();
    double currentD = ((leftD + rightD)/2);
    double ds = currentD - prevD;
    x  +=  Units.ticksToFeet(ds * Math.cos(Math.toRadians(Angle.getInstance().getAngle())));
    y -= Units.ticksToFeet(ds * Math.sin(Math.toRadians(Angle.getInstance().getAngle())));
    double[] distances = {x, y};
    prevD = currentD;
    return distances;
  }

  public void setFPS(double leftP, double rightP){
    double lPow = Units.feetToTicks(leftP) / 10;
    double rPow = Units.feetToTicks(rightP) / 10;
    left.set(ControlMode.Velocity, lPow);
    right.set(ControlMode.Velocity, rPow);
  }

  public void printEncVals(){
    System.out.println("Left: " + left.getSelectedSensorPosition());
    System.out.println("Right: " + right.getSelectedSensorPosition());
    System.out.println("left Follow: " + leftFollow.getSelectedSensorPosition());
    System.out.println("right follow: " + rightFollow.getSelectedSensorPosition());
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //tankDrive(Robot.m_robotContainer.getLeftJoy().getY(), Robot.m_robotContainer.getRightJoy().getY());
  }
}
