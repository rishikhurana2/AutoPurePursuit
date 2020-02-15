/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;


import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Angle;
import frc.robot.subsystems.DriveTrain;
import option16.util.*;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
//  * project
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  public static RobotContainer m_robotContainer;
  
  Notifier odo;
  Notifier pp;
  double x = 0;
  double y = 0;
  double [] vels;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  
    //RobotContainer.DriveTrain.getInstance().resetAngle();
    //RobotContainer.DriveTrain.getInstance().resetDistance();
  //  PurePursuitHandler.addPoint(new Point(0, 0));
   // PurePursuitHandler.addPoint(new Point(0, 3));
    //PurePursuitHandler.clearPath();
    //PurePursuitHandler.addPoint(new Point(0,0));
    //PurePursuitHandler.addPoint(new Point(0,-3));
    PurePursuitHandler.addPoint(new Point(0,0));
    PurePursuitHandler.addPoint(new Point(0,2));
    PurePursuitHandler.addPoint(new Point(4,4));
    // odo = new Notifier(()-> {
    //   distance = DriveTrain.getInstance().getDistance() - prevD;
    //   theta = DriveTrain.getInstance().getAngle() - prevA;
    //   while(theta > 180) {
    //     theta -= 360;
    //   }
    //   while (theta < 180) {
    //     theta += 360;
    //   }
    //   theta = Math.toRadians(theta);
    //   x = distance*Math.cos(theta);
    //   y = distance*Math.sin(theta);
    // });
    // pp = new Notifier(()-> {
    //   vels = PurePursuitHandler.getNextVelocities(new Odometry(x, y, theta));
    //   DriveTrain.getInstance().setFPS(vels[0], vels[1]);
    //   if (PurePursuitHandler.isFinished()) {
    //     vels[0] = 0;
    //     vels[1] = 0;
    //   }
    // });
   // PurePursuitHandler.addPoint(new Point(4,4));
    //PurePursuitHandler.addPoint(new Point(4,0));
    //PurePursuitHandler.addPoint(new Point(4,4));


  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    double [] distance = DriveTrain.getInstance().getDistance();
    double angle = DriveTrain.getInstance().getAngle();
//    System.out.println("X: " + distance[0]);
  //  System.out.println("Y:" + distance[1]);
   // System.out.println("Angle: " + angle);
//    double angle = Angle.getInstance().getAngle();
    
    //System.out.println("X-Coordinate: " + distance * Math.cos(Math.toRadians(angle)));
    //System.out.println("Y-Coordinate: " + distance * Math.sin(Math.toRadians(angle)));
    //DriveTrain.getInstance().printEncVals();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
//    odo.startPeriodic(0.01);
  //  pp.startPeriodic(0.01);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    double[] distance = DriveTrain.getInstance().getDistance();
    double theta = (Angle.getInstance().getAngle());
    x = distance[0];
    y = distance[1];
    System.out.println("Angle: " + theta);
    System.out.println("X: " + x);
    System.out.println("Y: " + y);
    double [] velocities = PurePursuitHandler.getNextVelocities(new Odometry(x, y, Math.toRadians(theta)));
    if(PurePursuitHandler.isFinished()) {
     velocities[0] = 0;
     System.out.println("Finished!");
     velocities[1] = 0;
    }
   DriveTrain.getInstance().setFPS(velocities[0], velocities[1]);
    
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
  }
  double initialAngle;
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  //  DriveTrain.getInstance().tankDriveTrain.getInstance()(0.5,0.5);
    // double [] distance = DriveTrain.getInstance().getDistance();
    // double angle = Angle.getInstance().getAngle();
    // double x = distance * Math.cos(Math.toRadians(angle));
    // double y = distance * Math.sin(Math.toRadians(angle));
    // System.out.println("X: " + x);
    // System.out.println("Y: " + y);
    // System.out.println("angle: " + angle);
    // DriveTrain.getInstance().tankDrive(0.5, -0.5);
    double [] ds = DriveTrain.getInstance().getDistance();
    System.out.println("X: "+ ds[0]);
    System.out.println("Y: "+ ds[1]);
  }
}
