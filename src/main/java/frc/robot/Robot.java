// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Random;

import org.opencv.core.Mat;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.sensors.CANCoder;
import com.kauailabs.navx.frc.AHRS;
//For SparkMax Support add this online vendor library: 
//https://software-metadata.revrobotics.com/REVLib-2023.json
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import pabeles.concurrency.IntOperatorTask.Max;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  //Joystick
  Joystick controller = new Joystick(0);
  double xAxis;
  double yAxis;
  double zAxis;
  double wAxis;
  static double DEADBAND = 0.1;

  // navX
  AHRS navX = new AHRS();
  double navXYawAngle;
  double navXYawOffset;
  double navXRollOffset;
  double navXRollAngle;

  // Auto
  Timer timer = new Timer();

  
//*********************************************************************** */
//                           Custom Functions
//*********************************************************************** */

  SwervCorner cornerFL = new SwervCorner(5, 6, 12, 1 , 1, 9.4921875, -1);
  SwervCorner cornerFR = new SwervCorner(2, 1, 9, 1, -1, 207.421875, 1);
  SwervCorner cornerBL = new SwervCorner(7, 8, 11, -1, 1, 289.86328125, -1);
  SwervCorner cornerBR = new SwervCorner(4, 3, 10, -1, -1, 91.2304688, 1);
  Arm myArm = new Arm();
  private RobotContainer m_robotContainer;

  // public double getDegrees(double x, double y, double z, double zvecX, double zvecY) {
  //   double correctedX;
  //   double correctedY;
  //   if(x != 0 || y != 0){
  //     double startHypotenuse = Math.sqrt((Math.pow(x, 2)+(Math.pow(y, 2))));
  //     double startAngle = Math.toDegrees(Math.acos(x / startHypotenuse));
  //     if (y < 0) startAngle = 360 - startAngle;
  //     correctedX = Math.cos(Math.toRadians(startAngle + navXAngle))*startHypotenuse;
  //     correctedY = Math.sin(Math.toRadians(startAngle + navXAngle))*startHypotenuse;
  //   } else{
  //     correctedX = 0;
  //     correctedY = 0; 
  //   }
    
  //   double totalX = correctedX + (zvecX * z);
  //   double totalY = correctedY + (zvecY * z);
  //   double hypotenuse = Math.sqrt((Math.pow(totalX, 2)+(Math.pow(totalY, 2))));
  //   double angle = Math.toDegrees(Math.acos(totalX / hypotenuse));
  //   if (totalY < 0) angle = 360 - angle;
  //   return angle;
  // }

  // public double getMag(double x, double y, double z, double zvecX, double zvecY){
  //   double correctedX;
  //   double correctedY;
  //   if(x != 0 || y != 0){
  //     double startHypotenuse = Math.sqrt((Math.pow(x, 2)+(Math.pow(y, 2))));
  //     double startAngle = Math.toDegrees(Math.acos(x / startHypotenuse));
  //     if (y < 0) startAngle = 360 - startAngle;
  //     correctedX = Math.cos(Math.toRadians(startAngle + navXAngle))*startHypotenuse;
  //     correctedY = Math.sin(Math.toRadians(startAngle + navXAngle))*startHypotenuse;
  //   } else{
  //     correctedX = 0;
  //     correctedY = 0; 
  //   }

  //   double totalX = correctedX + (zvecX * z);
  //   double totalY = correctedY + (zvecY * z);
  //   double hypotenuse = Math.sqrt((Math.pow(totalX, 2)+(Math.pow(totalY, 2))));
  //   return hypotenuse;
  // }

  public double getMaxMag(double x, double y, double z){
    return Math.max(Math.max(cornerFL.driveMagnitude, cornerFR.driveMagnitude),Math.max(cornerBL.driveMagnitude, cornerBR.driveMagnitude));
  }

  public double normalize(double x, double y, double z, double zvecX, double zvecY){
    double correctedX;
    double correctedY;
    if(x != 0 || y != 0){
      double startHypotenuse = Math.sqrt((Math.pow(x, 2)+(Math.pow(y, 2))));
      double startAngle = Math.toDegrees(Math.acos(x / startHypotenuse));
      if (y < 0) startAngle = 360 - startAngle;
      correctedX = Math.cos(Math.toRadians(startAngle + navXYawAngle))*startHypotenuse;
      correctedY = Math.sin(Math.toRadians(startAngle + navXYawAngle))*startHypotenuse;
    } else{
      correctedX = 0;
      correctedY = 0; 
    }
    

    double maxMag = getMaxMag(x, y, z);
    double totalX = correctedX + (zvecX * z) / maxMag;
    double totalY = correctedY + (zvecY * z) / maxMag;
    double hypotenuse = Math.sqrt((Math.pow(totalX, 2)+(Math.pow(totalY, 2))));
    return hypotenuse;
  }

  public double getSpeed(double x, double y, double z, SwervCorner corner){
    if(getMaxMag(x, y, z) > 1){
      double speed = normalize(x, y, z, corner.zvecX, corner.zvecY);
      return speed;
    } else{
      double speed = corner.driveMagnitude;
      return speed;
    }
  }




  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    timer.reset();
    timer.start();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {

  }

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    cornerFL.setEncoder();
    cornerFR.setEncoder();
    cornerBL.setEncoder();
    cornerBR.setEncoder();
  }
   public boolean autoTest = false;
   public double autoTestDistance;
   public double currentDistance;
   public double distanceOffset;
   public double rev = 0.08;
  @Override
  public void teleopPeriodic() {
    // Drive Code
    xAxis = controller.getRawAxis(0);
    yAxis = controller.getRawAxis(1) * -1;
    zAxis = controller.getRawAxis(4);
    wAxis = controller.getRawAxis(3) - controller.getRawAxis(2);

    // Deadband
    if (Math.abs(xAxis) < DEADBAND) xAxis = 0;
    if (Math.abs(yAxis) < DEADBAND) yAxis = 0;
    if (Math.abs(zAxis) < DEADBAND) zAxis = 0;
    if (Math.abs(wAxis) < DEADBAND) wAxis = 0;
    zAxis = zAxis*0.5;

    myArm.moveArm(wAxis);
    if(controller.getRawButtonPressed(1)){
      myArm.switchArm();
    }
    if (controller.getRawButton(5)){
      System.out.println("Intake");
      myArm.Intake();
    }else if (controller.getRawButton(6)){
      System.out.println("Outtake");
      myArm.Outtake();
    }else{
      myArm.Stoptake();
    }
/*
    if(controller.getRawButton(2)){ // Auto Balance
      zAxis = 0;
      xAxis = 0;
      yAxis = navXRollAngle/100;
    }

    if(controller.getRawButton(3) && !autoTest){
      autoTest = true;
      autoTestDistance = 10;
      distanceOffset = cornerBL.getDriveEncoderPosition();
      currentDistance = cornerBL.getDriveEncoderPosition();
    } */

    if(autoTest && (Math.abs(currentDistance-distanceOffset) <= autoTestDistance) && !controller.getRawButton(4)){
      zAxis = 0;
      xAxis = 0;
      yAxis = 0.15+(1 - Math.abs(currentDistance-distanceOffset)/autoTestDistance)*0.3;
      currentDistance = cornerBL.getDriveEncoderPosition();
      System.out.println(Math.abs(currentDistance-distanceOffset));
    } else if (autoTest){
      autoTest = false;
    }

    if (xAxis != 0 || yAxis != 0 || zAxis != 0){
      // Update Swerve Corner Vals
      cornerFL.updateCorner(xAxis, yAxis, zAxis, navXYawAngle);
      cornerFR.updateCorner(xAxis, yAxis, zAxis, navXYawAngle);
      cornerBL.updateCorner(xAxis, yAxis, zAxis, navXYawAngle);
      cornerBR.updateCorner(xAxis, yAxis, zAxis, navXYawAngle);

      // Set Drive Angle
      cornerFL.goTo(cornerFL.driveAngle);
      cornerFR.goTo(cornerFR.driveAngle);
      cornerBL.goTo(cornerBL.driveAngle);
      cornerBR.goTo(cornerBR.driveAngle);

      // Set Drive Speed
      cornerFL.driveSpeed(getSpeed(xAxis, yAxis, zAxis, cornerFL) * rev);
      cornerFR.driveSpeed(getSpeed(xAxis, yAxis, zAxis, cornerFR)* rev);
      cornerBL.driveSpeed(getSpeed(xAxis, yAxis, zAxis, cornerBL)* rev);
      cornerBR.driveSpeed(getSpeed(xAxis, yAxis, zAxis, cornerBR)* rev);
      if (rev < 1){
        rev = rev + 0.02;
      }
    } else{
      cornerFL.stopRotation();
      cornerFR.stopRotation();
      cornerBL.stopRotation();
      cornerBR.stopRotation();

      cornerFL.driveSpeed(0);
      cornerFR.driveSpeed(0);
      cornerBL.driveSpeed(0);
      cornerBR.driveSpeed(0);
      rev = 0.08;
    }

    //navX
    if(controller.getRawButton(2)){
      navXYawOffset = navX.getYaw();
      navXRollOffset = navX.getPitch();
    }
    navXYawAngle = navX.getYaw() - navXYawOffset;
    navXRollAngle = navX.getRoll() - navXRollOffset;

  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  System.out.println(cornerBL.getDriveEncoderPosition());

    // if(!navX.isCalibrating() && controller.getRawButton(1)){
    //   navXYawOffset = navX.getYaw();
    //   navXRollOffset = navX.getPitch();
    // }
    // navXYawAngle = navX.getYaw() - navXYawOffset;
    // navXRollAngle = navX.getRoll() - navXRollOffset;
    // System.out.println(navXRollAngle);

    // System.out.println("FL: " + cornerFL.getEncoderPosition());
    // System.out.println("FR: " + cornerFR.getEncoderPosition());
    // System.out.println("BL: " + cornerBL.getEncoderPosition());
    // System.out.println("BR: " + cornerBR.getEncoderPosition());
  }

  @Override
  public void testExit() {}
}
