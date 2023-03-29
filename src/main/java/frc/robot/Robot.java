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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import pabeles.concurrency.IntOperatorTask.Max;
import edu.wpi.first.networktables.*;

public class Robot extends TimedRobot {
  
  private Command m_autonomousCommand;

  //Joystick
  Joystick controller = new Joystick(0);
  double xAxis;
  double yAxis;
  double zAxis;
  double wAxis;
  static double DEADBAND = 0.1;
  public int intakeSwitch = 1;

  //Drive Station
  Joystick driveStation = new Joystick(1);

  // navX
  AHRS navX = new AHRS();
  double navXYawAngle = 0;
  double navXYawOffset = 0;
  double navXRollOffset = 0;
  double navXRollAngle = 0;
  double navXPitchOffset = 0;
  double navXPitchAngle = 0;

  // Auto
  Timer timer = new Timer();
  boolean waitingTimer = false;
  double autoSetting;
  double autoBalanceMaxAngle = 0;

  // Limelight
  double llDistance;
  double llAngle;

  // Human Player Auto
  double vx;
  double vy;
  double thetaF;
  boolean calculate = true;
  boolean rotateBeforeCalc = true;
  double hyp; 

  
//*********************************************************************** */
//                           Custom Functions
//*********************************************************************** */

  SwervCorner cornerFL = new SwervCorner(5, 6, 12, 1 , 1, 9.4921875, -1);
  SwervCorner cornerFR = new SwervCorner(2, 1, 9, 1, -1, 201.121875, 1);
  SwervCorner cornerBL = new SwervCorner(7, 8, 11, -1, 1, 289.86328125, -1);
  SwervCorner cornerBR = new SwervCorner(4, 3, 10, -1, -1, 5.18547, 1);
  Arm myArm = new Arm();
  private RobotContainer m_robotContainer;

  public boolean robotWait(double waitTimeSeconds){
    if(!waitingTimer){
      waitingTimer = true;
      timer.reset();
      timer.start();
      return true;
    }else if(timer.get() < waitTimeSeconds){
      return true;
    }else{
      waitingTimer = false;
      return false;
    }
  }

  public boolean robotTurnTo(double degree) {
    xAxis = 0;
    yAxis = 0;
    double clockwise = (navXYawAngle + degree) % 360;
    double anticlockwise = (navXYawAngle - degree) % 360;
    if (navXYawAngle < degree - 1 || navXYawAngle > degree + 1 ) {
        if (clockwise > anticlockwise) {
            zAxis = anticlockwise * -0.1;
        }
        if (clockwise < anticlockwise) {
            zAxis = clockwise/4 * 0.1;
        }
        return true;
    }else{
        return false;
    }
  }

  public boolean robotDriveTo(double distance, double x, double y){
    if(autoDrive == false){
        autoDrive = true;
        autoDriveDistance = distance;
        distanceOffset = cornerBL.getDriveEncoderPosition();
        currentDistance = cornerBL.getDriveEncoderPosition();
    }

    if(autoDrive && (Math.abs(currentDistance-distanceOffset) <= autoDriveDistance)){
        zAxis = 0 ;
        xAxis = x*(1 - Math.abs(currentDistance-distanceOffset)/autoDriveDistance);
        yAxis = y*(1 - Math.abs(currentDistance-distanceOffset)/autoDriveDistance);
        currentDistance = cornerBL.getDriveEncoderPosition();
        return true;
      } else {
        autoDrive = false;
        return false;
    }
  }

  public void resetnavX(){
    navXYawOffset = navX.getYaw();
    navXPitchOffset = navX.getPitch();
    navXRollOffset = navX.getRoll();
    myArm.resetArmEncoderOffset();
  }

  public boolean leftHumanPlayerAuto(){
    if(!robotTurnTo(0) && rotateBeforeCalc){
        calculate = true;
        return true;
    }else{
        if(calculate){
            rotateBeforeCalc = false;
            calculate = false;
            vx = llDistance*Math.cos(Math.toRadians(90-llAngle))-4.1*Math.sin(Math.toRadians(39.66666));
            vy = llDistance*Math.sin(Math.toRadians(90-llAngle))-4.1*Math.cos(Math.toRadians(39.66666));
            thetaF = Math.toDegrees(Math.atan(vx/vy));
            hyp = Math.sqrt(Math.pow(vx, 2)+Math.pow(vy, 2));
        }
        if(!robotDriveTo(hyp, Math.cos(Math.toRadians(thetaF)), Math.sin(Math.toRadians(thetaF))) && !myArm.armHigh()){
            return false;
        }else{
            rotateBeforeCalc = true;
            return true;
        }
    }
  }

  public boolean rightHumanPlayerAuto(){
    if(!robotTurnTo(0) && rotateBeforeCalc){
        calculate = true;
        return true;
    }else{
        if(calculate){
            rotateBeforeCalc = false;
            calculate = false;
            vx = llDistance*Math.cos(Math.toRadians(90-llAngle))+4.1*Math.sin(Math.toRadians(39.66666));
            vy = llDistance*Math.sin(Math.toRadians(90-llAngle))-4.1*Math.cos(Math.toRadians(39.66666));
            thetaF = Math.toDegrees(Math.atan(vx/vy));
            hyp = Math.sqrt(Math.pow(vx, 2)+Math.pow(vy, 2));
        }
        if(!robotDriveTo(hyp, Math.cos(Math.toRadians(thetaF)), Math.sin(Math.toRadians(thetaF))) && !myArm.armHigh()){
          return false;
        }else{
          rotateBeforeCalc = true;
          return true;
        }
    }
  }


  public void flip180() {
    if ( Math.abs(navXYawAngle)>90) {
      robotTurnTo(180);
    } else {
      robotTurnTo(0);
    }
  }

  public void autoBalance(){
    if(Math.abs(navXRollAngle)>=Math.abs(autoBalanceMaxAngle)){
      autoBalanceMaxAngle = navXRollAngle;
    }else if(Math.abs(autoBalanceMaxAngle)-Math.abs(navXRollAngle)>8){
      navXRollAngle = 0; //Weird fix
    }
    yAxis = navXRollAngle/20;
    if(autoBalanceMaxAngle * navXRollAngle < 0) autoBalanceMaxAngle = 0;
  }

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

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  public boolean autoDrive = false;
  public double step = 1;
  public double autoDriveDistance;
  public double currentDistance;
  public double distanceOffset;
  @Override
  public void autonomousPeriodic() {
    autoSetting = 0;

    // Auto Setting 1
    if (autoSetting == 0){

      if (step == 1){
        // Place cube
        intakeSwitch = 1;
        myArm.Outtake(intakeSwitch);
        if(!robotWait(1)){
          step += 1;
          myArm.Stoptake();
        }
      }
      
      if (step == 2){
        // Drive forwards
        if(!robotDriveTo(2,0,1)) step +=1;
      }
  
      if (step == 3){
        // Wait 1 second
        if(!robotWait(1)) step += 1;
      }
  
      if (step == 4){
        // Drive Backwards
        if(!robotDriveTo(5,0,-1)) step +=1;
      }
  
      if (step == 5){
        autoBalance();
      }
    }

    if(autoSetting == 1){
      // Move Arm All the way out
      if(step == 1){
        if(!myArm.Bswitchlong.get()){
          myArm.armHigh();
        }else{
          step += 1;
        }
      }

      // Place cone
      if(step == 2){
        intakeSwitch = -1;
        myArm.Outtake(intakeSwitch);
        if(!robotWait(3)) step += 1;
      }

      // Driveforwards
      if(step == 3){
        if(!robotDriveTo(2, 0, 0.5)) step += 1;
      }
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
        
      // Rev Adjustment
      if (rev < 1){
        double revFix = getMaxMag(xAxis, yAxis, zAxis);
        if(revFix > 1) revFix = 1;
        rev = rev + 0.02/revFix;
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
    }
  

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    SmartDashboard.putNumber("llDistance: ", llDistance);
    SmartDashboard.putNumber("llAngle: ", llAngle);
    SmartDashboard.putNumber("navXYawAngle: ", navXYawAngle);
    SmartDashboard.putNumber("ArmEncoder: ", myArm.getArmEncoderReading());
    SmartDashboard.putBoolean("Intake", true);

    //Arm
    SmartDashboard.putString("ArmLevel: ", "Front");
    SmartDashboard.putBoolean("Fswitchlong: ", myArm.Fswitchlong.get());
    SmartDashboard.putBoolean("Bswitchlong: ", myArm.Bswitchlong.get());
    SmartDashboard.putBoolean("Fswitchshort: ", myArm.Fswitchshort.get());
    SmartDashboard.putBoolean("Bswitchshort: ", myArm.Bswitchshort.get());

    SmartDashboard.putNumber("POV: ", controller.getPOV());
    SmartDashboard.putBoolean("Flip180", controller.getRawButton(8));
    SmartDashboard.putNumber("FLE", cornerFL.getEncoderPosition());
    SmartDashboard.putNumber("FRE", cornerFR.getEncoderPosition());
    SmartDashboard.putNumber("BLE", cornerBL.getEncoderPosition());
    SmartDashboard.putNumber("BRE", cornerBR.getEncoderPosition());
    SmartDashboard.putNumber("navXRollAngle", navXRollAngle);

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    cornerFL.setEncoder();
    cornerFR.setEncoder();
    cornerBL.setEncoder();
    cornerBR.setEncoder();
  }

   public double rev = 0.08;
  @Override
  public void teleopPeriodic() {
    //Smart Dashboard
    boolean test = true;
    if (intakeSwitch == 1) {
      test = true;
    }
    else {
      test = false;
    }
    SmartDashboard.putNumber("llDistance: ", llDistance);
    SmartDashboard.putNumber("llAngle: ", llAngle);
    SmartDashboard.putNumber("navXYawAngle: ", navXYawAngle);
    SmartDashboard.putNumber("ArmEncoder: ", myArm.getArmEncoderReading());
    SmartDashboard.putBoolean("Intake", test);

    //Arm
    if(myArm.LongArm) SmartDashboard.putString("ArmLevel: ", "Front");
    else SmartDashboard.putString("ArmLevel: ", "Back");
    SmartDashboard.putBoolean("Fswitchlong: ", myArm.Fswitchlong.get());
    SmartDashboard.putBoolean("Bswitchlong: ", myArm.Bswitchlong.get());
    SmartDashboard.putBoolean("Fswitchshort: ", myArm.Fswitchshort.get());
    SmartDashboard.putBoolean("Bswitchshort: ", myArm.Bswitchshort.get());

    SmartDashboard.putNumber("POV: ", controller.getPOV());
    SmartDashboard.putBoolean("Flip180", controller.getRawButton(8));
    SmartDashboard.putNumber("FLE", cornerFL.getEncoderPosition());
    SmartDashboard.putNumber("FRE", cornerFR.getEncoderPosition());
    SmartDashboard.putNumber("BLE", cornerBL.getEncoderPosition());
    SmartDashboard.putNumber("BRE", cornerBR.getEncoderPosition());
    SmartDashboard.putNumber("navXRollAngle", navXRollAngle);
    
    // Drive Code
    xAxis = controller.getRawAxis(0);
    yAxis = controller.getRawAxis(1) * -1;
    zAxis = controller.getRawAxis(4);
    wAxis = controller.getRawAxis(2) - controller.getRawAxis(3);

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
      myArm.Intake(intakeSwitch);
    }else if (controller.getRawButton(6)){
      myArm.Outtake(intakeSwitch);
    }else{
      myArm.Stoptake();
    }
    
    // Switch intake for cone and cube
    if(controller.getRawButtonPressed(4)) intakeSwitch *= -1;

    //Macros 
    if(controller.getPOV() > 180 && controller.getPOV() != -1) leftHumanPlayerAuto();
    if(controller.getPOV() < 180 && controller.getPOV() != -1) rightHumanPlayerAuto();
    if(controller.getRawButton(8)) flip180();
    if(controller.getRawButton(7)) resetnavX();
    if(controller.getRawButton(2)) autoBalance();
    if(controller.getRawButton(3)) myArm.armMed();

    //Limelight
    double[] llArray = NetworkTableInstance.getDefault().getTable("limelight").getEntry("camerapose_targetspace").getDoubleArray(new double[6]);
    llDistance = 0;
    //llDistance = llArray[2];
    llDistance *= -3.281; // Unit coneversion meters -> feet
    llAngle = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    

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
      
      // Rev Adjustment
      if (rev < 1){
        double revFix = getMaxMag(xAxis, yAxis, zAxis);
        if(revFix > 1) revFix = 1;
        rev = rev + 0.02/revFix;
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
    navXYawAngle = navX.getYaw() - navXYawOffset;
    navXRollAngle = navX.getRoll() - navXRollOffset;
    navXPitchAngle = navX.getPitch() - navXPitchOffset;
  }

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
    
  }

  @Override
  public void testExit() {}
}
