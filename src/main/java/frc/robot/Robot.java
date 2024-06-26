//                                   ,█▌ ▄█, █▀█ █▀█
//                     ;▄▄▄▄████▀▀═ ╘▀▀█^█▄█J█▄█ █▄█ `▀▀████▄▄▄▄▄▄╓
//             ,▄▄███▀▀▀"`       ,                          `¬█▀ ▐▌
//         ╓▄██▀▀'          ,▄▄█▀▀█▌    ;;                    └▀▀▀'     █▀▀█L
//      ▄██▀▀          ███▀▀▀▀▀▀█▄▀^  j█'└█▄▄▄▄▄▄▄▄▄▄▄▄;                ▀█▄██▄
//    ▄█▀-          ▄▄▄█⌐              ▀&█▀      `-`└▀▀▀▀▀▀███▄▄▄▄          "▀█▄
//   ██└      ╓▄▄██▀▀▀¬                                        ¬▀▀▀██▄▄▄      └██
//  ██    ╓▄██▀▀                                                      ▀▀██▄ç    ██
// ▐█   ▄█▀▀                                                              '▀█▄  ██▌
// █▌ ▄█▀                  *** 2023 Complete Robot Code ***                  ▀███▐█
// ███▀                           Lead: Chet Petro                           ▄█▀ ▐█
// ▐██       ▄▄                                                           ▄██▀   █▌
//  ▐█▄     █"'█▄                                                    ,▄▄██▀¬   ,█▀
//   ▀█▄    ▀▀▀▀▀███▄▄ç                                        ╓▄▄███▀▀¬      ▄█▀
//    '██▀█⌐    ▄▄▄;╘▀▀▀▀████▄▄▄▄▄▄;,,,        ,,,;▄▄▄▄▄▄▄███▀▀▀▀╘  ▄▄▄   ▄█▀██▀
//     █▄ ▄▌    █▄"▀█   ▄⌐    "-'▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀▀'-¬     ▄▄▄ ▐▀██╘    █▄,▄▌
//      ¬└¬     ▄▀▀█▄  ███   █  █⌐  █ ▄▄▄▄▄  ╓▄▄     ████▄ ╓█▀▀▀▌  ▐█      --`
//              ▀█▄▄█⌐▐█ ▐▌  █  ██.j█ ▀▀█▀▀^██▀▀█    █  ▄█ █▌  ▐█  ▐█
//                 `  █▀▀██µ █  █▐█d█   █▌  ██▄▄     ██▀▀█ █▌  █▌  └▀
//                        ▐█ █  █ ▀██   █▌   "▀▀█L   █ ,▄█ ▀██▀▀
//                           ▀  ▀  ▀█   █▌  ██▄▄█-   █▀▀▀

//~~~~~~~~~~~~~~IMPORTS~~~~~~~~~~~~~~~~~

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.networktables.*;

//~~~~~~~~~~~~~~Initiating~~~~~~~~~~~~~~~~~

public class Robot extends TimedRobot {
  // Joystick
  Joystick controller = new Joystick(0);
  double xAxis;
  double yAxis;
  double zAxis;
  double wAxis;
  static double DEADBAND = 0.1;
  public int intakeSwitch = 1;

  // Drive Station
  Joystick driveStation = new Joystick(1);

  // navX
  AHRS navX = new AHRS();
  double navXYawAngle = 0;
  double navXYawOffset = 0;
  double navXRollOffset = 0;
  double navXRollAngle = 0;
  double navXPitchOffset = 0;
  double navXPitchAngle = 0;
  double navXRate = 0;

  // Auto
  Timer timer = new Timer();
  boolean waitingTimer = false;
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

  // Robot Components
  SwervCorner cornerFL = new SwervCorner(5, 6, 12, 1, 1, 9.4921875, -1);
  SwervCorner cornerFR = new SwervCorner(2, 1, 9, 1, -1, 201.121875, 1);
  SwervCorner cornerBL = new SwervCorner(7, 8, 11, -1, 1, 289.86328125, -1);
  SwervCorner cornerBR = new SwervCorner(4, 3, 10, -1, -1, 5.18547, 1);
  Arm myArm = new Arm();

  //Stuff for Auto Init, The 3 switches and the resulting auto number
  DigitalInput switch1 = new DigitalInput(3);
  DigitalInput switch2 = new DigitalInput(4);
  DigitalInput switch3 = new DigitalInput(5);
  int autoSelect = 0;

  //Stuff for during auto
  public boolean autoDrive = false;
  public double step = 1;
  public double autoDriveDistance;
  public double currentDistance;
  public double distanceOffset;
  public boolean autoDriveStep = true;
  public int flipCount = 1;
  public double prevAx = 0;
  boolean robotIntake = false;
  boolean robotOutake = false;

  //Rev Init
  public double rev = 0.08;
  
  //Empty array Init
  double[] myarray = { 0, 0, 0, 0, 0, 0 };



  // *********************************************************************** */
  // Custom Functions
  // *********************************************************************** */

  public boolean robotWait(double waitTimeSeconds) {
    if (!waitingTimer) {
      waitingTimer = true;
      timer.reset();
      timer.start();
      return true;
    } else if (timer.get() < waitTimeSeconds) {
      return true;
    } else {
      waitingTimer = false;
      return false;
    }
  }

  public boolean robotTurnTo(double degree) {
    xAxis = 0;
    yAxis = 0;
    double clockwise = ((navXYawAngle % 360) + degree) % 360;
    double anticlockwise = ((navXYawAngle % 360) - degree) % 360;
    if (navXYawAngle < degree - 1 || navXYawAngle > degree + 1) {
      if (clockwise > anticlockwise) {
        zAxis = anticlockwise * -0.1;
      }
      if (clockwise < anticlockwise) {
        zAxis = clockwise / 4 * 0.1;
      }
      return true;
    } else {
      return false;
    }
  }

  public boolean robotDriveTo(double distance, double xAutoDrive, double yAutoDrive) {
    if (autoDrive == false) {
      autoDrive = true;
      autoDriveDistance = distance;
      cornerFR.resetEncoder();
      cornerFL.resetEncoder();
      cornerBR.resetEncoder();
      cornerBL.resetEncoder();
    }

    if (autoDrive && (autoDriveDistance > 0)) {
      zAxis = 0;
      xAxis = xAutoDrive * 0.4;
      yAxis = yAutoDrive * 0.4;
      currentDistance = currentDistance - Math.min(cornerBL.getDriveEncoderPosition(), Math.min(cornerBR.getDriveEncoderPosition(), Math.min(cornerFR.getDriveEncoderPosition(),  cornerFL.getDriveEncoderPosition())));
      return true;
    } else {
      autoDrive = false;
      zAxis = 0;
      xAxis = 0;
      yAxis = 0;
      return false;
    }
  }

  public void resetnavX(double d) {
    navXYawOffset = (navX.getYaw() + d) % 360;
    if (navXYawOffset > 180) {
      navXYawOffset = -(360 - navXYawOffset);
    }
    navXPitchOffset = navX.getPitch();
    navXRollOffset = navX.getRoll();
    myArm.resetArmEncoderOffset();
  }

  public boolean leftHumanPlayerAuto() {
    if (!robotTurnTo(0) && rotateBeforeCalc) {
      calculate = true;
      return true;
    } else {
      if (calculate) {
        rotateBeforeCalc = false;
        calculate = false;
        vx = llDistance * Math.cos(Math.toRadians(90 - llAngle)) - 4.1 * Math.sin(Math.toRadians(39.66666));
        vy = llDistance * Math.sin(Math.toRadians(90 - llAngle)) - 4.1 * Math.cos(Math.toRadians(39.66666));
        thetaF = Math.toDegrees(Math.atan(vx / vy));
        hyp = Math.sqrt(Math.pow(vx, 2) + Math.pow(vy, 2));
      }
      if (!robotDriveTo(hyp, Math.cos(Math.toRadians(thetaF)), Math.sin(Math.toRadians(thetaF))) && !myArm.armHigh()) {
        return false;
      } else {
        rotateBeforeCalc = true;
        return true;
      }
    }
  }

  public boolean rightHumanPlayerAuto() {
    if (!robotTurnTo(0) && rotateBeforeCalc) {
      calculate = true;
      return true;
    } else {
      if (calculate) {
        rotateBeforeCalc = false;
        calculate = false;
        vx = llDistance * Math.cos(Math.toRadians(90 - llAngle)) + 4.1 * Math.sin(Math.toRadians(39.66666));
        vy = llDistance * Math.sin(Math.toRadians(90 - llAngle)) - 4.1 * Math.cos(Math.toRadians(39.66666));
        thetaF = Math.toDegrees(Math.atan(vx / vy));
        hyp = Math.sqrt(Math.pow(vx, 2) + Math.pow(vy, 2));
      }
      if (!robotDriveTo(hyp, Math.cos(Math.toRadians(thetaF)), Math.sin(Math.toRadians(thetaF))) && !myArm.armHigh()) {
        return false;
      } else {
        rotateBeforeCalc = true;
        return true;
      }
    }
  }

  public void flip180() {
    if (Math.abs(navXYawAngle) > 90) {
      robotTurnTo(180);
    } else {
      robotTurnTo(0);
    }
  }

  public boolean autoBalance() {
    if(Math.abs(navXRollAngle) > 6){
      yAxis = -0.5;
      return false;
    }else{
      zAxis = 1; // lock wheels 
      return true;
    }
  }

  public double angleThresh = 3;
  public double speedGain = 0.5;

  public boolean autoBalanceManual(){
    if (Math.abs(navXRollAngle) < angleThresh){
      yAxis = 0;
      return true;
    }
    navXRate = navX.getRawGyroY();
    yAxis = speedGain * navXRollAngle * navXRate;
    return false;
  }

  /*
   *   public void autoBalanceManual(){
    if(autoDriveStep){
      if(robotWait(1/flipCount)){
        if(Math.abs(navXRollAngle)> 1){
          yAxis = -(navXRollAngle / (45 + flipCount * 10));
        }
      }else{
        autoDriveStep = false;
      }
    }else{
      if(robotWait(1)){
        //wait
      }else{
        autoDriveStep = true;
      }
    }
    if(navXRollAngle * prevAx < 0){
      flipCount += 2;
      prevAx = navXRollAngle;
    }
  }
   */
  

  public double getMaxMag(double x, double y, double z) {
    return Math.max(Math.max(cornerFL.driveMagnitude, cornerFR.driveMagnitude),
        Math.max(cornerBL.driveMagnitude, cornerBR.driveMagnitude));
  }

  public double normalize(double x, double y, double z, double zvecX, double zvecY) {
    double correctedX;
    double correctedY;
    if (x != 0 || y != 0) {
      double startHypotenuse = Math.sqrt((Math.pow(x, 2) + (Math.pow(y, 2))));
      double startAngle = Math.toDegrees(Math.acos(x / startHypotenuse));
      if (y < 0)
        startAngle = 360 - startAngle;
      correctedX = Math.cos(Math.toRadians(startAngle + navXYawAngle)) * startHypotenuse;
      correctedY = Math.sin(Math.toRadians(startAngle + navXYawAngle)) * startHypotenuse;
    } else {
      correctedX = 0;
      correctedY = 0;
    }

    double maxMag = getMaxMag(x, y, z);
    double totalX = correctedX + (zvecX * z) / maxMag;
    double totalY = correctedY + (zvecY * z) / maxMag;
    double hypotenuse = Math.sqrt((Math.pow(totalX, 2) + (Math.pow(totalY, 2))));
    return hypotenuse;
  }

  public double getSpeed(double x, double y, double z, SwervCorner corner) {
    if (getMaxMag(x, y, z) > 1) {
      double speed = normalize(x, y, z, corner.zvecX, corner.zvecY);
      return speed;
    } else {
      double speed = corner.driveMagnitude;
      return speed;
    }
  }

  @Override
  public void robotInit() {

  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {

    //Converting the switches to binary
    if (!switch1.get()) {
      autoSelect += 1;
    }
    if (!switch2.get()) {
      autoSelect += 2;
    }
    if (!switch3.get()) {
      autoSelect += 4;
    }
    SmartDashboard.putNumber("AutoSelect", autoSelect);
  }

  @Override
  public void autonomousPeriodic() {
    SmartDashboard.putNumber("Step", step);

    //############################
    //Auto 0 Putting a cone high and drive straight forward
    //############################
    
    if (autoSelect == 0) {
      //Raising arm to top
      if (step == 1) {
        if (myArm.armHigh()) {
          step += 1;
          myArm.switchArm();
          resetnavX(180);
        }
      }

      // Place cone
      if (step == 2) {
        myArm.Outtake(1);
        if (!robotWait(0.5)) {
          step += 1;
          myArm.Stoptake();
        }
      }

      //Go Forward
      if(step == 3){
        if (!robotDriveTo(15, 0, 1)){
          step += 1;
        }else{
          myArm.armBack();
        }
      }
    }

    //############################
    //Auto 1: Putting a cube low
    //############################

    if(autoSelect == 1){
    // Place cube
      if (step == 1) {
        myArm.Outtake(-1);
        if (!robotWait(0.5)) {
          step += 1;
          myArm.Stoptake();
          resetnavX(0);
        }
      }

      //Drive forward
      if (step == 2) {
        if (!robotDriveTo(15, 0, 1)){
          step += 1;
        }else{
          myArm.armBack();
        }
      }
    }
    //############################
    //Auto 2: Putting a cone mid
    //############################

    if (autoSelect == 2) {
      //Calibrate Arm
      if (step == 1) {
        if (myArm.armBack()) {
          step += 1;
          resetnavX(180);
        }
      }

      //Go to Medium Height and switch to top arm
      if (step == 2) {
        if (myArm.armMed()) {
          step += 1;
          myArm.switchArm();
        }
      }

      // Place cone
      if (step == 3) {
        myArm.Outtake(1);
        if (!robotWait(0.5)) {
          step += 1;
          myArm.Stoptake();
        }
      }

      //Go Forward
      if(step == 4){
        if (!robotDriveTo(15, 0, 1)){
          step += 1;
        }else{
          myArm.armBack();
        }
      }
    }

    //############################
    //Auto 3: Putting a cube mid
    //############################

    if (autoSelect == 3) {
      //Calibrate Arm
      if (step == 1) {
        if (myArm.armBack()) {
          step += 1;
          resetnavX(180);
        }
      }

      //Go to Medium Height and switch to top arm
      if (step == 2) {
        if (myArm.armMed()) {
          step += 1;
          myArm.switchArm();
        }
      }

      // Place cube
      if (step == 3) {
        myArm.Outtake(-1);
        if (!robotWait(0.5)) {
          step += 1;
          myArm.Stoptake();
        }
      }

      //Go Forward
      if(step == 4){
        if (!robotDriveTo(15, 0, 1)){
          step += 1;
        }else{
          myArm.armBack();
        }
      }

    }

    //############################
    //Auto 4: Putting a cone high and auto balance
    //############################

    if (autoSelect == 4) {
      //Raising arm to top
      if (step == 1) {
        if (myArm.armHigh()) {
          step += 1;
          myArm.switchArm();
          resetnavX(180);
        }
      }

      // Place cone
      if (step == 2) {
        myArm.Outtake(1);
        if (!robotWait(0.5)) {
          step += 1;
          myArm.Stoptake();
        }
      }

      //Moving Forward
      if (step == 3) {
        if (!robotDriveTo(15, 0, 1)){
          step += 1;
        }else{
          myArm.armBack();
        }
      }


      // Wait 1 second
      if(step == 4){
        if(!robotWait(1)){
          step += 1;
        }
      }

      //Moving Backward
      if (step == 5) {
        if (!robotDriveTo(9.5, 0, -1))
          step += 1;
      }


      // Tip charge station
      if (step == 6){
        if(!autoBalance())
          step += 1;
      }

      // Move forwards 
      if (step == 7) {
        if (!robotDriveTo(1, 0, 1)){
          step += 1;
        }
      }
    }

    //############################
    //Auto 5: Putting a cube high
    //############################

    if (autoSelect == 5) {
      //Raising arm to top
      if (step == 1) {
        if (myArm.armHigh()) {
          step += 1;
          myArm.switchArm();
          resetnavX(180);
        }
      }

      // Place cube
      if (step == 2) {
        myArm.Outtake(-1);
        if (!robotWait(0.5)) {
          step += 1;
          myArm.Stoptake();
        }
      }

      //Go Forward
      if(step == 3){
        if (!robotDriveTo(15, 0, 1)){
          step += 1;
        }else{
          myArm.armBack();
        }
      }
    }

    //############################
    //Rest Of Auto Code
    //############################

    if (xAxis != 0 || yAxis != 0 || zAxis != 0) {
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
      cornerFR.driveSpeed(getSpeed(xAxis, yAxis, zAxis, cornerFR) * rev);
      cornerBL.driveSpeed(getSpeed(xAxis, yAxis, zAxis, cornerBL) * rev);
      cornerBR.driveSpeed(getSpeed(xAxis, yAxis, zAxis, cornerBR) * rev);

      // Rev Adjustment
      if (rev < 1) {
        double revFix = getMaxMag(xAxis, yAxis, zAxis);
        if (revFix > 1)
          revFix = 1;
        rev = rev + 0.02 / revFix;
      }

    } else {
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

    // navX
    navXYawAngle = navX.getYaw() - navXYawOffset;
    navXRollAngle = navX.getRoll() - navXRollOffset;
    navXPitchAngle = navX.getPitch() - navXPitchOffset;
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    SmartDashboard.putNumber("llDistance: ", llDistance);
    SmartDashboard.putNumber("llAngle: ", llAngle);
    SmartDashboard.putNumber("navXYawAngle: ", navXYawAngle);
    SmartDashboard.putNumber("ArmEncoder: ", myArm.getArmEncoderReading());
    SmartDashboard.putBoolean("Intake", true);

    // Arm
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

    cornerFL.setEncoder();
    cornerFR.setEncoder();
    cornerBL.setEncoder();
    cornerBR.setEncoder();
  }

  @Override
  public void teleopPeriodic() {
    // Smart Dashboard
    boolean test = true;
    if (intakeSwitch == 1) {
      test = true;
    } else {
      test = false;
    }
    SmartDashboard.putNumber("llDistance: ", llDistance);
    SmartDashboard.putNumber("llAngle: ", llAngle);
    SmartDashboard.putNumber("navXYawAngle: ", navXYawAngle);
    SmartDashboard.putNumber("ArmEncoder: ", myArm.getArmEncoderReading());
    SmartDashboard.putBoolean("Intake", test);

    // Arm
    if (myArm.LongArm)
      SmartDashboard.putString("ArmLevel: ", "Front");
    else
      SmartDashboard.putString("ArmLevel: ", "Back");
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
    if (Math.abs(xAxis) < DEADBAND)
      xAxis = 0;
    if (Math.abs(yAxis) < DEADBAND)
      yAxis = 0;
    if (Math.abs(zAxis) < DEADBAND)
      zAxis = 0;
    if (Math.abs(wAxis) < DEADBAND)
      wAxis = 0;
    zAxis = zAxis * 0.5;

    if(controller.getRawButtonPressed(5)){
      robotIntake = true;
      robotOutake = false;
    }
    if(controller.getRawButtonPressed(6)){
      robotOutake = true;
      robotIntake = false;
    }
    if(controller.getRawButton(6) && controller.getRawButton(5)){
      robotIntake = false;
      robotOutake = false;
    }


    myArm.moveArm(wAxis);
    if (controller.getRawButtonPressed(1)) {
      myArm.switchArm();
    }
    if (robotIntake) {
      myArm.Intake(intakeSwitch);
    } else if (robotOutake) {
      myArm.Outtake(intakeSwitch);
    } else {
      myArm.Stoptake();
    }

    // Switch intake for cone and cube
    if (controller.getRawButtonPressed(4))
      intakeSwitch *= -1;

    // Macros
    if (controller.getPOV() > 180 && controller.getPOV() != -1)
      leftHumanPlayerAuto();
    if (controller.getPOV() < 180 && controller.getPOV() != -1)
      rightHumanPlayerAuto();
    if (controller.getRawButton(7))
      resetnavX(0);
    if (controller.getRawButton(2))
      autoBalanceManual();
    if (controller.getRawButton(3))
      myArm.armMed();

    // Limelight
    double[] llArray = NetworkTableInstance.getDefault().getTable("limelight").getEntry("camerapose_targetspace")
        .getDoubleArray(myarray);
    llDistance = llArray[2];
    llDistance *= -3.281; // Unit coneversion meters -> feet
    llAngle = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);

    // Encoder Limit Switch Fix
    if (controller.getRawButtonPressed(8)){
      myArm.useEncoderValues = true;
    }



    if (xAxis != 0 || yAxis != 0 || zAxis != 0) {
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
      cornerFR.driveSpeed(getSpeed(xAxis, yAxis, zAxis, cornerFR) * rev);
      cornerBL.driveSpeed(getSpeed(xAxis, yAxis, zAxis, cornerBL) * rev);
      cornerBR.driveSpeed(getSpeed(xAxis, yAxis, zAxis, cornerBR) * rev);

      // Rev Adjustment
      if (rev < 1) {
        double revFix = getMaxMag(xAxis, yAxis, zAxis);
        if (revFix > 1)
          revFix = 1;
        rev = rev + 0.02 / revFix;
      }

    } else {
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

    // navX
    navXYawAngle = navX.getYaw() - navXYawOffset;
    navXRollAngle = navX.getRoll() - navXRollOffset;
    navXPitchAngle = navX.getPitch() - navXPitchOffset;
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {

  }

  @Override
  public void testExit() {
  }
}
