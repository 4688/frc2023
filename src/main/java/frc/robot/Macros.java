// package frc.robot;

// public class Macros extends Robot {

//     public boolean robotTurnTo(double degree) {
//         xAxis = 0;
//         yAxis = 0;
//         double clockwise = (navXYawAngle + degree) % 360;
//         double anticlockwise = (navXYawAngle - degree) % 360;
//         double zAxis = 0.25;
//         if (navXYawAngle != degree) {
//             if (clockwise > anticlockwise) {
//                 zAxis = zAxis * -1;
//             }
//             if (clockwise < anticlockwise) {
//                 zAxis = zAxis * 1;
//             }
//             return true;
//         }else{
//             return false;
//         }
//     }

//     public boolean robotDriveTo(double distance, double x, double y){
//         if(autoDrive == false){
//             autoDrive = true;
//             autoDriveDistance = distance;
//             distanceOffset = cornerBL.getDriveEncoderPosition();
//             currentDistance = cornerBL.getDriveEncoderPosition();
//         }

//         if(autoDrive && (Math.abs(currentDistance-distanceOffset) <= autoDriveDistance)){
//             zAxis = 0;
//             xAxis = x*(1 - Math.abs(currentDistance-distanceOffset)/autoDriveDistance);
//             yAxis = y*(1 - Math.abs(currentDistance-distanceOffset)/autoDriveDistance);
//             currentDistance = cornerBL.getDriveEncoderPosition();
//             return true;
//           } else {
//             autoDrive = false;
//             return false;
//         }
//     }

//     public void resetnavX(){
//         navXYawOffset = navX.getYaw();
//         navXRollOffset = navX.getPitch();
//     }

//     public boolean leftHumanPlayerAuto(){
//         if(!robotTurnTo(0) && rotateBeforeCalc){
//             calculate = true;
//             return true;
//         }else{
//             if(calculate){
//                 rotateBeforeCalc = false;
//                 calculate = false;
//                 vx = llDistance*Math.cos(Math.toRadians(90-llAngle))-4.1*Math.sin(Math.toRadians(39.66666));
//                 vy = llDistance*Math.sin(Math.toRadians(90-llAngle))-4.1*Math.cos(Math.toRadians(39.66666));
//                 thetaF = Math.toDegrees(Math.atan(vx/vy));
//                 hyp = Math.sqrt(Math.pow(vx, 2)+Math.pow(vy, 2));
//             }
//             if(!robotDriveTo(hyp, Math.cos(Math.toRadians(thetaF)), Math.sin(Math.toRadians(thetaF))) && !myArm.armHigh()){
//                 return false;
//             }else{
//                 rotateBeforeCalc = true;
//                 return true;
//             }
//         }
//     }

//     public boolean rightHumanPlayerAuto(){
//         if(!robotTurnTo(0) && rotateBeforeCalc){
//             calculate = true;
//             return true;
//         }else{
//             if(calculate){
//                 rotateBeforeCalc = false;
//                 calculate = false;
//                 vx = llDistance*Math.cos(Math.toRadians(90-llAngle))+4.1*Math.sin(Math.toRadians(39.66666));
//                 vy = llDistance*Math.sin(Math.toRadians(90-llAngle))-4.1*Math.cos(Math.toRadians(39.66666));
//                 thetaF = Math.toDegrees(Math.atan(vx/vy));
//                 hyp = Math.sqrt(Math.pow(vx, 2)+Math.pow(vy, 2));
//             }
//             if(!robotDriveTo(hyp, Math.cos(Math.toRadians(thetaF)), Math.sin(Math.toRadians(thetaF))) && !myArm.armHigh()){
//                 return false;
//             }else{
//                 rotateBeforeCalc = true;
//                 return true;
//             }
//         }
//     }
    

//     public double flip180() {
//         double yaw = navX.getYaw();
//         if ((yaw > 270) || (yaw < 90)) {
//             return 180;
//         } else {
//             return 0;
//         }
//     }

//     public void AprilTags(double degree, SwervCorner cornerFL, SwervCorner cornerFR, SwervCorner cornerBL,SwervCorner cornerBR, AHRS navX) {
//         double myVal = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDouble(0);
//         double zAxis =  NetworkTableInstance.getDefault().getTable("limelight").getEntry("camerapose_targetspace").getDouble(0);

//         if (myVal != 1 || myVal != 2|| myVal != 3 || myVal != 4 || myVal != 5 || myVal != 6 || myVal != 7 ||myVal != 8) {
//             flip180();
//             if (zAxis > 1) {
//                 cornerFL.driveSpeed(0.25);
//                 cornerFR.driveSpeed(0.25);
//                 cornerBL.driveSpeed(0.25);
//                 cornerBR.driveSpeed(0.25);
//             } else {
//                 cornerFL.driveSpeed(0);
//                 cornerFR.driveSpeed(0);
//                 cornerBL.driveSpeed(0);
//                 cornerBR.driveSpeed(0);
//             }
//         }
//         else { 
//             if (zAxis < 1) {
//                 cornerFL.driveSpeed(0.25);
//                 cornerFR.driveSpeed(0.25);
//                 cornerBL.driveSpeed(0.25);
//                 cornerBR.driveSpeed(0.25);
//             } else {
//                 cornerFL.driveSpeed(0);
//                 cornerFR.driveSpeed(0);
//                 cornerBL.driveSpeed(0);
//                 cornerBR.driveSpeed(0);
//             }
//         }
//     }

//     public void autoBalance(){
//         zAxis = 0;
//         xAxis = 0;
//         yAxis = navXRollAngle/100;
//     }
// }