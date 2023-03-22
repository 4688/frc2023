package frc.robot;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.networktables.*;

public class Macros extends Robot {
    public void turnto(double degree, SwervCorner cornerFL, SwervCorner cornerFR, SwervCorner cornerBL,
            SwervCorner cornerBR, AHRS navX) {

        double yaw = navX.getYaw();
        if (yaw != degree) {
            double xAxis = 0;
            double yAxis = 0;
            double navXYawAngle = degree;
            double clockwise = (yaw + degree) % 360;
            double anticlockwise = (yaw - degree) % 360;
            double zAxis = 0.25;
            if (clockwise > anticlockwise) {
                zAxis = zAxis * -1;
            }
            if (clockwise < anticlockwise) {
                zAxis = zAxis * 1;
            }
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
            cornerFL.driveSpeed(getSpeed(xAxis, yAxis, zAxis, cornerFL));
            cornerFR.driveSpeed(getSpeed(xAxis, yAxis, zAxis, cornerFR));
            cornerBL.driveSpeed(getSpeed(xAxis, yAxis, zAxis, cornerBL));
            cornerBR.driveSpeed(getSpeed(xAxis, yAxis, zAxis, cornerBR));
        }

        else {
            cornerFL.stopRotation();
            cornerFR.stopRotation();
            cornerBL.stopRotation();
            cornerBR.stopRotation();

            cornerFL.driveSpeed(0);
            cornerFR.driveSpeed(0);
            cornerBL.driveSpeed(0);
            cornerBR.driveSpeed(0);
        }
    }

    public double flip180(double degree, SwervCorner cornerFL, SwervCorner cornerFR, SwervCorner cornerBL,
            SwervCorner cornerBR, AHRS navX) {
        double yaw = navX.getYaw();
        if ((yaw > 270) || (yaw < 90)) {
            return 180;
        } else {
            return 0;
        }

    }
    public void armMed(VictorSPX LongArmMotor,DigitalInput Fswitchlong,DigitalInput Bswitchlong,DigitalInput Fswitchshort,DigitalInput Bswitchshort,Encoder Armcoder){
        while(Armcoder.getDistance()<1) {
            LongArmMotor.set(ControlMode.PercentOutput, 0.25);
        }
        LongArmMotor.set(ControlMode.PercentOutput, 0.06); 
    }
    public void armHigh(VictorSPX LongArmMotor,DigitalInput Fswitchlong,DigitalInput Bswitchlong,DigitalInput Fswitchshort,DigitalInput Bswitchshort,Encoder Armcoder){
        while(Armcoder.getDistance()<2) {
            LongArmMotor.set(ControlMode.PercentOutput, 0.25);
        }
        LongArmMotor.set(ControlMode.PercentOutput, 0.06); 
    }
    public void AprilTags(double degree, SwervCorner cornerFL, SwervCorner cornerFR, SwervCorner cornerBL,SwervCorner cornerBR, AHRS navX) {
        double myVal = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDouble(0);
        double zAxis =  NetworkTableInstance.getDefault().getTable("limelight").getEntry("camerapose_targetspace").getDouble(0);

        if (myVal != 1 || myVal != 2|| myVal != 3 || myVal != 4 || myVal != 5 || myVal != 6 || myVal != 7 ||myVal != 8) {
            flip180(kDefaultPeriod, cornerFL, cornerFR, cornerBL, cornerBR, navX);
            if (zAxis > 1) {
                cornerFL.driveSpeed(0.25);
                cornerFR.driveSpeed(0.25);
                cornerBL.driveSpeed(0.25);
                cornerBR.driveSpeed(0.25);
            } else {
                cornerFL.driveSpeed(0);
                cornerFR.driveSpeed(0);
                cornerBL.driveSpeed(0);
                cornerBR.driveSpeed(0);
            }
        }
        else { 
            if (zAxis < 1) {
                cornerFL.driveSpeed(0.25);
                cornerFR.driveSpeed(0.25);
                cornerBL.driveSpeed(0.25);
                cornerBR.driveSpeed(0.25);
            } else {
                cornerFL.driveSpeed(0);
                cornerFR.driveSpeed(0);
                cornerBL.driveSpeed(0);
                cornerBR.driveSpeed(0);
            }


        }
    }
}