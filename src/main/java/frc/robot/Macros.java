package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.networktables.*;

public class Macros extends Robot {
    public boolean TurnTo(double deg, SwervCorner FL,SwervCorner FR, SwervCorner BL, SwervCorner BR, AHRS navX) {

        double yaw = navX.getYaw();
        if (yaw != deg) {
            double xAxis = 0;
            double yAxis = 0;
            double navXYawAngle = deg;
            double clockwise = (yaw + deg) % 360;
            double anticlockwise = (yaw - deg) % 360;
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
            return false;
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
            return true;
        }
        
    }
    public double flip180(SwervCorner FL,SwervCorner FR, SwervCorner BL, SwervCorner BR, AHRS navX){
        double yaw = navX.getYaw();
        double deg = 180 ; 
        deg=(yaw-180)%360;
        return deg;
    }  
    

    
    public void armMed(VictorSPX LongArmMotor,DigitalInput Fswitchlong,DigitalInput Bswitchlong,DigitalInput Fswitchshort,DigitalInput Bswitchshort,Encoder Armcoder){
        if(Armcoder.getDistance()<=1) {
            LongArmMotor.set(ControlMode.PercentOutput, 0.25);
        }
        else if(Armcoder.getDistance()>=1) {
            LongArmMotor.set(ControlMode.PercentOutput, -0.25);
        }
        else {LongArmMotor.set(ControlMode.PercentOutput, 0.06);} 
    }
    public void armHigh(VictorSPX LongArmMotor,DigitalInput Fswitchlong,DigitalInput Bswitchlong,DigitalInput Fswitchshort,DigitalInput Bswitchshort,Encoder Armcoder){
        if (Armcoder.getDistance()<2) {
            LongArmMotor.set(ControlMode.PercentOutput, 0.25);
        }
        else if (Armcoder.getDistance()>2) {
            LongArmMotor.set(ControlMode.PercentOutput, 0.25);
        }
        else {LongArmMotor.set(ControlMode.PercentOutput, 0.06);}
    }
    }
    




