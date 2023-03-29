package frc.robot;

import java.beans.Encoder;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class Arm {
    VictorSPX LongArmMotor;
    VictorSPX LongIntake;
    VictorSPX ShortArmMotor;
    VictorSPX ShortIntake;
    DigitalInput Fswitchlong;
    DigitalInput Bswitchlong;
    DigitalInput Fswitchshort;
    DigitalInput Bswitchshort;
    DutyCycleEncoder Armcoder;
    boolean LongArm;
    boolean switchup;
    double armRangeDeadzone = 0.02;
    double armMedVal = 0.2565;
    double armHighVal = 0.428;
    double armEncoderOffset;

    public Arm() {
        LongArmMotor= new VictorSPX(13);
        LongIntake=new VictorSPX(14);
        ShortArmMotor= new VictorSPX(15);
        ShortIntake=new VictorSPX(16);
        Fswitchlong= new DigitalInput(8);
        Bswitchlong= new DigitalInput(9);
        Fswitchshort= new DigitalInput(2);
        Bswitchshort= new DigitalInput(1);
        Armcoder = new DutyCycleEncoder(6);
        LongArm = false;
        switchup = true;
        armEncoderOffset = Armcoder.getDistance();
    }

    public double getArmEncoderReading(){
        return Armcoder.getDistance()-armEncoderOffset;
    }

    public void resetArmEncoderOffset(){
        armEncoderOffset = Armcoder.getDistance();
    }

    public boolean armMed(){
        double armSpeed = armMedVal-(Armcoder.getDistance()-armEncoderOffset);
        if(armSpeed > 0.10) armSpeed = 0.10; 
        if(armSpeed < -0.10) armSpeed = -0.10;
        if((Armcoder.getDistance()-armEncoderOffset)<armMedVal) {
            LongArmMotor.set(ControlMode.PercentOutput, -(armSpeed+0.05));
            return false;
        } else if ((Armcoder.getDistance()-armEncoderOffset)>armMedVal+armRangeDeadzone){
            LongArmMotor.set(ControlMode.PercentOutput, -(armSpeed-0.05));
            return false;
        }else{
            LongArmMotor.set(ControlMode.PercentOutput, 0); 
            return true;
        }
    }

    public boolean armHigh(){
        if(Bswitchlong.get()) {
            LongArmMotor.set(ControlMode.PercentOutput, -0.25);
            return false;
        }else{
            LongArmMotor.set(ControlMode.PercentOutput, -0.06); 
            return true;
        }
    }

    public void switchArm(){
        LongArm = !LongArm;
        switchup = true;
    }

    public void moveArm(double a){
        a = a * 0.25;
        if(LongArm){
            if(a > 0 && Fswitchlong.get()){
                LongArmMotor.set(ControlMode.PercentOutput, a);
            }else if(a < 0 && Bswitchlong.get() && !Fswitchshort.get()){
                LongArmMotor.set(ControlMode.PercentOutput, a);
            }else{
                if(Fswitchlong.get()){
                    LongArmMotor.set(ControlMode.PercentOutput, -0.06);
                }else{
                    LongArmMotor.set(ControlMode.PercentOutput, 0.04);
                }
                    
            }
            if(switchup){
                if (Fswitchshort.get()){
                    ShortArmMotor.set(ControlMode.PercentOutput, 0.25);
                }else{
                    ShortArmMotor.set(ControlMode.PercentOutput, 0.05);
                    switchup = false;
                }
                
            }
        }else{
            if(a > 0 && Fswitchshort.get()){
                ShortArmMotor.set(ControlMode.PercentOutput, a);
            }else if(a < 0 && Bswitchshort.get() && !Fswitchlong.get()){
                ShortArmMotor.set(ControlMode.PercentOutput, a);
            }else{
                ShortArmMotor.set(ControlMode.PercentOutput, 0.0);
            }
            if(switchup){
                if (Fswitchlong.get()){
                    LongArmMotor.set(ControlMode.PercentOutput, 0.25);
                }else{
                    LongArmMotor.set(ControlMode.PercentOutput, 0.06);
                    switchup = false;
                }
                
            }
        }
    }

    public void Intake(int intakeSwitch){
        if(LongArm){
            LongIntake.set(ControlMode.PercentOutput, -1*intakeSwitch);
        }else{
            ShortIntake.set(ControlMode.PercentOutput, -1*intakeSwitch);
        }
    }

    public void Outtake(int intakeSwitch){
        if(LongArm){
            LongIntake.set(ControlMode.PercentOutput, 1*intakeSwitch);
        }else{
            ShortIntake.set(ControlMode.PercentOutput, 1*intakeSwitch);
        }
    }

    public void Stoptake(){
        if(LongArm){
            LongIntake.set(ControlMode.PercentOutput, 0);
        }else{
            ShortIntake.set(ControlMode.PercentOutput, 0);
        }
    }

}