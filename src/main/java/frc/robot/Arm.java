package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;

public class Arm {
    VictorSPX LongArmMotor;
    VictorSPX LongIntake;
    VictorSPX ShortArmMotor;
    VictorSPX ShortIntake;
    DigitalInput Fswitchlong;
    DigitalInput Bswitchlong;
    DigitalInput Fswitchshort;
    DigitalInput Bswitchshort;
    //Encoder Armcoder;
    boolean LongArm;

    public Arm() {
        LongArmMotor= new VictorSPX(13);
        LongIntake=new VictorSPX(14);
        ShortArmMotor= new VictorSPX(15);
        ShortIntake=new VictorSPX(16);
        Fswitchlong= new DigitalInput(8);
        Bswitchlong= new DigitalInput(9);
        Fswitchshort= new DigitalInput(2);
        Bswitchshort= new DigitalInput(1);
        //Armcoder= new Encoder(1, 0);//A?????????
        LongArm = true;
    }

    public void switchArm(){
        LongArm = !LongArm;
    }

    public void moveArm(double a){
        a = a * 0.25;
        if(LongArm){
            if(a > 0 && Fswitchlong.get()){
                LongArmMotor.set(ControlMode.PercentOutput, a);
            }else if(a < 0 && Bswitchlong.get()){
                LongArmMotor.set(ControlMode.PercentOutput, a);
            }else{
                if(Fswitchlong.get()){
                    LongArmMotor.set(ControlMode.PercentOutput, 0);
                }else{
                    LongArmMotor.set(ControlMode.PercentOutput, 0);
                }
                    
            }
        }else{
            if(a > 0 && Fswitchshort.get()){
                ShortArmMotor.set(ControlMode.PercentOutput, a);
            }else if(a < 0 && Bswitchshort.get()){
                ShortArmMotor.set(ControlMode.PercentOutput, a);
            }else{
                ShortArmMotor.set(ControlMode.PercentOutput, 0);
            }
        }
    }

    public void Intake(){
        if(LongArm){
            LongIntake.set(ControlMode.PercentOutput, -1);
        }else{
            ShortIntake.set(ControlMode.PercentOutput, -1);
        }
    }

    public void Outtake(){
        if(LongArm){
            LongIntake.set(ControlMode.PercentOutput, 1);
        }else{
            ShortIntake.set(ControlMode.PercentOutput,    1);
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