package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;

public class Arm {
    VictorSPX LongArmMotor;
    VictorSPX LongIntake;
    VictorSPX ShortArmMotor;
    VictorSPX ShortIntake;
    DigitalInput Fswitchlong;
    DigitalInput Bswitchlong;
    DigitalInput Fswitchshort;
    DigitalInput Bswitchshort;
    boolean LongArm;
    boolean switchup;

    public Arm() {
        LongArmMotor= new VictorSPX(13);
        LongIntake=new VictorSPX(14);
        ShortArmMotor= new VictorSPX(15);
        ShortIntake=new VictorSPX(16);
        Fswitchlong= new DigitalInput(8);
        Bswitchlong= new DigitalInput(9);
        Fswitchshort= new DigitalInput(2);
        Bswitchshort= new DigitalInput(1);
        LongArm = true;
        switchup = true;
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
                    ShortArmMotor.set(ControlMode.PercentOutput, 0.04);
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