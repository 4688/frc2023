package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import javax.naming.LimitExceededException;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;

public class Arm {
    TalonSRX Arm_motor1;
    TalonSRX Intake_motor1;
    TalonSRX Arm_motor2;
    TalonSRX Intake_motor2;
    DigitalInput Fswitch1;
    DigitalInput Bswitch1;
    DigitalInput Fswitch2;
    DigitalInput Bswitch2;
    boolean LongArm = true;

    public Arm() {
        Arm_motor1= new TalonSRX(0);
        Intake_motor1=new TalonSRX(1);
        Arm_motor2= new TalonSRX(2);
        Intake_motor2=new TalonSRX(3);
        Fswitch1= new DigitalInput(0);
        Bswitch1= new DigitalInput(1);
        Fswitch2= new DigitalInput(2);
        Bswitch2= new DigitalInput(3);
    }

    public void switchArm(){
        LongArm = !LongArm;
    }

    public void moveArm(double a){
        if()

    }




    
    public vid teleopPeriodic() {
        if (leftstick.toggleWhenPressed(true)){
            LongArm = false;
        }
        
        if (LongArm==true) {
          // this sets F to the first limitswitch as a boolean value
          boolean F1 = Fswitch1.get();
          // this sets bl to the second limitswitch as a boolean value
          boolean B1 = Bswitch1.get();
          // sets x to the axis of the joystick
          double y = leftstick.getRawAxis(5);
          y=y*0.1;
          //this checks if both of the limoswitches are false and if y is = to 0
          if((F1==true && B1==true) && y<=0.03){
            // sets the arm motor to x
            Arm_motor1.set(ControlMode.PercentOutput, y);
          }
          //checks if the limitswitch is on and then moves in the opposite direction
          else if(F1==false && y>0){
            Arm_motor1.set(ControlMode.PercentOutput,y);
        
          }
          //checks if the limitswitch is on and then moves in the opposite direction
          else if (B1==false && y<0){
            Arm_motor1.set(ControlMode.PercentOutput,y);
          }
          //checks to see if the button's been pressed and then sets the intake motor to 1
          if (leftstick.getRawButton(0)==true){
            Intake_motor1.set(ControlMode.PercentOutput,1);    
          }
          //chezks to see if the butoon is pressed and then inverts the intake motor
          else if(leftstick.getRawButton(1)==true){
            Intake_motor1.set(ControlMode.PercentOutput,-1);
          }
        }
      
        if (LongArm==false){
          // this sets l to the first limitswitch as a boolean value
          boolean F2 = Fswitch2.get();
          // this sets bl to the second limitswitch as a boolean value
          boolean B2 = Bswitch2.get();
          // sets x to the axis of the joystick
          double y = leftstick.getRawAxis(5);
          y=y*0.1;
          //this checks if both of the limoswitches are false and if y is = to 0
          if((F2==true && B2==true) || y==0){
            // sets the arm motor to x
            Arm_motor2.set(ControlMode.PercentOutput, y);
          }
          //checks if the limitswitch is on and then moves in the opposite direction
          else if(F2==false && y>0){
            Arm_motor2.set(ControlMode.PercentOutput,y);
        
          }
          //checks if the limitswitch is on and then moves in the opposite direction
          else if (B2==false && y<0){
            Arm_motor2.set(ControlMode.PercentOutput,y);
          }
          //checks to see if the button's been pressed and then sets the intake motor to 1
          if (leftstick.getRawButton(0)==true){
            Intake_motor2.set(ControlMode.PercentOutput,1);    
          }
          //chezks to see if the butoon is pressed and then inverts the intake motor
          else if(leftstick.getRawButton(1)==true){
            Intake_motor2.set(ControlMode.PercentOutput,-1);
            }
        }
    }
    
}
