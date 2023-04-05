package frc.robot;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class SwervCorner {


    private final CANSparkMax driveMotor;
    private final CANSparkMax turnMotor;
    private final CANCoder encoder;
    private final RelativeEncoder driveEncoder;

    
    public boolean isForward;
    public int zvecX;
    public int zvecY;
    private double rotateOffset;
    private double isNatForward;

    public double driveMagnitude;
    public double driveAngle;

    private double maxDriveSpeed = 1;
    private double zeroNum = 0;

    public SwervCorner(int driveID, int turnID, int encoderID, int zvecX, int zvecY, double rotateOffset, double isNatForward){
        driveMotor = new CANSparkMax(driveID, MotorType.kBrushless);
        turnMotor = new CANSparkMax(turnID, MotorType.kBrushless);

        driveEncoder = driveMotor.getEncoder();
        encoder = new CANCoder(encoderID);
        
        isForward = true;
        this.zvecX = zvecX;
        this.zvecY = zvecY;
        this.rotateOffset = rotateOffset; 
        this.isNatForward = isNatForward;
    }

    public void updateCorner(double x, double y, double z, double navXYawAngle){
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
        
        double totalX = correctedX + (zvecX * z);
        double totalY = correctedY + (zvecY * z);
        double hypotenuse = Math.sqrt((Math.pow(totalX, 2)+(Math.pow(totalY, 2))));
        double angle = Math.toDegrees(Math.acos(totalX / hypotenuse));
        if (totalY < 0) angle = 360 - angle;
        driveAngle = angle;
        driveMagnitude = hypotenuse;
    }

    public double getDriveEncoderPosition(){
        return ((driveEncoder.getPosition()- zeroNum)/-6.6);
    }

    public double getEncoderPosition(){
        return encoder.getAbsolutePosition();
    }

    public void resetEncoder(){
        zeroNum = driveEncoder.getPosition();
    }

    public void goTo(double deg){
        deg = (deg + rotateOffset)%360;
        double curAngle = encoder.getAbsolutePosition();
        

        double cwiseF = (360 + deg - curAngle) % 360;
        double countercwiseF = (360 - deg + curAngle) % 360;
        double cwiseB = (360 - curAngle + ((deg + 180)%360)) % 360;
        double countercwiseB = (360 - ((deg + 180)%360) + curAngle) % 360;
        
        double rotateSpeed = 0; 

        if(cwiseF < 90){
            rotateSpeed = -(Math.abs(cwiseF)/90);
            isForward = true;
        }
        else if(countercwiseF < 90){
            rotateSpeed = (Math.abs(countercwiseF)/90);
            isForward = true;
        }
        else if(cwiseB < 90){
            rotateSpeed = -(Math.abs(cwiseB)/90);
            isForward = false;
        }else{
            rotateSpeed = (Math.abs(countercwiseB)/90);
            isForward = false;
        }
        turnMotor.set(rotateSpeed);
    }

    public void stopRotation(){
        turnMotor.set(0);
    }

    public void driveSpeed(double speed){
        if (isForward){
            driveMotor.set(speed*maxDriveSpeed*isNatForward);
        }else{
            driveMotor.set(speed*-1*maxDriveSpeed*isNatForward);
        }
    }

    public void setEncoder(){
        encoder.setPositionToAbsolute();
    }
}
