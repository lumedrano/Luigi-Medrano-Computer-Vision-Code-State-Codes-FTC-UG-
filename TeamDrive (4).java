package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp

public class TeamDrive extends OpMode{
    DcMotor lf;
    DcMotor lr;
    DcMotor rf;
    DcMotor rr;
    DcMotor intake;
    DcMotor slide;
    //DcMotor container;
    DcMotorEx launcher;
    DcMotorEx launcher2;
   // Servo aimer;
    Servo clamp;
    Servo armswivel;
    Servo containerarm;
    RevBlinkinLedDriver lights;
    int temp = 1; //temp is for temporary and 1 for time later on in the code.
    boolean lastPress = false;
    double motorVelocity = 0;
    
    boolean ServoLPress = false;
    double ServoPos = .9;

    boolean lastPosition = false;
    double servoPosition = .6;
    
     double currentVelocity;
    double maxVelocity = 0.0;
    
boolean lastPressBackup = false;
double motorVelocityBackup = 0; // for powershot power on launcher.


public void init(){
    lf = hardwareMap.dcMotor.get("lf");
    lr = hardwareMap.dcMotor.get("lr");
    rf = hardwareMap.dcMotor.get("rf");
    rr = hardwareMap.dcMotor.get("rr");
    intake = hardwareMap.dcMotor.get("intake");
    slide = hardwareMap.dcMotor.get("slide");
    /*slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    slide.setTargetPosition(0);*/
    //container = hardwareMap.dcMotor.get("container");
     //container.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    //container.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    //container.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    //container.setTargetPosition(0);
    //container.setDirection(DcMotorSimple.Direction.REVERSE);
   // aimer = hardwareMap.servo.get("aimer");
    launcher = hardwareMap.get(DcMotorEx.class, "launcher");
    launcher.setDirection(DcMotorSimple.Direction.REVERSE);
    launcher2 = hardwareMap.get(DcMotorEx.class, "launcher2");
    containerarm = hardwareMap.servo.get("containerarm");
    clamp = hardwareMap.servo.get("clamp");
    armswivel = hardwareMap.servo.get("armswivel");
    lf.setDirection(DcMotorSimple.Direction.REVERSE);
    lr.setDirection(DcMotorSimple.Direction.REVERSE);
    launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    launcher2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
   launcher.setVelocityPIDFCoefficients(1.22, .122, 0.0, 12.2);
   launcher.setPositionPIDFCoefficients(5.0);
   launcher2.setPositionPIDFCoefficients(5.0);
  
    launcher2.setVelocityPIDFCoefficients(1.22, .122, 0.0, 12.2);
    launcher.setPositionPIDFCoefficients(5.0);
    
    lights = hardwareMap.get(RevBlinkinLedDriver.class, "lights");
    lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
    
    //launcher.setPositionPIDFCoefficients(5.0);
    //clamp.setPosition(.5);//closes clamp for wobble goal
    //clamp.setPosition(.1);// opens clamp servo
   // armswivel.setPosition(.76);//puts arm in position to clamp servo.
    //containerarm.setPosition(1);
    //containerarm.setPosition(.65);

    
}

public void loop(){


double Speed = gamepad1.left_stick_y;
double Turn = gamepad1.right_stick_x;
double Strafe = -gamepad1.left_stick_x;

double lfspeed = Speed + Turn + Strafe;
double rfspeed = Speed - Turn - Strafe;
double lrspeed = Speed + Turn - Strafe;
double rrspeed = Speed - Turn + Strafe;
 
lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
lr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

if(gamepad1.right_trigger >= .5){
lf.setPower(Range.clip(lfspeed, -.4, .4));
rf.setPower(Range.clip(rfspeed, -.4, .4));
lr.setPower(Range.clip(lrspeed, -.4, .4));
rr.setPower(Range.clip(rrspeed, -.4, .4));
}
else{
lf.setPower(Range.clip(lfspeed, -1, 1));
rf.setPower(Range.clip(rfspeed, -1, 1));
lr.setPower(Range.clip(lrspeed, -1, 1));
rr.setPower(Range.clip(rrspeed, -1, 1));}

if(temp ==1){
    resetStartTime();
    temp = 2;
}
if(time >= 90 && time <= 110){
    lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED); //Endgame Timer
} 
else if(time >= 110 && time <= 120){
    lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.LIGHT_CHASE_BLUE);
    }
else if(time >= 120){lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
}

   

if(gamepad2.right_trigger >= .5){intake.setPower(1);}
else if(gamepad2.left_trigger >= .5){intake.setPower(-0.5);}
else{intake.setPower(0);}

if(gamepad2.dpad_up){
    slide.setPower(1);
    
}
else if(gamepad2.dpad_down){
    slide.setPower(-1);
}
else{slide.setPower(0);
    slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    
}


if(gamepad2.b){armswivel.setPosition(.72);}
else if(gamepad2.y){armswivel.setPosition(.46);}
else if(gamepad2.x){armswivel.setPosition(.18);}
else{}

//if(gamepad2.right_bumper){clamp.setPosition(.46);}
//else{clamp.setPosition(.75);}

boolean Thispress = gamepad2.right_bumper;
if(Thispress && !ServoLPress){
if(ServoPos == .9){
 ServoPos = .4;
}

else{
ServoPos = .9;
}
}
ServoLPress = Thispress;
clamp.setPosition(ServoPos);

if(gamepad2.left_bumper){containerarm.setPosition(.34);}
else{containerarm.setPosition(0.17);}



boolean thisPress = gamepad1.right_bumper;
if(thisPress && !lastPress){
if(motorVelocity == 0){
  // container.setTargetPosition(3200);
    //container.setPower(1);
    motorVelocity = 2125;
    lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
}
else if(motorVelocity == 2125){
    motorVelocity = 1990;
   lights.setPattern( RevBlinkinLedDriver.BlinkinPattern.RED);
   
}
else{//container.setTargetPosition(0);
//container.setPower(-.9);
    motorVelocity = 0;
   lights.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_WITH_GLITTER);
    
launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
launcher2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

}
}
lastPress = thisPress;
launcher.setVelocity(motorVelocity);
launcher2.setVelocity(motorVelocity);
            currentVelocity = launcher.getVelocity();
            currentVelocity = launcher2.getVelocity();
            if (currentVelocity > maxVelocity) {
                maxVelocity = currentVelocity;
            }
            
            telemetry.addData("current velocity", currentVelocity);
            telemetry.addData("maximum velocity", maxVelocity);
            telemetry.update();

}



}

