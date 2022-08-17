package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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
    DcMotor container;
    DcMotorEx launcher;
    Servo aimer;
    Servo clamp;
    Servo armswivel;
    Servo containerarm;
    
    boolean lastPress = false;
    double motorVelocity = 0;
    
    boolean lastPosition = false;
    double servoPosition = .12;

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
    container = hardwareMap.dcMotor.get("container");
     container.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    container.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    container.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    container.setTargetPosition(0);
    container.setDirection(DcMotorSimple.Direction.REVERSE);
    aimer = hardwareMap.servo.get("aimer");
    launcher = hardwareMap.get(DcMotorEx.class, "launcher");
    containerarm = hardwareMap.servo.get("containerarm");
    clamp = hardwareMap.servo.get("clamp");
    armswivel = hardwareMap.servo.get("armswivel");
    lf.setDirection(DcMotorSimple.Direction.REVERSE);
    lr.setDirection(DcMotorSimple.Direction.REVERSE);
    launcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    
    //clamp.setPosition(.5);//closes clamp for wobble goal
    //clamp.setPosition(.1);// opens clamp servo
   // armswivel.setPosition(.76);//puts arm in position to clamp servo.
    //containerarm.setPosition(1);
    //containerarm.setPosition(.65);

    
}

public void loop(){


double Speed = -gamepad1.left_stick_y;
double Turn = gamepad1.right_stick_x;
double Strafe = gamepad1.left_stick_x;

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


if(gamepad2.b){armswivel.setPosition(.67);}
else if(gamepad2.y){armswivel.setPosition(.46);}
else if(gamepad2.x){armswivel.setPosition(.6);}
else{}

if(gamepad2.right_bumper){clamp.setPosition(.5);}
else{clamp.setPosition(.75);}


if(gamepad2.left_bumper){containerarm.setPosition(.35);}
else{containerarm.setPosition(.65);}


boolean thisPress = gamepad1.right_bumper;
if(thisPress && !lastPress){
if(motorVelocity == 0){
    container.setTargetPosition(2900);
    container.setPower(1);
    motorVelocity = 3000;
}
else{container.setTargetPosition(0);
container.setPower(-.9);
    motorVelocity = 0;
launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

}
}
lastPress = thisPress;
launcher.setVelocity(motorVelocity);

boolean thisPosition = gamepad2.a;
if(thisPosition && !lastPosition){
if(servoPosition == .12){servoPosition = .18;}
else{servoPosition = .12;}
}
lastPosition = thisPosition;
aimer.setPosition(servoPosition);


/*if(gamepad2.dpad_left){aimer.setPosition(.18);}
else{
aimer.setPosition(.12);
}*/
}

}


