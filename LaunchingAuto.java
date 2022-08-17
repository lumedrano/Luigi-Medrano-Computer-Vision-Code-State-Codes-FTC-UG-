package AutonomousCodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous

public class LaunchingAuto extends LinearOpMode{
DcMotor container;
DcMotorEx launcher;
Servo containerarm;
Servo aimer;

public void runOpMode(){
container = hardwareMap.dcMotor.get("container");
     container.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    container.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    container.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    container.setTargetPosition(0);
    container.setDirection(DcMotorSimple.Direction.REVERSE);
    launcher = hardwareMap.get(DcMotorEx.class, "launcher"); 
 launcher.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
 aimer = hardwareMap.servo.get("aimer");
 containerarm = hardwareMap.servo.get("containerarm");
 containerarm.setPosition(.65);
 
 
 
 
 waitForStart();
 
 
 
 
 aimer.setPosition(.14);
 container.setTargetPosition(2900);
 container.setPower(1);
 sleep(2000);
 while(opModeIsActive() && container.isBusy()){
launcher.setVelocity(3000);
sleep(1000);
containerarm.setPosition(.35);
sleep(600);
containerarm.setPosition(.65);
sleep(600);
containerarm.setPosition(.35);
sleep(600);
containerarm.setPosition(.65);
sleep(600);
containerarm.setPosition(.35);
sleep(600);
containerarm.setPosition(.65);
sleep(800);
launcher.setVelocity(0);
container.setTargetPosition(0);
container.setPower(-.9);
sleep(40000);
 }
}
}
