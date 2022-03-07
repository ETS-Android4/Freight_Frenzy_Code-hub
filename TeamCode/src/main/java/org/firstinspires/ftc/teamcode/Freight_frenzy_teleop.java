package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.commands.core.LynxSetMotorPIDFControlLoopCoefficientsCommand;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;

@TeleOp

public class Freight_frenzy_teleop extends LinearOpMode {
    //
    DcMotor stangafata;
    DcMotor dreaptafata;
    DcMotor stangaspate;
    DcMotor dreaptaspate;

    //servos
    Servo carusel;
    Servo control;
    Servo retragere;

    DcMotor intake1;
    DcMotor intake2;
    DcMotor extindere;
    DcMotor glisiera;

    DistanceSensor distanta;

    @Override
    public void runOpMode() {


        // sensor
        distanta = hardwareMap.get(DistanceSensor.class, "distanta");
        //servos
        retragere = hardwareMap.servo.get("retragere");
        carusel = hardwareMap.servo.get("carusel");
        control = hardwareMap.servo.get("control");

        //dc motors
        stangafata = hardwareMap.dcMotor.get("stangafata");
        dreaptafata = hardwareMap.dcMotor.get("dreaptafata");
        stangaspate = hardwareMap.dcMotor.get("stangaspate");
        dreaptaspate = hardwareMap.dcMotor.get("dreaptaspate");

        intake1 = hardwareMap.dcMotor.get("intake1");
        intake2 = hardwareMap.dcMotor.get("intake2");
        extindere = hardwareMap.dcMotor.get("extindere");
        glisiera = hardwareMap.dcMotor.get("glisiera");


        dreaptafata.setDirection(DcMotorSimple.Direction.FORWARD);
        dreaptaspate.setDirection(DcMotorSimple.Direction.FORWARD);
        stangafata.setDirection(DcMotorSimple.Direction.REVERSE);
        stangaspate.setDirection(DcMotorSimple.Direction.REVERSE);

        stangafata.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dreaptafata.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        stangaspate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dreaptaspate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        extindere.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        glisiera.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        extindere.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        glisiera.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.update();
        waitForStart();
        while (opModeIsActive()){
            //control miscare
            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;

            //slowmode - default-half speed  pressed-full speed

            if(!gamepad1.left_bumper){
                x/=2;
                y/=2;
            }
            if(!gamepad1.right_bumper) {
                turn /= 2;
            }
            mecanum(x, y, turn);


            //gamepad 2 - control brat si intake + carusel
            if (gamepad2.left_bumper){
                carusel.setPosition(1);
                sleep(1200);
                carusel.setPosition(0.5);
            }

            if(gamepad2.right_trigger>0)
            {
                intake1.setPower(gamepad2.right_trigger);
                intake2.setPower(-gamepad2.right_trigger);

            }else if(gamepad2.left_trigger>0)
            {
                intake1.setPower(-gamepad2.left_trigger);
                intake2.setPower(gamepad2.left_trigger);

            }else{

                intake1.setPower(0);
                intake2.setPower(0);
            }


            if (gamepad2.dpad_up) {
                control.setPosition(-0.2);
                retragere.setPosition(-0.4);
            }
            else if(gamepad2.dpad_down) {
                retragere.setPosition(0.6);
                control.setPosition(0.85);
            }


            /*
            * Extindere + retragere glisiera
            */

            if(gamepad2.dpad_up) {
                glisiera.setTargetPosition(1850);
                glisiera.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                glisiera.setPower(1);

                extindere.setTargetPosition(1200);
                extindere.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                extindere.setPower(0.8);
                telemetry.addData("ajuns la nivel 3", "");
            }
            else if(gamepad2.dpad_down){
                extindere.setTargetPosition(10);
                extindere.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                extindere.setPower(0.8);

                glisiera.setTargetPosition(10);
                glisiera.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                glisiera.setPower(1);
            }

        }
    }

    /**
     * Control a mecanum drive base with three double inputs
     *
     * @param Strafe  is the first double X value which represents how the base should strafe
     * @param Forward is the only double Y value which represents how the base should drive forward
     * @param Turn    is the second double X value which represents how the base should turn
     */
    public void mecanum(double Strafe, double Forward, double Turn) {
        //Find the magnitude of the controller's input
        double r = Math.hypot(Strafe, Forward);

        //returns point from +X axis to point (forward, strafe)
        double robotAngle = Math.atan2(Forward, Strafe) - Math.PI / 4;

        //Quantity to turn by (turn)
        double rightX = Turn;

        //double vX represents the velocities sent to each motor
        final double v1 = (r * Math.cos(robotAngle)) + rightX;
        final double v2 = (r * Math.sin(robotAngle)) - rightX;
        final double v3 = (r * Math.sin(robotAngle)) + rightX;
        final double v4 = (r * Math.cos(robotAngle)) - rightX;

        stangafata.setPower(v1);
        dreaptafata.setPower(v2);
        stangaspate.setPower(v3);
        dreaptaspate.setPower(v4);

    }
}
