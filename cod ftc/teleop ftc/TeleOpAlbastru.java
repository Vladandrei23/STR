package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="TeleOp Nationala Albastru", group="Linear Opmode")
//@Disabled
public class TeleOpAlbastru extends LinearOpMode {

    int pozbrat = 0;
    int pozglisiera = 0;
    double precizie = 1;
    double depozit=1; //depozit = 0 ---> depozitare la shipping hub; depozit = 1 ---> depozitare la shared hub;
    double k=0;
    double oprit=0;

    public enum atransport {
        INTAKE,
        CUTIE,
        RESET
    }

    atransport transport = atransport.INTAKE;

    ElapsedTime Timer1 = new ElapsedTime();
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("leftFront");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("leftRear");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("rightFront");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("rightRear");
        DcMotor brat = hardwareMap.dcMotor.get("brat");
        DcMotor carusel = hardwareMap.dcMotor.get("carusel");
        DcMotor glisiera = hardwareMap.dcMotor.get("glisiera");
        DcMotor intake = hardwareMap.dcMotor.get("intake");
        Servo kicker = hardwareMap.servo.get("kicker");
        ///Servo creanga = hardwareMap.servo.get("creanga");
        DistanceSensor senzordistanta = hardwareMap.get(DistanceSensor.class, "SenzorCuloare");

        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        carusel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotor.Direction.FORWARD);
        motorFrontLeft.setDirection(DcMotor.Direction.FORWARD);

        intake.setDirection(DcMotor.Direction.REVERSE);
        glisiera.setDirection(DcMotor.Direction.FORWARD);
        glisiera.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        brat.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;
            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator / precizie;
            double backLeftPower = (y - x + rx) / denominator / precizie;
            double frontRightPower = (y - x - rx) / denominator / precizie;
            double backRightPower = (y + x - rx) / denominator / precizie;

            if (gamepad1.left_bumper) {precizie = 1.60;}
            if (gamepad1.right_bumper) {precizie = 1;}

            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);

            brat.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            if (brat.getCurrentPosition()==pozbrat) {brat.setPower(0);}
            if (brat.getCurrentPosition()!=pozbrat) {
                brat.setTargetPosition(pozbrat);
                brat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                brat.setPower(0.8);
            }

            glisiera.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            if (glisiera.getCurrentPosition()==pozglisiera) {glisiera.setPower(0);}
            if (glisiera.getCurrentPosition()!=pozglisiera) {
                glisiera.setTargetPosition(pozglisiera);
                glisiera.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                glisiera.setPower(0.6);
            }

            switch(transport) {
                case INTAKE:
                    if (brat.getCurrentPosition() >= -5 && brat.getCurrentPosition() <= 5) {
                        if (oprit==0) {intake.setPower(0.6);}
                        if (oprit==1) {intake.setPower(0);}
                        kicker.setPosition(0.8);
                    }
                    if (senzordistanta.getDistance(DistanceUnit.CM) > 5) {Timer1.reset();}
                    if (senzordistanta.getDistance(DistanceUnit.CM) <= 5 && oprit==0) {
                        if (Timer1.seconds()>0.3) {
                            kicker.setPosition(0.67);
                            intake.setPower(0);
                            transport = atransport.CUTIE;
                        }}
                    break;
                case CUTIE:
                    if (depozit == 0) {
                        k=0;
                        if (glisiera.getCurrentPosition() <= 25) {pozglisiera=292;}
                        if (glisiera.getCurrentPosition() >= 275 && brat.getCurrentPosition() <= 50) {
                            pozbrat=134;
                        }
                        if (glisiera.getCurrentPosition() >= 275 && brat.getCurrentPosition() >= 130 && glisiera.getCurrentPosition() <= 375) {
                            pozglisiera=728;
                        }
                        if (brat.getCurrentPosition() >= 130 && glisiera.getCurrentPosition() >= 718 && brat.getCurrentPosition() <= 500) {
                            pozbrat=1540;
                        }
                        if (brat.getCurrentPosition() >= 1400) {pozglisiera=2025;}
                        if (glisiera.getCurrentPosition() >= 1925 && gamepad1.b) {
                            kicker.setPosition(1);
                            transport = atransport.RESET;
                        }
                    }
                    if (depozit == 1) {
                        k=1;
                        if (glisiera.getCurrentPosition() <= 25) {pozglisiera=292;}
                        if (glisiera.getCurrentPosition() >= 275 && brat.getCurrentPosition() <= 50) {
                            pozbrat=134;
                        }
                        if (glisiera.getCurrentPosition() >= 275 && brat.getCurrentPosition() >= 130 && glisiera.getCurrentPosition() <= 375) {
                            pozglisiera=728;
                        }
                        if (brat.getCurrentPosition() >= 130 && glisiera.getCurrentPosition() >= 718 && brat.getCurrentPosition() <= 500) {
                            pozbrat=1835;
                        }
                        if (brat.getCurrentPosition() >= 1800 && glisiera.getCurrentPosition() >= 718 && gamepad1.b) {
                            kicker.setPosition(1);
                            transport = atransport.RESET;
                        }
                    }
                    break;
                case RESET:
                    if (k==0) {
                        if (gamepad1.y) {pozglisiera=728;}
                        if (glisiera.getCurrentPosition() <= 738) {pozbrat=134;}
                        if (brat.getCurrentPosition() <= 138 && brat.getCurrentPosition() >= 120) {pozglisiera=292;}
                        if (glisiera.getCurrentPosition() <= 300) {pozbrat=0;}
                        if (brat.getCurrentPosition() <= 10) {pozglisiera=0;}
                        if (glisiera.getCurrentPosition() <= 5 && brat.getCurrentPosition() <= 5) {transport = atransport.INTAKE;}
                    }
                    if (k==1) {
                        if (gamepad1.y) {
                            pozbrat=134;
                        }
                            if (brat.getCurrentPosition() <= 138 && brat.getCurrentPosition() >= 120) {pozglisiera=292;}
                            if (glisiera.getCurrentPosition() <= 300) {pozbrat=0;}
                            if (brat.getCurrentPosition() <= 10) {pozglisiera=0;}
                            if (glisiera.getCurrentPosition() <= 5 && brat.getCurrentPosition() <= 5) {transport = atransport.INTAKE;}
                    }
                    break;
            }

            if (gamepad2.left_bumper) {carusel.setPower(0.6);}
            if (gamepad2.right_bumper) {carusel.setPower(0);}
            if (gamepad2.dpad_up && transport!=atransport.CUTIE) {depozit=0;}
            if (gamepad2.dpad_down && transport!=atransport.CUTIE) {depozit=1;}
            /*
            if (gamepad1.dpad_up) {creanga.setPosition(0.5);}
            if (gamepad1.dpad_down) {creanga.setPosition(0.8);}
            if (gamepad1.dpad_right) {creanga.setPosition(0.6);}
            if (gamepad1.dpad_left) {creanga.setPosition(0.4);}
            if (gamepad2.x) {creanga.setPosition(0);}
             */
            if (gamepad2.b) {oprit=1;}
            if (gamepad2.a) {oprit=0;}

            telemetry.addData("Motors", "backleft, backright", backLeftPower, backRightPower);
            telemetry.addData("Motors", "frontleft, frontright ", frontLeftPower, frontRightPower);
            telemetry.addData("upleft position", motorFrontLeft.getCurrentPosition());
            telemetry.addData("backleft position", motorBackLeft.getCurrentPosition());
            telemetry.addData("upright position", motorFrontRight.getCurrentPosition());
            telemetry.addData("backright position", motorBackRight.getCurrentPosition());
            telemetry.addData("glisiera position", glisiera.getCurrentPosition());
            telemetry.addData("brat position", brat.getCurrentPosition());
            telemetry.update();
        }

    }
}
