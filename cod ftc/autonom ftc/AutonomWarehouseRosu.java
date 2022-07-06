package org.firstinspires.ftc.teamcode.drive.advanced;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Autonomous(name="Autonom Warehouse Rosu", group = "advanced")
public class AutonomWarehouseRosu extends LinearOpMode {

    private OpenCvCamera webcam;
    private ContourPipeline pipeline;

    private int minRectangleArea = 0;

    public static Scalar scalarLowerYCrCb = new Scalar(0.0, 0.0, 0.0);
    public static Scalar scalarUpperYCrCb = new Scalar(255.0, 120.0, 120.0);

    enum State {
        PRELOADED,
        PRELOADED2,
        TRAJECTORY_1,
        TRAJECTORY_2,
        TRAJECTORY_3,
        IDLE
    }

    double once = 0;
    double preload = 0; //preload din detectie
    double gata = 0;
    double hold = 0.67;
    double out = 1;
    double in = 0.8;
    int pozbrat = 0;
    int pozglisiera = 0;

    State currentState = State.PRELOADED;

    Pose2d startPose = new Pose2d(10, -66, Math.toRadians(90));

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveCancelable drive = new SampleMecanumDriveCancelable(hardwareMap);
        DcMotor brat = hardwareMap.dcMotor.get("brat");
        DcMotor carusel = hardwareMap.dcMotor.get("carusel");
        DcMotor glisiera = hardwareMap.dcMotor.get("glisiera");
        DcMotor intake = hardwareMap.dcMotor.get("intake");
        Servo kicker = hardwareMap.servo.get("kicker");
        DistanceSensor senzordistanta = hardwareMap.get(DistanceSensor.class, "SenzorCuloare");

        drive.setPoseEstimate(startPose);

        Trajectory trajectory1caz1 = drive.trajectoryBuilder(new Pose2d(10, -66, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(0, -49, Math.toRadians(110)))
                .build();

        Trajectory trajectory2caz1 = drive.trajectoryBuilder(trajectory1caz1.end())
                .lineToLinearHeading(new Pose2d(10, -66, Math.toRadians(180)))
                .build();

        Trajectory trajectory1caz2 = drive.trajectoryBuilder(new Pose2d(10, -66, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(0, -54, Math.toRadians(110)))
                .build();

        Trajectory fataputint1c2 = drive.trajectoryBuilder(trajectory1caz2.end())
                .lineToLinearHeading(new Pose2d(0, -48, Math.toRadians(110)))
                .build();

        Trajectory trajectory2caz2 = drive.trajectoryBuilder(fataputint1c2.end())
                .lineToLinearHeading(new Pose2d(10, -66, Math.toRadians(180)))
                .build();

        Trajectory trajectory1caz3 = drive.trajectoryBuilder(new Pose2d(10, -66, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(0, -56, Math.toRadians(110)))
                .build();

        Trajectory fataputint1c3 = drive.trajectoryBuilder(trajectory1caz3.end())
                .lineToLinearHeading(new Pose2d(0, -47, Math.toRadians(110)))
                .build();

        Trajectory trajectory2caz3 = drive.trajectoryBuilder(fataputint1c3.end())
                .lineToLinearHeading(new Pose2d(10, -66, Math.toRadians(180)))
                .build();

        Trajectory TrajectoryWarehouse = drive.trajectoryBuilder(trajectory2caz1.end())
                .lineToLinearHeading(new Pose2d(42, -66, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(55, -66, Math.toRadians(180)), SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        ElapsedTime Timer1 = new ElapsedTime();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        pipeline = new ContourPipeline(0.2, 0.2, 0.2, 0.2);

        pipeline.configureScalarLower(scalarLowerYCrCb.val[0], scalarLowerYCrCb.val[1], scalarLowerYCrCb.val[2]);
        pipeline.configureScalarUpper(scalarUpperYCrCb.val[0], scalarUpperYCrCb.val[1], scalarUpperYCrCb.val[2]);

        webcam.setPipeline(pipeline);

        // Webcam Streaming
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        FtcDashboard.getInstance().startCameraStream(webcam, 10);

        waitForStart();

        Thread closeCameraThread = new Thread(() -> webcam.closeCameraDevice());
        closeCameraThread.start();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {

            double rectangleArea = pipeline.getRectArea();

            if (rectangleArea >= minRectangleArea) {
                if (pipeline.getRectMidpointX() < 240) {
                    telemetry.addData("TSE", "Low");
                    preload = 3;
                } else if (pipeline.getRectMidpointX( ) >= 240 && pipeline.getRectMidpointX() < 455) {
                    telemetry.addData("TSE", "Mid");
                    preload = 2;
                } else if (pipeline.getRectMidpointX() >= 455) {
                    telemetry.addData("TSE", "High");
                    preload = 1;
                }
                telemetry.update();
            }

            carusel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            brat.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            if (brat.getCurrentPosition() == pozbrat) {
                brat.setPower(0);
            }
            if (brat.getCurrentPosition() != pozbrat) {
                brat.setTargetPosition(pozbrat);
                brat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                brat.setPower(0.7);
            }

            glisiera.setDirection(DcMotorSimple.Direction.FORWARD);
            glisiera.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            if (glisiera.getCurrentPosition() == pozglisiera) {
                glisiera.setPower(0);
            }
            if (glisiera.getCurrentPosition() != pozglisiera) {
                glisiera.setTargetPosition(pozglisiera);
                glisiera.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                glisiera.setPower(0.4);
            }

            switch (currentState) {
                case PRELOADED:
                    kicker.setPosition(hold);
                    if (preload == 1) {
                        if (once == 0) {
                            Timer1.reset();
                            drive.followTrajectoryAsync(trajectory1caz1);
                            once = 1;
                        }
                        ridica();
                    }
                    if (preload == 2) {
                        if (once == 0) {
                            Timer1.reset();
                            drive.followTrajectoryAsync(trajectory1caz2);
                            once = 1;
                        }
                        ridicacaz2();
                    }
                    if (preload == 3) {
                        if (once == 0) {
                            Timer1.reset();
                            drive.followTrajectoryAsync(trajectory1caz3);
                            once = 1;
                        }
                        ridicacaz3();
                    }
                    if (!drive.isBusy() && gata == 1 && preload==2) {
                        drive.followTrajectory(fataputint1c2);
                        kicker.setPosition(out);
                        currentState = State.PRELOADED2;
                        once = 0;
                    }

                    if (!drive.isBusy() && gata == 1 && preload==3) {
                        drive.followTrajectory(fataputint1c3);
                        kicker.setPosition(out);
                        currentState = State.PRELOADED2;
                        once = 0;
                    }

                    if (!drive.isBusy() && gata == 1 && preload==1) {
                        kicker.setPosition(out);
                        currentState = State.PRELOADED2;
                        once = 0;
                    }
                    break;

                case PRELOADED2:
                    if (preload == 1) {
                        if (once == 0) {
                            sleep(600);
                            drive.followTrajectoryAsync(trajectory2caz1);
                            once = 1;
                        }
                        coboara();
                    }
                    if (preload==2) {
                        if (once == 0) {
                            sleep(600);
                            drive.followTrajectoryAsync(trajectory2caz2);
                            once=1;
                        }
                        coboaracaz2();}

                    if (preload==3) {
                        if (once == 0) {
                            sleep(600);
                            drive.followTrajectoryAsync(trajectory2caz3);
                            once=1;
                        }
                        coboaracaz3();}
                    if (!drive.isBusy() && gata == 0) {
                        once = 0;
                        currentState = State.TRAJECTORY_1;
                    }
                    break;

                case TRAJECTORY_1:
                    if (senzordistanta.getDistance(DistanceUnit.CM) > 5) {
                        kicker.setPosition(in);
                        if (once == 0) {
                            drive.followTrajectoryAsync(TrajectoryWarehouse);
                            once = 1;
                        }
                        intake.setPower(-0.6);
                    }
                    if (senzordistanta.getDistance(DistanceUnit.CM) <= 5) {
                        sleep(100);
                        kicker.setPosition(hold);
                        intake.setPower(0);
                        drive.breakFollowing();
                        drive.setDrivePower(new Pose2d());
                        if (Timer1.seconds()<19) {
                            once = 0;
                            currentState = State.TRAJECTORY_2;
                        }
                        if (Timer1.seconds()>=19) {
                            currentState = State.IDLE;
                        }
                    }

                    break;
                case TRAJECTORY_2:
                    if (once == 0) {
                        TrajectorySequence TrajectorySeqSpreHub = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(10, -66, Math.toRadians(180)))
                                .lineToLinearHeading(new Pose2d(0, -47, Math.toRadians(120)))
                                .build();
                        drive.followTrajectorySequenceAsync(TrajectorySeqSpreHub);
                        once = 1;
                    }
                    ridica();
                    if (!drive.isBusy() && gata == 1) {
                        kicker.setPosition(out);
                        sleep(200);
                        once = 0;
                        currentState = State.TRAJECTORY_3;
                    }
                    break;
                case TRAJECTORY_3:
                    if (once == 0) {
                        drive.followTrajectoryAsync(trajectory2caz1);
                        once = 1;
                    }
                    coboara();
                    if (gata == 0) {
                        once = 0;
                        currentState = State.TRAJECTORY_1;
                    }
                    break;
                case IDLE:

                    break;
            }

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();

            PoseStorage.currentPose = poseEstimate;

            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }

    private void ridica() {
        DcMotor brat = hardwareMap.dcMotor.get("brat");
        DcMotor glisiera = hardwareMap.dcMotor.get("glisiera");
        if (glisiera.getCurrentPosition() <= 25) {
            pozglisiera = 294;
        }
        if (glisiera.getCurrentPosition() >= 290 && brat.getCurrentPosition() <= 50) {
            pozbrat = 134;
        }
        if (glisiera.getCurrentPosition() >= 290 && brat.getCurrentPosition() >= 130 && glisiera.getCurrentPosition() <= 375) {
            pozglisiera = 728;
        }
        if (brat.getCurrentPosition() >= 130 && glisiera.getCurrentPosition() >= 725 && brat.getCurrentPosition() <= 500) {
            pozbrat = 1540;
        }
        if (brat.getCurrentPosition() >= 1400) {
            pozglisiera = 1825;
        }
        if (glisiera.getCurrentPosition() >= 1820) {
            gata = 1;
        }
    }

    private void coboara() {
        DcMotor brat = hardwareMap.dcMotor.get("brat");
        DcMotor glisiera = hardwareMap.dcMotor.get("glisiera");
        if (glisiera.getCurrentPosition() >= 1500) {
            pozglisiera = 728;
        }
        if (glisiera.getCurrentPosition() <= 735) {
            pozbrat = 134;
        }
        if (brat.getCurrentPosition() <= 138 && brat.getCurrentPosition() >= 120) {
            pozglisiera = 294;
        }
        if (glisiera.getCurrentPosition() <= 298) {
            pozbrat = 0;
        }
        if (brat.getCurrentPosition() <= 10) {
            pozglisiera = 0;
        }
        if (glisiera.getCurrentPosition() < 5) {
            gata = 0;
        }

    }

    private void ridicacaz2() {
        DcMotor brat = hardwareMap.dcMotor.get("brat");
        DcMotor glisiera = hardwareMap.dcMotor.get("glisiera");
        if (glisiera.getCurrentPosition() <= 25) {
            pozglisiera = 294;
        }
        if (glisiera.getCurrentPosition() >= 290 && brat.getCurrentPosition() <= 50) {
            pozbrat = 134;
        }
        if (glisiera.getCurrentPosition() >= 290 && brat.getCurrentPosition() >= 130 && glisiera.getCurrentPosition() <= 375) {
            pozglisiera = 728;
        }
        if (brat.getCurrentPosition() >= 130 && glisiera.getCurrentPosition() >= 720 && brat.getCurrentPosition() <= 500) {
            pozbrat = 1725;
        }
        if (brat.getCurrentPosition() >= 1720 && glisiera.getCurrentPosition() >= 720) {
            gata = 1;
        }
    }
    private void coboaracaz2() {
        DcMotor brat = hardwareMap.dcMotor.get("brat");
        DcMotor glisiera = hardwareMap.dcMotor.get("glisiera");
        if (brat.getCurrentPosition()>=1715) {
            pozbrat=134;
        }
        if (brat.getCurrentPosition() <= 138 && brat.getCurrentPosition() >= 120) {pozglisiera=294;}
        if (glisiera.getCurrentPosition() <= 298) {pozbrat=0;}
        if (brat.getCurrentPosition() <= 10) {pozglisiera=0;}
        if (glisiera.getCurrentPosition() <= 5 && brat.getCurrentPosition() <= 5) {gata=0;}

    }

    private void ridicacaz3() {
        DcMotor brat = hardwareMap.dcMotor.get("brat");
        DcMotor glisiera = hardwareMap.dcMotor.get("glisiera");
        if (glisiera.getCurrentPosition() <= 25) {
            pozglisiera = 294;
        }
        if (glisiera.getCurrentPosition() >= 290 && brat.getCurrentPosition() <= 50) {
            pozbrat = 134;
        }
        if (glisiera.getCurrentPosition() >= 290 && brat.getCurrentPosition() >= 130 && glisiera.getCurrentPosition() <= 375) {
            pozglisiera = 728;
        }
        if (brat.getCurrentPosition() >= 130 && glisiera.getCurrentPosition() >= 720 && brat.getCurrentPosition() <= 500) {
            pozbrat = 2022;
        }
        if (brat.getCurrentPosition() >= 2017 && glisiera.getCurrentPosition() >= 720) {
            gata = 1;
        }
    }

    private void coboaracaz3() {
        DcMotor brat = hardwareMap.dcMotor.get("brat");
        DcMotor glisiera = hardwareMap.dcMotor.get("glisiera");
        if (brat.getCurrentPosition()>=1975) {
            pozbrat=134;
        }
        if (brat.getCurrentPosition() <= 138 && brat.getCurrentPosition() >= 120) {pozglisiera=294;}
        if (glisiera.getCurrentPosition() <= 298) {pozbrat=0;}
        if (brat.getCurrentPosition() <= 10) {pozglisiera=0;}
        if (glisiera.getCurrentPosition() <= 5 && brat.getCurrentPosition() <= 5) {gata=0;}

    }
}