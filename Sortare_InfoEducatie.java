package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.advanced.ContourPipeline;
import org.firstinspires.ftc.teamcode.drive.advanced.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Autonomous(name="Sortare cu detectie RGB", group = "advanced")
public class Sortare_InfoEducatie extends LinearOpMode {

    private OpenCvCamera webcam;
    private ContourPipeline pipeline;

    private int minRectangleArea = 10;

    //range scalar verde
    public static Scalar scalarLowerYCrCb = new Scalar(0.0, 0.0, 0.0);
    public static Scalar scalarUpperYCrCb = new Scalar(255.0, 120.0, 120.0);

    enum State {
        Inceput,
        ColectareCub,
        LivrareCub,
        ResetareLivrare,
        Stop
    }

    double hold = 0.67, out = 1, in = 0.8; //pozitii servo
    double detectieverde = 0, detectierosu = 0, detectiealbastru = 0; //pozitii x detectie
    double ydetectieverde = 0, ydetectierosu = 0, ydetectiealbastru = 0; //pozitii y detectie
    int pozalbastru = 0, pozverde = 0, pozrosu = 0; //ordine pozitii detectie
    int ypozalbastru = 0, ypozverde = 0, ypozrosu = 0; //ordine pozitii y detectie
    int destinatie;
    int inaltime;
    int pozbrat = 0;
    int pozglisiera = 0;
    int gata = 0;
    int corectare=0;

    State currentState = State.Inceput;

    Pose2d startPose = new Pose2d(-39, -38, Math.toRadians(90));

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDriveCancelable drive = new SampleMecanumDriveCancelable(hardwareMap);
        DcMotor brat = hardwareMap.dcMotor.get("brat");
        DcMotor glisiera = hardwareMap.dcMotor.get("glisiera");
        DcMotor intake = hardwareMap.dcMotor.get("intake");
        Servo kicker = hardwareMap.servo.get("kicker");
        DistanceSensor senzordistanta = hardwareMap.get(DistanceSensor.class, "SenzorCuloare");
        ColorSensor senzorculoare = hardwareMap.get(ColorSensor.class, "SenzorCuloare");

        drive.setPoseEstimate(startPose);

        Trajectory AliniereInceputColectare = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-35.2, -38, Math.toRadians(90)))
                .build();

        Trajectory ColectareCuburi = drive.trajectoryBuilder(AliniereInceputColectare.end())
                .lineToLinearHeading(new Pose2d(-35.2, -62, Math.toRadians(90)), SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                .build();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        pipeline = new ContourPipeline(0.2, 0.2, 0.2, 0.2);

        pipeline.configureScalarLower(scalarLowerYCrCb.val[0], scalarLowerYCrCb.val[1], scalarLowerYCrCb.val[2]);
        pipeline.configureScalarUpper(scalarUpperYCrCb.val[0], scalarUpperYCrCb.val[1], scalarUpperYCrCb.val[2]);

        webcam.setPipeline(pipeline);

        //Webcam Streaming
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

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {

            if (pipeline.error) {
                telemetry.addData("Exception: ", pipeline.debug.getStackTrace());
            }

            while ((detectieverde==0 || detectiealbastru==0 || detectierosu==0) && opModeIsActive()) {
                while (detectieverde == 0 && opModeIsActive()) {
                    double rectangleArea = pipeline.getRectArea();
                    if (rectangleArea > minRectangleArea) {
                        detectieverde = pipeline.getRectMidpointX();
                        ydetectieverde = pipeline.getRectMidpointY();
                    }
                }

                while (detectierosu == 0 && opModeIsActive()) {
                    //range scalar rosu
                    scalarLowerYCrCb = new Scalar(0.0, 190.0, 0.0);
                    scalarUpperYCrCb = new Scalar(255.0, 255.0, 128.0);

                    pipeline.configureScalarLower(scalarLowerYCrCb.val[0], scalarLowerYCrCb.val[1], scalarLowerYCrCb.val[2]);
                    pipeline.configureScalarUpper(scalarUpperYCrCb.val[0], scalarUpperYCrCb.val[1], scalarUpperYCrCb.val[2]);

                    webcam.setPipeline(pipeline);
                    sleep(2000);
                    double rectangleArea = pipeline.getRectArea();
                    if (rectangleArea > minRectangleArea) {
                        detectierosu = pipeline.getRectMidpointX();
                        ydetectierosu = pipeline.getRectMidpointY();
                    }
                }

                while (detectiealbastru == 0 && opModeIsActive()) {
                    //range scalar albastru
                    scalarLowerYCrCb = new Scalar(0.0, 0.0, 160.0);
                    scalarUpperYCrCb = new Scalar(255.0, 130.0, 255.0);

                    pipeline.configureScalarLower(scalarLowerYCrCb.val[0], scalarLowerYCrCb.val[1], scalarLowerYCrCb.val[2]);
                    pipeline.configureScalarUpper(scalarUpperYCrCb.val[0], scalarUpperYCrCb.val[1], scalarUpperYCrCb.val[2]);

                    webcam.setPipeline(pipeline);
                    sleep(2000);
                    double rectangleArea = pipeline.getRectArea();
                    if (rectangleArea > minRectangleArea) {
                        detectiealbastru = pipeline.getRectMidpointX();
                        ydetectiealbastru = pipeline.getRectMidpointY();
                    }
                }

            }

            if (detectiealbastru<detectieverde && detectiealbastru<detectierosu) {pozalbastru=1;}
            else if (detectiealbastru>detectieverde && detectiealbastru>detectierosu) {pozalbastru=3;}
            else {pozalbastru=2;}
            if (detectieverde<detectiealbastru && detectieverde<detectierosu) {pozverde=1;}
            else if (detectieverde>detectiealbastru && detectieverde>detectierosu) {pozverde=3;}
            else {pozverde=2;}
            pozrosu = 6 - pozalbastru - pozverde;

            if (ydetectiealbastru<ydetectieverde && ydetectiealbastru<ydetectierosu) {ypozalbastru=1;}
            else if (ydetectiealbastru>ydetectieverde && ydetectiealbastru>ydetectierosu) {ypozalbastru=3;}
            else {ypozalbastru=2;}
            if (ydetectieverde<ydetectiealbastru && ydetectieverde<ydetectierosu) {ypozverde=1;}
            else if (ydetectieverde>ydetectiealbastru && ydetectieverde>ydetectierosu) {ypozverde=3;}
            else {ypozverde=2;}
            ypozrosu = 6 - ypozalbastru - ypozverde;

            closeCameraThread.start();

            //control brat
            brat.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            if (brat.getCurrentPosition() == pozbrat) {
                brat.setPower(0);
            }
            if (brat.getCurrentPosition() != pozbrat) {
                brat.setTargetPosition(pozbrat);
                brat.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                brat.setPower(0.6);
            }

            //control glisiera
            glisiera.setDirection(DcMotorSimple.Direction.FORWARD);
            glisiera.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            if (glisiera.getCurrentPosition() == pozglisiera) {
                glisiera.setPower(0);
            }
            if (glisiera.getCurrentPosition() != pozglisiera) {
                glisiera.setTargetPosition(pozglisiera);
                glisiera.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                glisiera.setPower(0.6);
            }

            switch (currentState) {
                case Inceput:
                    drive.followTrajectory(AliniereInceputColectare);
                    currentState= State.ColectareCub;
                    break;

                case ColectareCub:
                    if (senzordistanta.getDistance(DistanceUnit.CM) > 5 && !drive.isBusy()) {
                        kicker.setPosition(in);
                        intake.setPower(-0.6);
                        drive.followTrajectoryAsync(ColectareCuburi);
                    }
                    if (senzordistanta.getDistance(DistanceUnit.CM) <= 5) {
                        sleep(100);
                        kicker.setPosition(hold);
                        intake.setPower(0);
                        drive.breakFollowing();
                        drive.setDrivePower(new Pose2d());
                        int albastru = senzorculoare.blue();
                        int rosu = senzorculoare.red();
                        int verde = senzorculoare.green();
                        if (albastru>rosu && albastru>verde) {destinatie=pozalbastru;inaltime=ypozalbastru;}
                        else if (rosu>albastru && rosu>verde) {destinatie=pozrosu;inaltime=ypozrosu;}
                        else {destinatie=pozverde;inaltime=ypozverde;}
                        currentState= State.LivrareCub;
                    }
                    if (drive.getPoseEstimate().getY()<ColectareCuburi.end().getY()+3) {currentState= State.Stop;}
                    break;

                case LivrareCub:
                    if (inaltime==1) {ridica();}
                    if (inaltime==2) {ridicacaz2();}
                    if (inaltime==3) {ridicacaz3();}
                    switch (destinatie) {
                        case 1:
                            if (inaltime==1) {
                                TrajectorySequence DepozitStanga = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                        .forward(7)
                                        .lineToLinearHeading(new Pose2d(-61, drive.getPoseEstimate().getY()+7, Math.toRadians(90)))
                                        .lineToLinearHeading(new Pose2d(-61, 14+corectare/2, Math.toRadians(90)))
                                        .build();
                                drive.followTrajectorySequenceAsync(DepozitStanga);}
                            if (inaltime==2) {
                                TrajectorySequence DepozitStanga = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                        .forward(7)
                                        .lineToLinearHeading(new Pose2d(-61, drive.getPoseEstimate().getY()+7, Math.toRadians(90)))
                                        .lineToLinearHeading(new Pose2d(-61, 18+corectare/2, Math.toRadians(90)))
                                        .build();
                                drive.followTrajectorySequenceAsync(DepozitStanga);}
                            if (inaltime==3) {
                                TrajectorySequence DepozitStanga = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                        .forward(7)
                                        .lineToLinearHeading(new Pose2d(-61, drive.getPoseEstimate().getY()+7, Math.toRadians(90)))
                                        .lineToLinearHeading(new Pose2d(-61, 20+corectare/2, Math.toRadians(90)))
                                        .build();
                                drive.followTrajectorySequenceAsync(DepozitStanga);
                            }
                            destinatie=0;
                            break;

                        case 2:
                            if (inaltime==1) {
                            Trajectory DepozitMijloc = drive.trajectoryBuilder(drive.getPoseEstimate())
                                    .lineToLinearHeading(new Pose2d(-37, 14+corectare, Math.toRadians(90)))
                                    .build();
                            drive.followTrajectoryAsync(DepozitMijloc);}
                            if (inaltime==2) {
                                Trajectory DepozitMijloc = drive.trajectoryBuilder(drive.getPoseEstimate())
                                        .lineToLinearHeading(new Pose2d(-37, 18+corectare, Math.toRadians(90)))
                                        .build();
                                drive.followTrajectoryAsync(DepozitMijloc);}
                            if (inaltime==3) {
                                Trajectory DepozitMijloc = drive.trajectoryBuilder(drive.getPoseEstimate())
                                        .lineToLinearHeading(new Pose2d(-37, 20+corectare, Math.toRadians(90)))
                                        .build();
                                drive.followTrajectoryAsync(DepozitMijloc);
                            }
                            destinatie=0;
                            break;

                        case 3:
                            if (inaltime==1) {
                                TrajectorySequence DepozitDreapta = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                        .forward(7)
                                        .lineToLinearHeading(new Pose2d(-11, drive.getPoseEstimate().getY()+7, Math.toRadians(90)))
                                        .lineToLinearHeading(new Pose2d(-12-corectare, 14+corectare/2, Math.toRadians(90)))
                                        .build();
                                drive.followTrajectorySequenceAsync(DepozitDreapta);}
                            if (inaltime==2) {
                                TrajectorySequence DepozitDreapta = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                        .forward(7)
                                        .lineToLinearHeading(new Pose2d(-11, drive.getPoseEstimate().getY()+7, Math.toRadians(90)))
                                        .lineToLinearHeading(new Pose2d(-12-corectare, 18+corectare/2, Math.toRadians(90)))
                                        .build();
                                drive.followTrajectorySequenceAsync(DepozitDreapta);}
                            if (inaltime==3) {
                                TrajectorySequence DepozitDreapta = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                        .forward(7)
                                        .lineToLinearHeading(new Pose2d(-11, drive.getPoseEstimate().getY()+7, Math.toRadians(90)))
                                        .lineToLinearHeading(new Pose2d(-12-corectare, 20+corectare/2, Math.toRadians(90)))
                                        .build();
                                drive.followTrajectorySequenceAsync(DepozitDreapta);
                            }
                            destinatie=0;
                            break;

                        case 0:

                            break;
                    }
                    if (!drive.isBusy() && gata == 1) {
                        kicker.setPosition(out);
                        TrajectorySequence IntoarcereLaColectare = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                                .lineToLinearHeading(new Pose2d(drive.getPoseEstimate().getX(), AliniereInceputColectare.end().getY(), Math.toRadians(90)))
                                .lineToLinearHeading(AliniereInceputColectare.end())
                                .build();
                        drive.followTrajectorySequenceAsync(IntoarcereLaColectare);
                        corectare=corectare+1;
                        currentState = State.ResetareLivrare;
                    }
                    break;

                case ResetareLivrare:
                    if (inaltime==1) {coboara();}
                    if (inaltime==2) {coboaracaz2();}
                    if (inaltime==3) {coboaracaz3();}
                    if (!drive.isBusy() && gata == 0) {
                        currentState = State.ColectareCub;
                    }
                    break;

                case Stop:
                    stop();
                    break;
            }
            drive.update();
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
            pozbrat = 1410;
        }
        if (brat.getCurrentPosition() >= 1390) {
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
            pozbrat = 1625;
        }
        if (brat.getCurrentPosition() >= 1620 && glisiera.getCurrentPosition() >= 720) { ///1720
            gata = 1;
        }
    }
    private void coboaracaz2() {
        DcMotor brat = hardwareMap.dcMotor.get("brat");
        DcMotor glisiera = hardwareMap.dcMotor.get("glisiera");
        if (brat.getCurrentPosition()>=1615) {
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
            pozbrat = 1872;
        }
        if (brat.getCurrentPosition() >= 1867 && glisiera.getCurrentPosition() >= 720) {
            gata = 1;
        }
    }

    private void coboaracaz3() {
        DcMotor brat = hardwareMap.dcMotor.get("brat");
        DcMotor glisiera = hardwareMap.dcMotor.get("glisiera");
        if (brat.getCurrentPosition()>=1860) {
            pozbrat=134;
        }
        if (brat.getCurrentPosition() <= 138 && brat.getCurrentPosition() >= 120) {pozglisiera=294;}
        if (glisiera.getCurrentPosition() <= 298) {pozbrat=0;}
        if (brat.getCurrentPosition() <= 10) {pozglisiera=0;}
        if (glisiera.getCurrentPosition() <= 5 && brat.getCurrentPosition() <= 5) {gata=0;}

    }
}