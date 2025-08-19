package Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.rowanmcalpin.nextftc.core.command.utility.delays.Delay;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;

/**
 * This is an example auto that showcases movement and control of two servos autonomously.
 * It is a 0+4 (Specimen + Sample) bucket auto. It scores a neutral preload and then pickups 3 samples from the ground and scores them before parking.
 * There are examples of different ways to build paths.
 * A path progression method has been created and can advance based on time, position, or other factors.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 11/28/2024
 */

@Autonomous(name = "Specimen Auto States")
public class SpecimenAutoStates extends OpMode {

    private DcMotor frontLeft, backLeft, frontRight, backRight, extension;
    private DcMotor intake, rightVerticalMotor, leftVerticalMotor;
    private Servo extendDepo, depoLeft, depoRight, claw, leftHanger, rightHanger, holdChute, intakeTilt, wristClaw;
    private TouchSensor vertSwitch, hortSwitch;
    private ColorSensor colorChute;
    private boolean slidesOff = false;
    private boolean timerCondition = false;

    long currentTimer;


    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState;

    /* Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume our robot is 18 by 18 inches
     * Lets assume the Robot is facing the human player and we want to score in the bucket */

    /** Start Pose of our robot */
    private final Pose startPose = new Pose(9, 69-5.625, Math.toRadians(90));

    /** Scoring Pose of our robot.  */
    private final Pose scorePose = new Pose(34.3, 69, Math.toRadians(180));
    private final Pose scorePoseSpeed = new Pose(34.3, 68.5, Math.toRadians(180));

    /** Push the samples on the ground**/
    private final Pose lineUpControl = new Pose (19,36,Math.toRadians(180));
    private final Pose lineUp = new Pose(54 ,32.5,Math.toRadians(180));
    private final Pose firstPush = new Pose(22,27, Math.toRadians(180));
    private final Pose goBackControl = new Pose(52.9,32.9, Math.toRadians(180));
    private final Pose goBack = new Pose(54,20, Math.toRadians(180));
    private final Pose secondPush = new Pose(20, 20, Math.toRadians(180));

    private final Pose goBack2Control = new Pose(54.6,24.9, Math.toRadians(180));
    private final Pose goBack2 = new Pose(54,12, Math.toRadians(180));
    private final Pose thirdPush = new Pose(20, 12, Math.toRadians(180)); //try 15 for x to go faster

    private final Pose goBack3 = new Pose(14.3,12, Math.toRadians(180));


    /** Block pickup pose**/
    private final Pose pickup1Pose = new Pose(9.75, 35, Math.toRadians(180));

    /** Middle (Second) Sample from the Spike Mark */
    private final Pose pickup3Pose = new Pose(49, 125, Math.toRadians(90));

    /** Highest (Third) Sample from the Spike Mark */
    private final Pose pickup2Pose = new Pose(40, 135, Math.toRadians(0));

    /** Park Pose for our robot, after we do all of the scoring. */
    private final Pose parkPose = new Pose(63, 95, Math.toRadians(-90));

    /** Park Control Pose for our robot, this is used to manipulate the bezier curve that we will create for the parking.
     * The Robot will not go to this pose, it is used a control point for our bezier curve. */
    private final Pose parkControlPose = new Pose(69, 112, Math.toRadians(-90));

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path  park;
    private PathChain scorePreload, grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3, scorePickup4,  pushChain;

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {

        /* There are two major types of paths components: BezierCurves and BezierLines.
         *    * BezierCurves are curved, and require >= 3 points. There are the start and end points, and the control points.
         *    - Control points manipulate the curve between the start and end points.
         *    - A good visualizer for this is [this](https://pedro-path-generator.vercel.app/).
         *    * BezierLines are straight, and require 2 points. There are the start and end points.
         * Paths have can have heading interpolation: Constant, Linear, or Tangential
         *    * Linear heading interpolation:
         *    - Pedro will slowly change the heading of the robot from the startHeading to the endHeading over the course of the entire path.
         *    * Constant Heading Interpolation:
         *    - Pedro will maintain one heading throughout the entire path.
         *    * Tangential Heading Interpolation:
         *    - Pedro will follows the angle of the path such that the robot is always driving forward when it follows the path.
         * PathChains hold Path(s) within it and are able to hold their end point, meaning that they will holdPoint until another path is followed.
         * Here is a explanation of the difference between Paths and PathChains <https://pedropathing.com/commonissues/pathtopathchain.html> */

        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */

        // Scored preloaded block
        scorePreload = follower.pathBuilder()
                //TODO Score Preload
                .addPath(new BezierLine(new Point(startPose), new Point(scorePose)))

                .addParametricCallback(0.1, () -> slidesRunUP(1400))
                .addParametricCallback(0.1, this::  specimenClip)
                .addParametricCallback(0.89, ()-> slidesDownTime(0.3))
                .addParametricCallback(1, this::openClaw)
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())

                //TODO 3 push chain and pickup
                .addPath(new BezierCurve(new Point(scorePose), new Point(lineUpControl), new Point(lineUp)))
                .addParametricCallback(0.05, this::depoReset)
                .addParametricCallback(0.14, this::depoStop)
                .setLinearHeadingInterpolation(scorePose.getHeading(), lineUp.getHeading())


                //FINDS THE SHORTEST LINE BETWEEN THE TWO POINTS
                .addPath(new BezierLine(new Point(lineUp)
                        ,new Point(firstPush)))
                .setLinearHeadingInterpolation(lineUp.getHeading()
                        , firstPush.getHeading())
                //FINDS THE CLOSEST PATH TO THE END POINT
                // WHILE GOING AROUND CONTROL POINT
                .addPath(new BezierCurve(new Point(firstPush),
                        new Point(goBackControl), new Point(goBack)) )
                .setLinearHeadingInterpolation(firstPush.getHeading(),
                        goBack.getHeading())

                .addPath(new BezierLine(new Point(goBack),new Point(secondPush)))
                .setLinearHeadingInterpolation(goBack.getHeading(), secondPush.getHeading())

                .addPath(new BezierCurve(new Point(secondPush), new Point(goBack2Control), new Point(goBack2)))
                .setLinearHeadingInterpolation(secondPush.getHeading(), goBack2.getHeading())
                .addParametricCallback(0.1, ()-> slidesRunUP(125))

                .addPath(new BezierLine(new Point(goBack2),new Point(thirdPush)))
                .addParametricCallback(0.1, this::pickupSpecimen)
                .setLinearHeadingInterpolation(goBack2.getHeading(), thirdPush.getHeading())

                .addPath(new BezierLine(new Point(thirdPush),new Point(goBack3)))
                .setLinearHeadingInterpolation(thirdPush.getHeading(), goBack3.getHeading())


                .build();

        /* Here is an example for Constant Interpolation
        scorePreload.setConstantInterpolation(startPose.getHeading()); */


        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */



        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                //TODO Score 1

                .addPath(new BezierLine(new Point(goBack3), new Point(scorePose)))
                .setLinearHeadingInterpolation(goBack3.getHeading(), scorePose.getHeading())
              //
                //  .addParametricCallback(0.05, this::  intermediateArmPosition)
                .addParametricCallback(0.05, () -> slidesRunUP(1300))
              //  .addParametricCallback(0.11, this::intermediateArmPosition)
                .addParametricCallback(0.88, ()-> slidesDownTime(0.45)) //TODO change back to 3 if too slow
                .addParametricCallback(0.65, this::  specimenClip)
                .addParametricCallback(1, this::openClaw)
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup1Pose)))
                .addParametricCallback(0.1, this::depoReset)
                .addParametricCallback(0.3, this::depoStop)
                .addParametricCallback(0.4,() -> slidesRunUP(125))
                .addParametricCallback(0.6, this::pickupSpecimen)
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())

                .build();





        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup1Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .addParametricCallback(0.2, this::  specimenClip)
                .addParametricCallback(0.05, () -> slidesRunUP(1400))
                //  .addParametricCallback(0.11, this::intermediateArmPosition)
                .addParametricCallback(0.87, ()-> slidesDownTime(0.45))
                .addParametricCallback(1, this::openClaw)
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup1Pose)))
                .addParametricCallback(0.1, this::depoReset)
                .addParametricCallback(0.3, this::depoStop)
                .addParametricCallback(0.4,() -> slidesRunUP(125))
                .addParametricCallback(0.6, this::pickupSpecimen)
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */


        /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup1Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .addParametricCallback(0.2, this::  specimenClip)
                .addParametricCallback(0.05, () -> slidesRunUP(1400))
                //  .addParametricCallback(0.11, this::intermediateArmPosition)
                .addParametricCallback(0.87, ()-> slidesDownTime(0.45))
                .addParametricCallback(1, this::openClaw)
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup1Pose)))
                .addParametricCallback(0.1, this::depoReset)
                .addParametricCallback(0.3, this::depoStop)
                .addParametricCallback(0.4,() -> slidesRunUP(125))
                .addParametricCallback(0.6, this::pickupSpecimen)
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        scorePickup4 = follower.pathBuilder()
                .addPath(new BezierLine(new Point(pickup1Pose), new Point(scorePose)))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .addParametricCallback(0.2, this::  specimenClip)
                .addParametricCallback(0.05, () -> slidesRunUP(1400))
                //  .addParametricCallback(0.11, this::intermediateArmPosition)
                .addParametricCallback(0.87, ()-> slidesDownTime(0.45))
                .addParametricCallback(1, this::openClaw)
                .addPath(new BezierLine(new Point(scorePose), new Point(pickup1Pose)))
                .addParametricCallback(0.1, this::depoReset)
                .addParametricCallback(0.3, this::depoStop)
                .addParametricCallback(0.4,() -> slidesRunUP(125))
                .addParametricCallback(0.6, this::pickupSpecimen)
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        /* This is our park path. We are using a BezierCurve with 3 points, which is a curved line that is curved based off of the control point */
        park = new Path(new BezierLine(new Point(scorePose), new Point(pickup1Pose)));
        park.setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading());
    }

    /** This switch is called continuously and runs the pathing, at certain points, it triggers the action state.
     * Everytime the switch changes case, it will reset the timer. (This is because of the setPathState() method)
     * The followPath() function sets the follower to run the specific path, but does NOT wait for it to finish before moving on. */
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload,true);
                setPathState(1);
                break;

            case 1:
                if(!follower.isBusy()){

                    if(!timerCondition) {
                         currentTimer = opmodeTimer.getElapsedTime();
                         timerCondition = true;
                     }

                    while(currentTimer + 700 > opmodeTimer.getElapsedTime()) {

                   }
                    closeClaw();
                    while(currentTimer + 200 > opmodeTimer.getElapsedTime()) {

                    }
                    intermediateArmPosition();

                    follower.followPath(scorePickup1);
                    setPathState(2);
                }
            case 2:
                if(!follower.isBusy()){
                    closeClaw();
                    follower.followPath(scorePickup2);
                    setPathState(3);
                }
            case 3:
                if(!follower.isBusy()){
                    closeClaw();
                    follower.followPath(scorePickup3);
                    setPathState(4);
                }
            case 4:
                if(!follower.isBusy()){
                    closeClaw();
                    follower.followPath(scorePickup4);
                    setPathState(-1);
                }


            //Pickup1 is skipped until the pushChain

        }
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        if(slidesOff){
            telemetry.addData("Slides", "Off");
        }
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {

        initHw();
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(startPose);

        buildPaths();
        closeClaw();
        //pickupSpecimen();
        //specimenClip();

        wristClaw.setPosition(0.96);
        //pickupSpecimen();
       // intakeTilt.setPosition(0.65);


    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }

    public void slidesRunUP(int pos){
        leftVerticalMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightVerticalMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftVerticalMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightVerticalMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftVerticalMotor.setTargetPosition(pos);
        rightVerticalMotor.setTargetPosition(pos);


        leftVerticalMotor.setPower(1);
        rightVerticalMotor.setPower(1);


        leftVerticalMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightVerticalMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);





    }
    public void slidesDownTime(double time){
        long timer = opmodeTimer.getElapsedTime();
        leftVerticalMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightVerticalMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while((timer+time*1000) >  opmodeTimer.getElapsedTime()){
            leftVerticalMotor.setPower(-1);
            rightVerticalMotor.setPower(-1);

            telemetry.addData("Slides Timer: ", opmodeTimer.getElapsedTime());
            telemetry.update();
        }
        leftVerticalMotor.setPower(0);
        rightVerticalMotor.setPower(0);



    }



    public void depoReset(){
        depoRight.setPosition(0.91);
     //   depoLeft.setPosition(0.91);
        wristClaw.setPosition(0.31); //0.8


        leftVerticalMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightVerticalMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftVerticalMotor.setPower(-1);
        rightVerticalMotor.setPower(-1);
    }

    public void depoResetNOSlide() {
        depoLeft.setPosition(0.97);
        depoRight.setPosition(0.03);
        extendDepo.setPosition(0.71);
        wristClaw.setPosition(0.6);


        leftVerticalMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightVerticalMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftVerticalMotor.setPower(0);
        rightVerticalMotor.setPower(0);

        slidesOff = true;

    }




    public void depoStop(){
        leftVerticalMotor.setPower(0);
        rightVerticalMotor.setPower(0);
        leftVerticalMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightVerticalMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addData("Slides:", "Stopped" );
        telemetry.update();
    }

    public void pickupSpecimen(){
        depoRight.setPosition(.91);
        //depoLeft.setPosition(0.91);
        wristClaw.setPosition(0.31); //0.8
        extendDepo.setPosition(0.4);//0.625


    }

    public void intermediateArmPosition(){
    //    depoLeft.setPosition(0.5);
        depoRight.setPosition(0.5);
        extendDepo.setPosition(.58);
        wristClaw.setPosition(0.96);

    }



    public void specimenClip(){
        depoRight.setPosition(0.1);
      //  depoLeft.setPosition(0.1);
        extendDepo.setPosition(0.58);
        wristClaw.setPosition(0.96);


    }

    public void specimenClipFirst(){
        depoLeft.setPosition(0.3);
        depoRight.setPosition(0.7);
        extendDepo.setPosition(.67);
        wristClaw.setPosition(0.96); // servo facin up on rotation
        // servo facin down   wristClaw.setPosition(0.45);

    }

    public void closeClaw(){
        claw.setPosition(0.65);
    }



    public void openClaw() {
        claw.setPosition(0.48);
    }

    public void initHw(){
        intake = hardwareMap.dcMotor.get("intake");
        extendDepo = hardwareMap.servo.get("extendDepo");
        //extendDepo.setDirection(Servo.Direction.REVERSE);
        depoRight = hardwareMap.servo.get("RightDepo");
        depoRight.setDirection(Servo.Direction.REVERSE);
        depoLeft = hardwareMap.servo.get("LeftDepo");
        claw = hardwareMap.servo.get("claw");
        holdChute = hardwareMap.servo.get("holdChute");

        leftHanger = hardwareMap.servo.get("leftHang");
        rightHanger = hardwareMap.servo.get("rightHang");

        intakeTilt = hardwareMap.servo.get("intakeTilt");
        wristClaw = hardwareMap.servo.get("wristClaw");


        rightVerticalMotor = hardwareMap.dcMotor.get("rightVert");
        leftVerticalMotor = hardwareMap.dcMotor.get("leftVert");
        extension = hardwareMap.dcMotor.get("extension");

        vertSwitch = hardwareMap.touchSensor.get("hortSwitch");//Todo: Changed cause screwed in wiring
        hortSwitch = hardwareMap.touchSensor.get("vertSwitch");

        colorChute = hardwareMap.get(ColorSensor.class, "colorChute");

        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backRight = hardwareMap.dcMotor.get("backRight");

        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);

        leftVerticalMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightVerticalMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftVerticalMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightVerticalMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightVerticalMotor.setDirection(DcMotor.Direction.FORWARD);
        leftVerticalMotor.setDirection(DcMotor.Direction.REVERSE);

    }

    private void tangentialSplineScore(){
        //new BezierCurve(
        //          new Point(34.300, 69.000, Point.CARTESIAN),
        //          new Point(29.728, 69.351, Point.CARTESIAN),
        //          new Point(29.212, 35.227, Point.CARTESIAN),
        //          new Point(11.750, 35.000, Point.CARTESIAN)
        //        )
        //      )
        //      .setTangentHeadingInterpolation();
    }
    private void tangentSplineToStopScore(){
      //new BezierCurve(
        //          new Point(34.300, 69.000, Point.CARTESIAN),
        //          new Point(18.387, 68.391, Point.CARTESIAN),
        //          new Point(32.134, 35.227, Point.CARTESIAN),
        //          new Point(11.750, 35.000, Point.CARTESIAN)
        //        )
        //      )
        //      .setTangentHeadingInterpolation();
    }


}

