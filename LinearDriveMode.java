package org.firstinspires.ftc.teamcode.drive.opmodetele;// un fel de #include, doar că pare că include mai multe biblioteci odată, sau ce o însemna un pachet

import static java.lang.Math.abs; // acum văd că #include biblioteca abs

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.roadrunner.geometry.Pose2d;

// astea cred că sunt bibliotecile specifice ftc-ului,
import org.firstinspires.ftc.teamcode.drive.robot.Robot;

@TeleOp(name="MecanumDriveMode", group="Linear OpMode")
//definiti ceva clasa teleop, pt aplicatia aia de pe telefon sau ce o fi mecanum drivemode
public class LinearDriveMode extends LinearOpMode { // un fel de main, doar fct intr-o clasa.
    private Robot robot = null;// definti si initializati variabila robot de tip Robot
    int direction = 1; // dc nu ma insel driver 1 era cel ce controla bratul ala sau ce o fi el, deci voi acum ii spuneti prin variabila asta ce sa faca fiecare chestie/componenta a macaralei, sau in ce directie sa mearga la driver 1, cand alege directia 1
    double servoPosSlides = 0.5
    double servoPosGrippy = 0;// pozitia unde vrei sa ajunga la driver 1, in directia 1
    public double calculateThrottle(float x) {// conform numelui, functie care calculeaza acceleratia pt macara
        int sign = -1;
        if (x > 0) sign = 1;// presupun ca aici iti spuneti sa tina cont de cat de tare dei de joystick si cine da de joystick-ul de pe consola, pt a misca macaraua cu viteza potrivita mai jos
        return sign * 3 * abs(x);
    }

    @Override// aici e fct sau programul carel face pe OpMode sa mearga
    public void runOpMode() throws InterruptedException {
        telemetry.addData(">", "Initializing...");// conform sintaxei afiseaza ceva, in telemetire ? 
        telemetry.update();

        robot = new Robot(hardwareMap); // poate intializaeaza robotul?
        while (robot.isInitialize() && opModeIsActive()) {
            idle();// buna gluma, e functie anti idle, mentine componenta definita activa
        }
// tot face ceva cu telemetria si dashboardul ftc
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData(">", "Initialized");//confirma initializarea dashboardului?
        telemetry.update();// idk, da update la acceasi chestie



        waitForStart();// asteapta sa porneasca OpModul
        if (isStopRequested()) return;


        while (opModeIsActive()) { //cat timp merge OpModul,  se intampla mai multe chestii



            if (gamepad2.left_bumper) {// dc se apasa left bumper macaraua se duce in directia 1, se misca in partea 5, se pregateste sa se exista 3.3 unitati, se extind 3.3 unitati
                robot.crane.slidesDirection = -1;
                robot.crane.setSlides(5);
                if(robot.crane.slideEncoderLastPosition > robot.crane.slideEncoder.getVoltage()){
                    robot.crane.slideExtension -= 3.3;
                }
            } else if (gamepad2.right_bumper) {// cam la fel, doar ca face referire la situatia in care apesi right bumperul
                robot.crane.slidesDirection = 1;
                robot.crane.setSlides(5);
                if(robot.crane.slideEncoderLastPosition < robot.crane.slideEncoder.getVoltage()){
                    robot.crane.slideExtension += 3.3;
                }
            } else {
               robot.crane.setSlides(0);// else, opreste miscarea dc nu se apasa butoane
            }
            robot.crane.slideEncoderLastPosition = robot.crane.slideEncoder.getVoltage();
//actualizeaza ultima pozitie a macaralei

            if(gamepad2.left_trigger > 0.2){//controleaza pozitia macaralei in sus sau in jos
                robot.crane.craneTarget -= (int) calculateThrottle(gamepad2.left_trigger);
            }
            else if(gamepad2.right_trigger > 0.2){
                robot.crane.craneTarget += (int) calculateThrottle(gamepad2.right_trigger);
            }// presupun ca seteaza puterea motoarelor, pt a fi potrivita pt diversele situatii din acest sezon
            robot.crane.motorCrane1.setPower(robot.crane.cranePower(robot.crane.craneTarget));
            robot.crane.motorCrane2.setPower(robot.crane.cranePower(robot.crane.craneTarget));
// seteaza directia la gripper pt prindere probabil din moment ce e prima serie de cod ( o sai spuna mana )
            if (gamepad2.a) {
                robot.crane.gripperDirection = 1;
                robot.crane.setGripper(1);
            }// la fel, dar ca probabil ca pt eliberarea gripperului
            else if (gamepad2.b) {
                robot.crane.gripperDirection = -1;
                robot.crane.setGripper(1);// -1 e fals, deci probabil ori opreste complet mana, ori o pune intro pozitie mai joasa pt a incepe driveul
            }
            else robot.crane.setGripper(0);// opreste complet masina

            robot.drive.setWeightedDrivePower(new Pose2d((-gamepad1.left_stick_y),(-gamepad1.left_stick_x),(-gamepad1.right_stick_x)));
// nu stiu






// afiseaza ceva, prin telemetrie?
            telemetry.addData("crane target: ", robot.crane.craneTarget);
                telemetry.addData("right trigger: ", gamepad2.right_trigger);
                telemetry.addData("encoder value: ", robot.crane.slideEncoder.getVoltage());
                telemetry.addData("last position ", robot.crane.slideEncoderLastPosition);
                telemetry.addData("slide extension ", robot.crane.slideExtension);
                telemetry.addData("sensor touch: ", robot.crane.slideSensor.isPressed());
//                telemetry.addData("CRANE TICKS LEFT: ", robot.crane.motorCraneLeft.getCurrentPosition());
//                telemetry.addData("CRANE TICKS RIGHT: ", robot.crane.motorCraneRight.getCurrentPosition());
//                telemetry.addData("DIRECTION: ", direction);
//                telemetry.addData("SERVO GRIPPER: ", robot.crane.servoGrippy1.getPosition());
                telemetry.update();
            }

        }

    }



