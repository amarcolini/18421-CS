package org.firstinspires.ftc.teamcode.opmode

import com.amarcolini.joos.command.CommandOpMode
import com.amarcolini.joos.dashboard.JoosConfig
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.CSRobot
import org.firstinspires.ftc.teamcode.Intake
import org.opencv.core.Core
import org.opencv.core.Mat
import org.opencv.core.Size
import org.opencv.imgproc.Imgproc

@JoosConfig
@TeleOp(group = "lol")
class IntakeTuner : CommandOpMode() {
    private val robot by robot<CSRobot>()

    companion object {
        var intakePosition = 0.5
        var motorPower = 0.0
        var armPosition = 0.9
        var leftPosition = 0.7
        var rightPosition = 0.7
        var gain = 1.0
    }

    override fun preInit() {
        telem.setDisplayFormat(Telemetry.DisplayFormat.HTML)
        robot.intake.motorState = Intake.MotorState.ACTIVE
        robot.intake.servoState = Intake.ServoState.DOWN
        robot.intake.motor.power = 0.0
        schedule(true) {
            robot.intake.motor.power = motorPower
            robot.intake.servo.position = intakePosition
            robot.outtake.armServo.position = armPosition
            robot.outtake.leftServo.position = leftPosition
            robot.outtake.rightServo.position = rightPosition
            val colors = robot.intake.getPixelColors()
            val numPixels = robot.intake.numPixels
            telem.addData("colors", colors)
            telem.addData("numPixels", numPixels)
            telem.addData("leftDist", robot.intake.leftSensor.getDistance(DistanceUnit.MM))
            telem.addData("rightDist", robot.intake.rightSensor.getDistance(DistanceUnit.MM))
        }

        map(gamepad.p1.a0::isJustActivated, ::stopOpMode)
    }
}