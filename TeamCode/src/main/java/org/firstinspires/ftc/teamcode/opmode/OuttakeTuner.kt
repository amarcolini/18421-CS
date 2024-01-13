package org.firstinspires.ftc.teamcode.opmode

import com.amarcolini.joos.command.CommandOpMode
import com.amarcolini.joos.dashboard.JoosConfig
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.CSRobot

@JoosConfig
@TeleOp(group = "lol")
class OuttakeTuner : CommandOpMode() {
    private val robot by robot<CSRobot>()

    companion object {
        var armPosition = 0.0
        var leftPosition = 0.5
        var rightPosition = 0.5
    }

    override fun preInit() {
        schedule(true) {
            robot.outtake.armServo.position = armPosition
            robot.outtake.leftServo.position = leftPosition
            robot.outtake.rightServo.position = rightPosition
        }

        map(gamepad.p1.a0::isJustActivated, ::stopOpMode)
    }
}