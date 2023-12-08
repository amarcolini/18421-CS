package org.firstinspires.ftc.teamcode.opmode

import com.amarcolini.joos.command.Command
import com.amarcolini.joos.command.CommandOpMode
import com.amarcolini.joos.dashboard.JoosConfig
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.CSRobot

@JoosConfig
@TeleOp(group = "lol")
class IntakeTuner : CommandOpMode() {
    private val robot by robot<CSRobot>()

    companion object {
        var targetPosition = 0.0
        var motorPower = 0.0
    }

    override fun preInit() {
        schedule(true) {
            robot.intake.motor.power = motorPower
            robot.intake.servo.position = targetPosition
        }

        map(gamepad.p1.run { a or cross }::justActivated, ::stopOpMode)
    }
}