//   ####    ###                                                                                              ###                ###
//   ####    ###                                                                                              ###                ###
//   ####    ###                      ###                                                                     ###                ###
//   #####   ###                      ###                                                                     ###
//   #####   ###                      ###                                                                     ###
//   ######  ###         #            ###                                            #                 ##     ###                                 ##           ##   ###
//   ######  ###      #######      ##########                   ###  ###  ###     #######        ########     ###    ####        ###        ### ######       ##########
//   ### ##  ###     #########     ##########                   ###  ###  ###    #########       ########     ###   ####         ###        ###########     ###########
//   ### ### ###     ###   ####       ###                        ### #### ###    ###   ####      ########     ###  ####          ###        #####   ###     ###  ####
//   ###  ## ###    ###     ###       ###                        ######## ##    ###     ###      ####         ### ###            ###        ####    ###     ###   ###
//   ###  ######    ###     ###       ###                        ##### #####    ###     ###      ###          ########           ###        ####    ###     ###   ###
//   ###  ######    ###     ###       ###                        ##### #####    ###     ###      ###          ########           ###        ###     ###     ########
//   ###   #####    ###     ###       ###                        ##### #####    ###     ###      ###          #### ####          ###        ###     ###     ########
//   ###   #####    ###     ###       ###                         #### ####     ###     ###      ###          ###   ###          ###        ###     ###     ######
//   ###    ####    ####   ####       ###    #                    #### ####     ####   ####      ###          ###    ###         ###        ###     ###    ########
//   ###    ####     #########        ########                    #### ####      #########       ###          ###    ####        ###        ###     ###     #########
//   ###    ####      #######          #######                    ####  ###       #######        ###          ###     ###        ###        ###     ###     ##########
//   ###     ###         ##              ###                       ##   ###          ##          ###          ###     ####       ###        ###     ###    ###     ###
package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.driving.NewMecanumDrive;

@Config
@TeleOp(name = "testOpmode", group = "tests")
public class testOpMode extends LinearOpMode {
    @Override
    public void runOpMode(){
        NewMecanumDrive drive = new NewMecanumDrive(hardwareMap);
        waitForStart();
        while(opModeIsActive()){
            drive.update();
            drive.updateOdo();
            drive.setFieldCentric(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, 1);
        }
    }
}