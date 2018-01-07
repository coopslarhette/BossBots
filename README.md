This is a repository that the Campolindo robotics team (#524) used for part of the '16-'17 First Tech Challenge robotics
competition. 


* See /TeamCode/src/main/java/org/firstinspires/ftc/teamcode/ for the code that we actually wrote to power
our robot. We moved over to a new [repo](https://github.com/stonemao9/BossBots) pretty late in the year due to a bad 
commit that we didn't know how to fix at the time. 

* We believe that the most innovative part of our code design was our 
MecanumOpMode which powered our drive terrain and a somewhat theorized, somewhat implemented PID system which enabled/would have enabled
us to have a highly programmable and highly scablable autonomous system. MecanumOpMode was based on the output of plugging
in the (slightly manipulated) angle of our joystick to a sine or cosine fuction and then mupltiplying that by the distance 
the joystick was from the center to get drive direction and power. This enabled us to have a robot that could move in virutally
any direction without changing direction. PID was a system that was desgined to solve that autonomous part of the challenge.
The brilliance of it was that it would allow us to tell the robot where to go in the arena with a high level of precision.
Unfortunately it was never completed within the '16-17' season, but will be further developed for, and in, the '17-'18 
season.



# FTC App

FTC Android Studio project to create FTC Robot Controller app.

This is the FTC SDK that can be used to create an FTC Robot Controller app, with custom op modes.
The FTC Robot Controller app is designed to work in conjunction with the FTC Driver Station app.
The FTC Driver Station app is available through Google Play.

To use this SDK, download/clone the entire project to your local computer.
Use Android Studio to import the folder  ("Import project (Eclipse ADT, Gradle, etc.)").

Documentation for the FTC SDK are included with this repository.  There is a subfolder called "doc" which contains several subfolders:

 * The folder "apk" contains the .apk files for the FTC Driver Station and FTC Robot Controller apps.
 * The folder "javadoc" contains the JavaDoc user documentation for the FTC SDK.
 * The folder "tutorial" contains PDF files that help teach the basics of using the FTC SDK.

For technical questions regarding the SDK, please visit the FTC Technology forum:

  http://ftcforum.usfirst.org/forumdisplay.php?156-FTC-Technology