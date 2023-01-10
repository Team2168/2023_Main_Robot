# 2023_Main_Robot

## Software Setup

### Install WPILib:

Download the latest release from: https://github.com/wpilibsuite/allwpilib/releases

Follow the installation steps here: https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html

A shortcut should be added to your desktop named `2023 WPILib VS Code`

### Clone the repo:

From `git bash`:
  * Change directories (`cd`) to where you want a copy of the project created (e.g. C:\Users\\\<username>\Documents\FRC or wherever is convenient/memorable for you) 
  * `git clone https://github.com/Team2168/2023_Main_Robot.git`

### Open the repo in VS Code:

* From the menu bar, click `File`
* Select `Open Folder...` or `Add Folder to Workspace...`
* Navigate to the folder where you cloned the repo

### Install vendor dependencies:

* CTRE: Support for F500s, Talon Motor controllers, CANCoders, Pigeon IMU, etc.

  Download and install the latest release of the Pheonix framework here: https://github.com/CrossTheRoadElec/Phoenix-Releases/releases

* REV Robotics: for the SPARK MAX
  Follow the Java instructions here: https://docs.revrobotics.com/sparkmax/software-resources/spark-max-api-information#java-api
  
For more information see here: https://docs.wpilib.org/en/stable/docs/software/vscode-overview/3rd-party-libraries.html
  
### Install imaging tools:

  * REV Hardware Client: https://www.revrobotics.com/software/
    * After installing run an update - yeah even if you just installed it.
  * Follow the instructions here: https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/frc-game-tools.html
    This will include stuff like the RIO Imaging tool (to load firmware onto the roboRIO) and the DriverStation application.


## Common Tasks in VS Code

### Videos

Videos for the above can be found at the following playlist  
https://www.youtube.com/playlist?list=PLUTJdMwEWueIyWRVWkQE8N3XxPGucEx0Q

### To set up git credentials

* Open git bash
* Set global name to your name by typing `git config --global user.name "David Buslegs"
* Set global email to your email by typing `git config --global user.email "Buslegs@team2168.org"

### To commit

* Use the source control pane
* Add files to the commit using the `+` symbol
* Add a commit message and hit the check symbol
* Push using the menu

### To build the code

* From within VS Code 2023, open the command pallet (Ctrl+Shift+P or click the WPI logo in the top right)
* Type `Build` and select `WPILib: Build Robot Code`
* If you code is syntactically correct (it can be compiled), you should see `BUILD SUCCESSFUL` displayed in the terminal  
    IF there are compilation errors you should recieve some diagnostic message identifying what the error is and in which file(s) it resides
    
### To test in sim

  * Being able to compile code doesn't mean it will do what you want.
    It's important to verify that your code does what you expect.  
    This can be challenging without the real robot & real hardware, but WPILib now ships with simulation functionality,
    allowing you to find runtime errors without having access to a physical robot

  * From within VS Code 2023, open the command pallet (Ctrl+Shift+P or click the WPI logo in the top right)
  * Type `Simulate` and select `WPILib: Simulate Robot Code`

### To deploy the code

  If your code compiles & runs in simulation, and you're ready to deploy code to the robot: 

  * Join the WiFi network for the robot you want to deploy code to
    * Its roboRIO needs 2023-compatible firmware flashed

  * From within VS Code 2023, open the command pallet (Ctrl+Shift+P or click the WPI logo in the top right)
  * Type `Deploy` and select `WPILib: Deploy Robot Code`
  * The code will build, be uploaded to the roboRIO, and execute

  * Before running your code:
     * Clear the area of people, tools, or debris that might be at risk of being hit
     * Talk to people in the area; make sure they know what you plan on doing / what the robot may do
   * Open the DriverStation
     * Double-check you're in the right game mode (Practice/Auto/Teloperated)
     * Audibly identify you're enabling the robot (yell `ENABLING - WATCH YOUR FACE`)
     * Enable the robot


## Requirements for Robot

1. RoboRIO must be flashed to latest 2023 image using USB (this only needs to be done once for the season): (https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-3/roborio2-imaging.html)
2. Radio must be programmed (https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-3/radio-programming.html)
3. RoboRIO IP on ethernet must be set to static using web dashboard to: 10.21.68.2

## Requirements for Students

https://docs.wpilib.org is the official documentation from FRC on the control system: Please give it a read:  
https://docs.wpilib.org/en/stable/docs/controls-overviews/control-system-hardware.html

1. If you would like to have the driverstation on your computer as well then install NI Update suite, but this is not a requirement to develop or deploy programs, only to flash robot images (https://www.ni.com/en-us/support/downloads/drivers/download.frc-game-tools.html#333285)
2. For returning students, and new students interested: understand what has changed in WPI Library since 2022 season (https://docs.wpilib.org/en/stable/docs/yearly-overview/yearly-changelog.html)
3. Understand how the robot is wired as it affects your code (https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-1/how-to-wire-a-robot.html)

## Cool things to know

### Everything you need to know about the control system is here:
  - WPILib readthedocs: https://docs.wpilib.org/en/latest/
  - CTRE Pheonix API
    - Readthedocs: https://phoenix-documentation.readthedocs.io/
    - Javadocs: https://api.ctr-electronics.com/phoenix/release/java/
    - Software downloads and manuals: https://www.ctr-electronics.com/talon-srx.html#product_tabs_technical_resources
  - Spark MAX
    - Documentation: https://docs.revrobotics.com/sparkmax/
    - Javadocs: https://codedocs.revrobotics.com/java/com/revrobotics/package-summary.html
    - Software downloads & manuals: http://www.revrobotics.com/sparkmax-software/

### Radio
1. You can access radio web page by logging into http://10.21.68.1 root/admin
2. RoboRIO should always be plugged into the port on the radio labeled "18-24 vPOE" only! Or connect to this port via a switch

### RoboRIO
1. You can access roboRIO diagnostics webpage by http://roboRIO-2168-FRC.local (using IE web browser) or http://10.21.68.2
2. You can program roboRIO over ethernet, USB, or WiFi (if USB, NI Update Suite needs to be installed to get USB drivers)
3. Files will be logged to /home/lvuser/Logs
4. You can ftp files to/from the roboRIO using filezilla, winscp, web browser, or your local file explorer at ftp://10.21.68.2:21
5. You can ssh into roboRIO using putty or console application at ssh 10.21.68.2:22 username:lvuser password: blank

### Dashboard (on driverstation)
1. Java dashboard will open if Java is selected from the driverstation menu
2. Python dash (if it installed) will open if "default" dashboard is selected from drivestation menu
3. If SmartDashboard doesn't update, but you have robot comms, in SmartDashboard preferences toggle "use mDNS" until it does


## Repository Guidelines
### Branches
Our repository and workflow loosely follows the gitflow workflow. This workflow is simple and is one of the most popular workflows when using git with a large number of developers. More info: https://www.atlassian.com/git/tutorials/comparing-workflows#gitflow-workflow
- The main branch contains code that is known-working, has been tested, and can be deployed to a competition-ready robot.
- The develop branch is our sandbox for integrating and testing new features and fixing problems. This isn't the latest and greatest code, but it may have problems and needs to be checked out on the robot before being pushed into main. 
- Everything else is lumped under feature/bugfix branches. When we need to add new capabilities, start by branching the latest code in the develop branch.  

### Checklist for committing/pushing code
- Commit often and create detailed log messages that describe what and why you're making a change. Be specific.
- Review the changes you make before pushing them. You should look through all the files being added/modified/removed before you commit.
- Always verify your code compiles before pushing to the repo. There shouldn't be any red-underlined text in your commits.
- Push your changes into a branch with a name that identifies what feature it is adding or problem it is addressing in the code.
- Never push to the main branch. 
- After pushing your changes to the repo, verify you can see your changes in GitHub from a web browser.

### Style and Architecture Guidelines
When creating member variables:
  - Set the accessibility to `private` and use a `_` before the name.
    ```java
    private <MotorController> _motor
    private DoubleSolenoid _piston
    private <Subsystem> _instance
    ```
  - Constants for subsystems should be added to their respective subsystems.
    ```java
    private final boolean IS_MOTOR_REVERSED
    private final double LIFT_HOLDING_VOLTAGE
    public final PIDPosition PID_POSITION_1
    ```
  - Constants for subsystems that can conflict with each other such as CAN IDs should be added to `Constants.java`.
    ```java
    public static final int <Subsystem1>_MOTOR_PDP = 0
    public static final int <Subsystem2>_MOTOR_PDP = 1
    ```

When creating a subsystem:
  - Every `Subsystem` should use singleton design pattern (private constructor, public getInstance() method). (is this still true?)

When creating methods in the subsystem:
  - Motors:
    - Create a method to set the speed of the motor.
    - The method name should be `drive`.
    - Add a comment for the polarity of the motor (What direction does postive go to)
      - Positive for a shooter wheel should be out.
      - Positive for a lift should be up.
    - EX:
      ```java
      /**
       * Sets the speed of the XYZ motor.
       *
       * @param speed : a value of 1.0 to -1.0 (positive is into the robot, negative is out of the robot)
       */
      public void drive(double speed) {
        Whatever this method does...
      }
      ```
  - Pneumatics
    - The class commonly used is DoubleSolenoid.
    - Create a method to extend the pneumatic.
      - The method name should be `extend`.
      - Unless there are multiple pneumatics, then extend the specific pneumatic.
    - Create a method to retract the pneumatic.
      - The method name should be `retract`.
      - Unless there are multiple pneumatics, then extend the specific pneumatic.
    - EX:
      ```java
      /**
       * Extends the XYZ pneumatic.
       *
       */
      public void extend() {
        Whatever this method does...
      }

      /**
       * Retract the XYZ pneumatic.
       *
       */
      public void retract() {
        Whatever this method does...
      }
      ```
When adding an instance of a subsystem to the `Robot` class:
  - The variable should be of `private` access type.
  - Any access to the subsystem elsewhere in the code should use the static getInstance method for the respective subsystem. (e.g. `<SubsystemName>.getInstance()`).

When creating commands:
  - Motors:
     - Create a command to drive the motor with a constant.
     - Create a command to drive the motor with a joystick.
  - Pneumatics:
     - Create a command to extend the pneumatic.
     - Create a command to retract the pneumatic.


## Robot Design (To be continued)
- All students are assigned one or many subsystems on the robot. Your task is the following:
1. Pull main and create a new branch (naming it after the subsystem you're working on)
2. Write the subsystem with all the hardware called out below
3. RobotMap ports may be pre-determined and assigned to your subsystem, but you need to add the code to Constants.java(?)
4. Write all the commands for your subsystem, and place them in the approporate command subfolders/packages for your subsystem
5. Add the subsystem to Robot.java
6. Push your code to your branch for review by your lead and mentors (they will write issues for you to fix)
7. Once tested on a robot, it will be merged into main
