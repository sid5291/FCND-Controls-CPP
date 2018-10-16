# Project 3: Building a Controller Write Up #
###### Implemented by Siddhartha Kumar ####


####1. Body Rate Controller:
   - *Input: Angular Rate commands and current estimate in Bodyframe*
   - *Output: Moment Commands in Body Frame*
   - The Body Controller is a First order P controller.
   - The `KpPQR` proportional Gain is used to calculate `ubarpqr` i.e, the Angular acceleration command
   - Moment Commands = Angular Acceleration * Moments of Inertia

####2. Roll Pitch Controller:
   - *Input: Lateral Acceleration Commands, Body Attitude and Collective Thrust*
   - *Output: Angular Rate Commands in the body frame*
   - The angular components of the collective thrust in the inertial frame for the x and y axis are calculated
     * Note: The provided positive thrust is in the upward direction and needs to be corrected for the NED frame
     * Constraints for the Maximum commanded lateral tilt are applied to these commands   
   - The Roll pitch controller is another First order P controller
   - The `KpBank` Proportional gain is used to calculate the rate of change of the x and y angular components
   - These are then rotated into the body frame to calculate the angular rates in the body frame i.e,`pqrcmd`
   
####3. Altitude Controller:
   - *Input: Desired Altitude, vertical velocity, current z position and attitude*
   - *Output: Collective Thrust*
   - The Altitude Controller is implemented as a Cascaded Proportional and  Proportional Integral Controller
   - The Z Velocity command:
        * Is calculated using a P Controller with  `KpPos` as the Proportional Gain
        * Is constrained to a Maximum Ascent and Descent Rates, and the Integral component is Zeroed out when the command is beyond these limits to prevent Integrator WindUp
   - The Z Acceleration command:
        * Is calculated by using a PI controller with `KpVel` as the proportional Gain and `KiPosZ` as the integrator Gain
        * Is Upwards in the Inertial frame, hence it is subtracted from the gravitational acceleration and then rotated to provide Collective Thrust in the Body Frame
   
####4. Lateral Position Controller:
   - *Input: Desired Lateral Position, velocity, current xy position*
   - *Output: Lateral Accelerations*
   - A Cascaded Double Proportional Controller is used
   - `KpPosXY` and `KpVelXY` are the proportional gains to calculate Commanded Velocity and Acceleration
   - Commanded Velocity and Acceleration are constrained by the provided Maximum XY Velocity and Acceleration
   
####5. Yaw Controller:
   - *Input: Desired Yaw and current Yaw*
   - *Output: YawRate Command*
   - A simple First Order P controller is used to calculate Commanded Yaw rate using the `KpYaw` proportional Gain

####6. Generate Motor Commands:
   - *Input: Commanded Thrust and Moments*
   - *Output: Thrust Command per Fan*
   - Factors To consider for this function:
     * The NED coordinate system is being used therefore using the right hand rule clockwise rotors provide a positive moment where as counter clockwise rotors provide a negative moment
     * A 'X' orientation for the Quad rotor is assumed:
       - Motor 1 Upper Left, CCW 
       - Motor 2 Upper Right, CW
       - Motor 3 Lower Left,  CW
       - Motor 4 Lower Right, CCW
     * Positive X Axis is the line from the center and bisects the line between Motor 1 and 2
     * Positive Y Axis is the line from the center and bisects the line between Motor 2 and 4
   - Using the above Factors we get:
     * Collective Thrust(Tc) = `f1 + f2 + f3 + f4`
     * X Moment (Mx) = `L/sqrt(2) * (f1 - f2 + f3 - f4)`
     * Y Moment (My) = `L/sqrt(2) * (f1 - f2 + f3 - f4)`
     * Z Moment (Mz) = `Kf/Km * (-f1 + f2 + f3 - f4)`
       - By Definition `kappa = Km/Kf`
   - From this we can solve to get f1, f2 ,f3 and f4 in terms of Tc, Mx, My and Mz   