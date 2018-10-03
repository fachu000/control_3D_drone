#include "Common.h"
#include "QuadControl.h"

#include "Utility/SimpleConfig.h"

#include "Utility/StringUtils.h"
#include "Trajectory.h"
#include "BaseController.h"
#include "Math/Mat3x3F.h"

#ifdef __PX4_NUTTX
#include <systemlib/param/param.h>
#endif

void QuadControl::Init()
{
  BaseController::Init();

  // variables needed for integral control
  integratedAltitudeError = 0;
    
#ifndef __PX4_NUTTX
  // Load params from simulator parameter system
  ParamsHandle config = SimpleConfig::GetInstance();
   
  // Load parameters (default to 0)
  kpPosXY = config->Get(_config+".kpPosXY", 0);
  kpPosZ = config->Get(_config + ".kpPosZ", 0);
  KiPosZ = config->Get(_config + ".KiPosZ", 0);
     
  kpVelXY = config->Get(_config + ".kpVelXY", 0);
  kpVelZ = config->Get(_config + ".kpVelZ", 0);

  kpBank = config->Get(_config + ".kpBank", 0);
  kpYaw = config->Get(_config + ".kpYaw", 0);

  kpPQR = config->Get(_config + ".kpPQR", V3F());

  maxDescentRate = config->Get(_config + ".maxDescentRate", 100);
  maxAscentRate = config->Get(_config + ".maxAscentRate", 100);
  maxSpeedXY = config->Get(_config + ".maxSpeedXY", 100);
  maxAccelXY = config->Get(_config + ".maxHorizAccel", 100);

  maxTiltAngle = config->Get(_config + ".maxTiltAngle", 100);

  minMotorThrust = config->Get(_config + ".minMotorThrust", 0);
  maxMotorThrust = config->Get(_config + ".maxMotorThrust", 100);
#else
  // load params from PX4 parameter system
  //TODO
  param_get(param_find("MC_PITCH_P"), &Kp_bank);
  param_get(param_find("MC_YAW_P"), &Kp_yaw);
#endif
}

VehicleCommand QuadControl::GenerateMotorCommands(float collThrustCmd, V3F momentCmd)
{
  // Convert a desired 3-axis moment and collective thrust command to 
  //   individual motor thrust commands
  // INPUTS: 
  //   collThrustCmd: desired collective thrust [N]
  //   momentCmd: desired rotation moment about each axis [N m]
  // OUTPUT:
  //   set class member variable cmd (class variable for graphing) where
  //   cmd.desiredThrustsN[0..3]: motor commands, in [N]

  // HINTS: 
  // - you can access parts of momentCmd via e.g. momentCmd.x
  // You'll need the arm length parameter L, and the drag/thrust ratio kappa

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
    ParamsHandle config = SimpleConfig::GetInstance();
    L = config->Get(_config + ".L", 0);
    kappa = config->Get(_config + ".kappa", 0);
    
    //float v[9] = {1.,2.,3.,4.,5.,6.,7.,8.,9.};
    //float v[9] = {-1,-1,0,0,-1,-1,-1,0,-1};
    //Mat3x3F a = Mat3x3F(v);
    //Mat3x3F M = Mat3x3F(v);
    
    float c = collThrustCmd;
    float b1 = momentCmd.x/L;
    float b2 = momentCmd.y/L;
    //float b3 = momentCmd.z/kappa;
    float b3 = -momentCmd.z/kappa;
    
    float F4 = -1/4.*(b3-b1+b2-c);
    float F3 = -1/2.*(2*F4+b2-c);
    float F2 = -1/2.*(2*F3 + b1 -c);
    float F1 = c - F2 - F3 - F4;

//if ((F1 < minMotorThrust)||(F1>maxMotorThrust)) printf("F1 exceeds limits: F1 = %.2f\n",F1);
    
    //printf("moment command %.4f, %.4f, %.4f\n",momentCmd.x,momentCmd.y,momentCmd.z);
    
    //////// THE FOLLOWING ORDERING CRITERIA DIFFERS FROM THE EXERCISE - DEBUG FOLLOWING
    
    //printf("F = %.3f %.3f %.3f %.3f\n",F1,F2,F3,F4);
    cmd.desiredThrustsN[0] = F1; //mass * 9.81f / 4.f; // front left
    cmd.desiredThrustsN[1] = F2; //mass * 9.81f / 4.f; // front right
    cmd.desiredThrustsN[2] = F4; //mass * 9.81f / 4.f; // rear left
    cmd.desiredThrustsN[3] = F3; //mass * 9.81f / 4.f; // rear right

    //cmd.desiredThrustsN[0] = CONSTRAIN(F1,minMotorThrust,maxMotorThrust); //mass * 9.81f / 4.f; // front left
    //cmd.desiredThrustsN[1] = CONSTRAIN(F2,minMotorThrust,maxMotorThrust); //mass * 9.81f / 4.f; // front right
    //cmd.desiredThrustsN[2] = CONSTRAIN(F4,minMotorThrust,maxMotorThrust); //mass * 9.81f / 4.f; // rear left
    //cmd.desiredThrustsN[3] = CONSTRAIN(F3,minMotorThrust,maxMotorThrust); //mass * 9.81f / 4.f; // rear right

    //cmd.desiredThrustsN[0] = 2*mass * 9.81f / 4.f; // front left
    //cmd.desiredThrustsN[1] = mass * 9.81f / 4.f; // front right
    //cmd.desiredThrustsN[2] = mass * 9.81f / 4.f; // rear left
    //cmd.desiredThrustsN[3] = 2*mass * 9.81f / 4.f; // rear right
  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return cmd;
}

V3F QuadControl::BodyRateControl(V3F pqrCmd, V3F pqr)
{
  // Calculate a desired 3-axis moment given a desired and current body rate
  // INPUTS: 
  //   pqrCmd: desired body rates [rad/s]
  //   pqr: current or estimated body rates [rad/s]
  // OUTPUT:
  //   return a V3F containing the desired moments for each of the 3 axes

  // HINTS: 
  //  - you can use V3Fs just like scalars: V3F a(1,1,1), b(2,3,4), c; c=a-b;
  //  - you'll need parameters for moments of inertia Ixx, Iyy, Izz
  //  - you'll also need the gain parameter kpPQR (it's a V3F)

  V3F momentCmd;

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
    ParamsHandle config = SimpleConfig::GetInstance();
    Ixx = config->Get(_config + ".Ixx", 0);
    Iyy = config->Get(_config + ".Iyy", 0);
    Izz = config->Get(_config + ".Izz", 0);
    
    // debug
    //pqrCmd.x = 0; pqrCmd.y =-0.1;pqrCmd.z =0;
    //
    
    V3F omega_dot = kpPQR*( pqrCmd - pqr );
    
    V3F I(Ixx,Iyy,Izz);
    momentCmd = I*omega_dot;
    //printf("u_x = %.3f, u_y = %.3f, u_z = %.3f\n",momentCmd.x,momentCmd.y,momentCmd.z);
    //printf("q = %.2f, q_cmd = %.2f, kp_q = %.2f\n",pqr.y,pqrCmd.y,kpPQR.y);
    //printf("p = %.2f, p_cmd = %.2f, kp_p = %.2f\n",pqr.x,pqrCmd.x,kpPQR.x);
    //printf("prod = %.3f\n",Ixx*kpPQR.x*(pqrCmd.x - pqr.x));
  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return momentCmd;
}

// returns a desired roll and pitch rate 
V3F QuadControl::RollPitchControl(V3F accelCmd, Quaternion<float> attitude, float collThrustCmd)
{
  // Calculate a desired pitch and roll angle rates based on a desired global
  //   lateral acceleration, the current attitude of the quad, and desired
  //   collective thrust command
  // INPUTS: 
  //   accelCmd: desired acceleration in global XY coordinates [m/s2]
  //   attitude: current or estimated attitude of the vehicle
  //   collThrustCmd: desired collective thrust of the quad [N]
  // OUTPUT:
  //   return a V3F containing the desired pitch and roll rates. The Z
  //     element of the V3F should be left at its default value (0)

  // HINTS: 
  //  - we already provide rotation matrix R: to get element R[1,2] (python) use R(1,2) (C++)
  //  - you'll need the roll/pitch gain kpBank
  //  - collThrustCmd is a force in Newtons! You'll likely want to convert it to acceleration first

  V3F pqrCmd;
  Mat3x3F R = attitude.RotationMatrix_IwrtB();

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
    
    // debug
//accelCmd.x = 0; accelCmd.y = -maxAccelXY; //accelCmd.z = 0;
   // accelCmd.x = 0; accelCmd.y = 2; //accelCmd.z = 0;
    
    //
    
    
    ParamsHandle config = SimpleConfig::GetInstance();
    float Mass = config->Get(_config + ".Mass", 0);
    float c_thrust = collThrustCmd/Mass;

    // debug: sign of the following two lines changed
    float b_x_c = -accelCmd.x/c_thrust;
    float b_y_c = -accelCmd.y/c_thrust;
    float b_x_dot_c = kpBank * (b_x_c - R(0,2));
    float b_y_dot_c = kpBank * (b_y_c - R(1,2));
    
    pqrCmd.x = 1/R(2,2)*(R(1,0)*b_x_dot_c - R(0,0)*b_y_dot_c);
    pqrCmd.y = 1/R(2,2)*(R(1,1)*b_x_dot_c - R(0,1)*b_y_dot_c);
    pqrCmd.z = 0;
    
   //printf("                               rp_cmd = %.2f, q_cmd = %.2f\n",pqrCmd.x,pqrCmd.y );
    
  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return pqrCmd;
}

float QuadControl::AltitudeControl(float posZCmd, float velZCmd, float posZ, float velZ, Quaternion<float> attitude, float accelZCmd, float dt)
{
  // Calculate desired quad thrust based on altitude setpoint, actual altitude,
  //   vertical velocity setpoint, actual vertical velocity, and a vertical 
  //   acceleration feed-forward command
  // INPUTS: 
  //   posZCmd, velZCmd: desired vertical position and velocity in NED [m]
  //   posZ, velZ: current vertical position and velocity in NED [m]
  //   accelZCmd: feed-forward vertical acceleration in NED [m/s2]
  //   dt: the time step of the measurements [seconds]
  // OUTPUT:
  //   return a collective thrust command in [N]

  // HINTS: 
  //  - we already provide rotation matrix R: to get element R[1,2] (python) use R(1,2) (C++)
  //  - you'll need the gain parameters kpPosZ and kpVelZ
  //  - maxAscentRate and maxDescentRate are maximum vertical speeds. Note they're both >=0!
  //  - make sure to return a force, not an acceleration
  //  - remember that for an upright quad in NED, thrust should be HIGHER if the desired Z acceleration is LOWER

  Mat3x3F R = attitude.RotationMatrix_IwrtB();
  float thrust = 0;

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
    integratedAltitudeError += (posZCmd - posZ)*dt;
    
    float u1_bar = kpPosZ*(posZCmd - posZ) + kpVelZ*(velZCmd - velZ) + accelZCmd + KiPosZ*integratedAltitudeError;  // commanded Z acceleration
    float c_thrust = (u1_bar - CONST_GRAVITY)/R(2,2);
    
    thrust = - mass * c_thrust;
    //thrust = - mass * CONST_GRAVITY;
    //printf("thrust = %.3f\n",posZ);

  /////////////////////////////// END STUDENT CODE ////////////////////////////
  
  return thrust;
}

// returns a desired acceleration in global frame
V3F QuadControl::LateralPositionControl(V3F posCmd, V3F velCmd, V3F pos, V3F vel, V3F accelCmdFF)
{
  // Calculate a desired horizontal acceleration based on 
  //  desired lateral position/velocity/acceleration and current pose
  // INPUTS: 
  //   posCmd: desired position, in NED [m]
  //   velCmd: desired velocity, in NED [m/s]
  //   pos: current position, NED [m]
  //   vel: current velocity, NED [m/s]
  //   accelCmdFF: feed-forward acceleration, NED [m/s2]
  // OUTPUT:
  //   return a V3F with desired horizontal accelerations. 
  //     the Z component should be 0
  // HINTS: 
  //  - use the gain parameters kpPosXY and kpVelXY
  //  - make sure you limit the maximum horizontal velocity and acceleration
  //    to maxSpeedXY and maxAccelXY

  // make sure we don't have any incoming z-component
  accelCmdFF.z = 0;
  velCmd.z = 0;
  posCmd.z = pos.z;

  // we initialize the returned desired acceleration to the feed-forward value.
  // Make sure to _add_, not simply replace, the result of your controller
  // to this variable
  V3F accelCmd = accelCmdFF;

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
    
    // debug
    //posCmd.x = 0; posCmd.y = 0; posCmd.z = -1;
    //velCmd.x = 0; velCmd.y = 0; velCmd.z = 0;
    //
    
    
    //velCmd.x = CONSTRAIN( velCmd.x , - maxSpeedXY , maxSpeedXY );
    //velCmd.y = CONSTRAIN( velCmd.y , - maxSpeedXY , maxSpeedXY );
    //accelCmdFF.x = CONSTRAIN( accelCmdFF.x , - maxAccelXY , maxAccelXY );
    //accelCmdFF.y = CONSTRAIN( accelCmdFF.y , - maxAccelXY , maxAccelXY );
    
    float vel_mag = sqrt(pow(velCmd.x,2) + pow(velCmd.y,2) );
    if (vel_mag >= maxSpeedXY)
    {
        velCmd.x = velCmd.x*maxSpeedXY/vel_mag;
        velCmd.y = velCmd.y*maxSpeedXY/vel_mag;
    }
    
    
    accelCmd = accelCmd + kpPosXY*(posCmd - pos) + kpVelXY*(velCmd -vel);
    
    //printf("accel x = %.2f, accel y = %.2f\n",accelCmd.x,accelCmd.y);
    
    float acc_mag = sqrt(pow(accelCmd.x,2) + pow(accelCmd.y,2) );
    if (acc_mag >= maxAccelXY)
    {
        accelCmd.x = accelCmd.x*maxAccelXY/acc_mag;
        accelCmd.y = accelCmd.y*maxAccelXY/acc_mag;
    }
    //accelCmd.x = CONSTRAIN( accelCmd.x , - maxAccelXY , maxAccelXY );
    //accelCmd.y = CONSTRAIN( accelCmd.y , - maxAccelXY , maxAccelXY );
    
    accelCmd.z = 0;

    
    //printf("clipped accel x = %.2f, accel y = %.2f\n",accelCmd.x,accelCmd.y);
    //printf("posCmd = %.2f, pos = %.2f, velCmd = %.2f, vel = %.2f\n",posCmd.x,pos.x,velCmd.x,vel.x);
    
    //printf("posCmd = %.2f, pos = %.2f, velCmd = %.2f, vel = %f\n",posCmd.x,pos.x,velCmd.x,vel.x);
  // printf("clipped accel cmd x = %.2f, accel y = %.2f\n",accelCmd.x,accelCmd.y);
   
    
    
    // debug
    //accelCmd = -1*accelCmd;
    //
    
  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return accelCmd;
}

// returns desired yaw rate
float QuadControl::YawControl(float yawCmd, float yaw)
{
  // Calculate a desired yaw rate to control yaw to yawCmd
  // INPUTS: 
  //   yawCmd: commanded yaw [rad]
  //   yaw: current yaw [rad]
  // OUTPUT:
  //   return a desired yaw rate [rad/s]
  // HINTS: 
  //  - use fmodf(foo,b) to unwrap a radian angle measure float foo to range [0,b]. 
  //  - use the yaw control gain parameter kpYaw

  float yawRateCmd=0;
    float PI = 3.141592;
  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////

    yawRateCmd = kpYaw*(   fmodf(yawCmd-yaw + PI,2*PI)-PI  );  // diff betw. -Pi and Pi
  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return yawRateCmd;

}

VehicleCommand QuadControl::RunControl(float dt, float simTime)
{
  curTrajPoint = GetNextTrajectoryPoint(simTime);

  float collThrustCmd = AltitudeControl(curTrajPoint.position.z, curTrajPoint.velocity.z, estPos.z, estVel.z, estAtt, curTrajPoint.accel.z, dt);

  // reserve some thrust margin for angle control
  float thrustMargin = .1f*(maxMotorThrust - minMotorThrust);
  collThrustCmd = CONSTRAIN(collThrustCmd, (minMotorThrust+ thrustMargin)*4.f, (maxMotorThrust-thrustMargin)*4.f);
  
  V3F desAcc = LateralPositionControl(curTrajPoint.position, curTrajPoint.velocity, estPos, estVel, curTrajPoint.accel);
  
  V3F desOmega = RollPitchControl(desAcc, estAtt, collThrustCmd);
  desOmega.z = YawControl(curTrajPoint.attitude.Yaw(), estAtt.Yaw());

  V3F desMoment = BodyRateControl(desOmega, estOmega);

  return GenerateMotorCommands(collThrustCmd, desMoment);
}
