///////////////////////////////////////////////////////////////////////////////
//
// Copyright (C) OMG Plc 2009.
// All rights reserved.  This software is protected by copyright
// law and international treaties.  No part of this software / document
// may be reproduced or distributed in any form or by any means,
// whether transiently or incidentally to some other use of this software,
// without the written permission of the copyright owner.
// 
// Modified by Wei Ding
// 
// 
//
///////////////////////////////////////////////////////////////////////////////

#include <lcm/lcm-cpp.hpp>
#include <time.h>
#include "../lcmtypes/vicon_state_t.hpp"
#include "../include/util.h"
#include "../include/filter_util.h"
#include <stdlib.h>
#include <sys/time.h>
#include "Client.h"

#include <iostream>
#include <fstream>
#include <cassert>
#include <ctime>
#include <vector>
#include <string.h>
#include <random>

#define NUM_SAMPLES_MED 5
#define NUM_SAMPLES_AVG 5

#define JUMP_THRESHOLD 0.5//maximum jump between velocities
#define POS_JUMP_THRES 150
#define JUMP_THRESHOLD_ATTITUDE 4 //maximum jump between attitudes in degree for every step

#ifdef WIN32
  #include <conio.h>   // For _kbhit()
  #include <cstdio>   // For getchar()
  #include <windows.h> // For Sleep()
#else
  #include <unistd.h> // For sleep()
#endif // WIN32

#include <time.h>

using namespace ViconDataStreamSDK::CPP;

#define output_stream if(!LogFile.empty()) ; else std::cout 

namespace
{
  std::string Adapt( const bool i_Value )
  {
    return i_Value ? "True" : "False";
  }

  std::string Adapt( const Direction::Enum i_Direction )
  {
    switch( i_Direction )
    {
      case Direction::Forward:
        return "Forward";
      case Direction::Backward:
        return "Backward";
      case Direction::Left:
        return "Left";
      case Direction::Right:
        return "Right";
      case Direction::Up:
        return "Up";
      case Direction::Down:
        return "Down";
      default:
        return "Unknown";
    }
  }

  std::string Adapt( const DeviceType::Enum i_DeviceType )
  {
    switch( i_DeviceType )
    {
      case DeviceType::ForcePlate:
        return "ForcePlate";
      case DeviceType::Unknown:
      default:
        return "Unknown";
    }
  }

  
#ifdef WIN32
  bool Hit()
  {
    bool hit = false;
    while( _kbhit() )
    {
      getchar();
      hit = true;
    }
    return hit;
  }
#endif
}


std::string OBJ_TO_TRACK = "hbird_white";
int64_t time_tmp = 0;
double dt;

double pre_x = 0.0, pre_y = 0.0, pre_z = 0.0; //store previous velocity for velcities
double vx = 0.0, vy = 0.0, vz = 0.0; //velocities
double prev_vx = 0.0, prev_vy = 0.0, prev_vz = 0.0; //in order to filter out unreasonable jump;

double dx_in_mm, dy_in_mm, dz_in_mm;

//position filters
double pos_x[NUM_SAMPLES_MED], pos_y[NUM_SAMPLES_MED], pos_z[NUM_SAMPLES_MED];
double pos_x_med[NUM_SAMPLES_AVG], pos_y_med[NUM_SAMPLES_AVG], pos_z_med[NUM_SAMPLES_AVG];
double pos_x_avg[NUM_SAMPLES_AVG], pos_y_avg[NUM_SAMPLES_AVG], pos_z_avg[NUM_SAMPLES_AVG];

//velocity filters

double vel_x[NUM_SAMPLES_MED], vel_y[NUM_SAMPLES_MED], vel_z[NUM_SAMPLES_MED];
double vel_x_med[NUM_SAMPLES_AVG], vel_y_med[NUM_SAMPLES_AVG], vel_z_med[NUM_SAMPLES_AVG];
double vel_x_avg[NUM_SAMPLES_AVG], vel_y_avg[NUM_SAMPLES_AVG], vel_z_avg[NUM_SAMPLES_AVG];

// store previous attitude to get the differentiation
double pre_rho = 0.0, pre_theta = 0.0, pre_psi = 0.0;
double v_rho = 0.0, v_theta = 0.0, v_psi = 0.0;

//store previous Euler rate to filter out unreasonable numbers
double v_rho_pre = 0.0, v_theta_pre = 0.0, v_psi_pre = 0.0;

//attitude filters
double att_x[NUM_SAMPLES_MED], att_y[NUM_SAMPLES_MED], att_z[NUM_SAMPLES_MED];
double att_x_med[NUM_SAMPLES_AVG], att_y_med[NUM_SAMPLES_AVG], att_z_med[NUM_SAMPLES_AVG];
double att_x_avg[NUM_SAMPLES_AVG], att_y_avg[NUM_SAMPLES_AVG], att_z_avg[NUM_SAMPLES_AVG];

//Euler rate filters
double rho_vel[NUM_SAMPLES_MED], theta_vel[NUM_SAMPLES_MED], psi_vel[NUM_SAMPLES_MED];
double rho_vel_med[NUM_SAMPLES_AVG], theta_vel_med[NUM_SAMPLES_AVG], psi_vel_med[NUM_SAMPLES_AVG];
double rho_vel_avg[NUM_SAMPLES_AVG], theta_vel_avg[NUM_SAMPLES_AVG], psi_vel_avg[NUM_SAMPLES_AVG];

  //noise generation
std::default_random_engine generator;
// covariance is 8e-6
std::normal_distribution<double> distribution(0.0,0.02);
double number;

int main( int argc, char* argv[] )
{

  //lcm 
  lcm::LCM lcm;
  if(!lcm.good())
    return 1;

  vicon_state_t vicon_msg;


  // Program options
  std::string HostName = "192.168.0.101:801";
  if( argc > 1 )
  {
    HostName = argv[1];
  }

  // log contains:
  // version number
  // log of framerate over time
  // --multicast
  // kill off internal app
  std::string LogFile = "";
  std::string MulticastAddress = "244.0.0.0:44801";
  bool ConnectToMultiCast = false;
  bool EnableMultiCast = false;
  bool EnableHapticTest = false;
  bool bReadCentroids = false;
  std::vector<std::string> HapticOnList(0);
  for(int a=2; a < argc; ++a)
  {
    std::string arg = argv[a];
    if(arg == "--help")
    {
      std::cout << argv[0] << " <HostName>: allowed options include:\n  --log_file <LogFile> --enable_multicast <MulticastAddress:Port> --connect_to_multicast <MulticastAddress:Port> --help --enable_haptic_test <DeviceName> --centroids" << std::endl;
      return 0;
    }
    else if (arg=="--log_file")
    {
      if(a < argc)
      {
        LogFile = argv[a+1];
        std::cout << "Using log file <"<< LogFile << "> ..." << std::endl;
        ++a;
      }
    }
    else if (arg=="--enable_multicast")
    {
      EnableMultiCast = true;
      if(a < argc)
      {
        MulticastAddress = argv[a+1];
        std::cout << "Enabling multicast address <"<< MulticastAddress << "> ..." << std::endl;
        ++a;
      }
    }
    else if (arg=="--connect_to_multicast")
    {
      ConnectToMultiCast = true;
      if(a < argc)
      {
        MulticastAddress = argv[a+1];
        std::cout << "connecting to multicast address <"<< MulticastAddress << "> ..." << std::endl;
        ++a;
      }
    }
    else if (arg=="--enable_haptic_test")
    {
      EnableHapticTest = true;
      ++a;
      if ( a < argc )
      {    
        //assuming no haptic device name starts with "--"
        while( a < argc && strncmp( argv[a], "--", 2 ) !=0  )
        {
          HapticOnList.push_back( argv[a] );
          ++a;
        }
      }
    }
    else if( arg=="--centroids" )
    {
      bReadCentroids = true;
    }
    else
    {
      std::cout << "Failed to understand argument <" << argv[a] << ">...exiting" << std::endl;
      return 1;
    }
  }

  std::ofstream ofs;
  if(!LogFile.empty())
  {
    ofs.open(LogFile.c_str());
    if(!ofs.is_open())
    {
      std::cout << "Could not open log file <" << LogFile << ">...exiting" << std::endl;
      return 1;
    }
  }
  // Make a new client
  Client MyClient;

  for(int i=0; i != 3; ++i) // repeat to check disconnecting doesn't wreck next connect
  {
    // Connect to a server
    std::cout << "Connecting to " << HostName << " ..." << std::flush;
    while( !MyClient.IsConnected().Connected )
    {
      // Direct connection

      bool ok = false;
      if(ConnectToMultiCast)
      {
        // Multicast connection
        ok = ( MyClient.ConnectToMulticast( HostName, MulticastAddress ).Result == Result::Success );

      }
      else
      {
        ok =( MyClient.Connect( HostName ).Result == Result::Success );
      }
      if(!ok)
      {
        std::cout << "Warning - connect failed..." << std::endl;
      }


      std::cout << ".";
  #ifdef WIN32
      Sleep( 1000 );
  #else
      sleep(1);
  #endif
    }
    // std::cout << std::endl;

    // Enable some different data types
    MyClient.EnableSegmentData();
    MyClient.EnableMarkerData();
    MyClient.EnableUnlabeledMarkerData();
    MyClient.EnableDeviceData();
    if( bReadCentroids )
    {
      MyClient.EnableCentroidData();
    }

    std::cout << "Segment Data Enabled: "          << Adapt( MyClient.IsSegmentDataEnabled().Enabled )         << std::endl;
    std::cout << "Marker Data Enabled: "           << Adapt( MyClient.IsMarkerDataEnabled().Enabled )          << std::endl;
    std::cout << "Unlabeled Marker Data Enabled: " << Adapt( MyClient.IsUnlabeledMarkerDataEnabled().Enabled ) << std::endl;
    std::cout << "Device Data Enabled: "           << Adapt( MyClient.IsDeviceDataEnabled().Enabled )          << std::endl;
    std::cout << "Centroid Data Enabled: "         << Adapt( MyClient.IsCentroidDataEnabled().Enabled )        << std::endl;

    // Set the streaming mode
    //MyClient.SetStreamMode( ViconDataStreamSDK::CPP::StreamMode::ClientPull );
    // MyClient.SetStreamMode( ViconDataStreamSDK::CPP::StreamMode::ClientPullPreFetch );
    MyClient.SetStreamMode( ViconDataStreamSDK::CPP::StreamMode::ServerPush );

    // Set the global up axis
    MyClient.SetAxisMapping( Direction::Forward, 
                             Direction::Left, 
                             Direction::Up ); // Z-up
    // MyClient.SetGlobalUpAxis( Direction::Forward, 
    //                           Direction::Up, 
    //                           Direction::Right ); // Y-up

    Output_GetAxisMapping _Output_GetAxisMapping = MyClient.GetAxisMapping();
    std::cout << "Axis Mapping: X-" << Adapt( _Output_GetAxisMapping.XAxis ) 
                           << " Y-" << Adapt( _Output_GetAxisMapping.YAxis ) 
                           << " Z-" << Adapt( _Output_GetAxisMapping.ZAxis ) << std::endl;

    // Discover the version number
    Output_GetVersion _Output_GetVersion = MyClient.GetVersion();
    std::cout << "Version: " << _Output_GetVersion.Major << "." 
                             << _Output_GetVersion.Minor << "." 
                             << _Output_GetVersion.Point << std::endl;

    if( EnableMultiCast )
    {
      assert( MyClient.IsConnected().Connected );
      MyClient.StartTransmittingMulticast( HostName, MulticastAddress );
    }

    size_t FrameRateWindow = 1000; // frames
    size_t Counter = 0;
    clock_t LastTime = clock();
    // Loop until a key is pressed
  #ifdef WIN32
    while( !Hit() )
  #else
    while( true)
  #endif
    {
      // Get a frame
      // output_stream << "Waiting for new frame...";
      while( MyClient.GetFrame().Result != Result::Success )
      {
        // Sleep a little so that we don't lumber the CPU with a busy poll
        #ifdef WIN32
          Sleep( 200 );
        #else
          sleep(1);
        #endif

        // output_stream << ".";
      }
      // output_stream << std::endl;
      if(++Counter == FrameRateWindow)
      {
        clock_t Now = clock();
        double FrameRate = (double)(FrameRateWindow * CLOCKS_PER_SEC) / (double)(Now - LastTime);
        if(!LogFile.empty())
        {
          time_t rawtime;
          struct tm * timeinfo;
          time ( &rawtime );
          timeinfo = localtime ( &rawtime );

          ofs << "Frame rate = " << FrameRate << " at " <<  asctime (timeinfo)<< std::endl;
        }

        LastTime = Now;
        Counter = 0;
      }

      // Get the frame number
      Output_GetFrameNumber _Output_GetFrameNumber = MyClient.GetFrameNumber();
      // output_stream << "Frame Number: " << _Output_GetFrameNumber.FrameNumber << std::endl;

      if( EnableHapticTest == true )
      {
        for (size_t i = 0; i < HapticOnList.size(); ++ i)
        {
          if( Counter % 2 == 0 )
          {
              Output_SetApexDeviceFeedback Output= MyClient.SetApexDeviceFeedback( HapticOnList[i],  true ); 
              if( Output.Result == Result::Success )
              {
                // output_stream<< "Turn haptic feedback on for device: " << HapticOnList[i]<<std::endl;
              }
              else if( Output.Result == Result::InvalidDeviceName )
              {
                output_stream<< "Device doesn't exist: "<< HapticOnList[i]<<std::endl;
              }
          }
          if( Counter % 20 == 0 )
          {
              Output_SetApexDeviceFeedback Output = MyClient.SetApexDeviceFeedback( HapticOnList[i],  false); 

              if( Output.Result == Result::Success )
              {
                output_stream<< "Turn haptic feedback off for device: " << HapticOnList[i]<<std::endl;
              }
          }
        }
      }


      // Count the number of subjects
      unsigned int SubjectCount = MyClient.GetSubjectCount().SubjectCount;
      // output_stream << "Subjects (" << SubjectCount << "):" << std::endl;
      for( unsigned int SubjectIndex = 0 ; SubjectIndex < SubjectCount ; ++SubjectIndex )
      {
        // output_stream << "  Subject #" << SubjectIndex << std::endl;

        // Get the subject name
        std::string SubjectName = MyClient.GetSubjectName( SubjectIndex ).SubjectName;
        // output_stream << "    Name: " << SubjectName << std::endl;

        // Get the root segment
        // std::string RootSegment = MyClient.GetSubjectRootSegmentName( SubjectName ).SegmentName;
        // output_stream << "    Root Segment: " << RootSegment << std::endl;

        // Count the number of segments
        unsigned int SegmentCount = MyClient.GetSegmentCount( SubjectName ).SegmentCount;
        // output_stream << "    Segments (" << SegmentCount << "):" << std::endl;
        for( unsigned int SegmentIndex = 0 ; SegmentIndex < SegmentCount ; ++SegmentIndex )
        {
          // output_stream << "      Segment #" << SegmentIndex << std::endl;

          
          // Get the segment name
          std::string SegmentName = MyClient.GetSegmentName( SubjectName, SegmentIndex ).SegmentName;

          //chose the the object by name
          
          if(SegmentName != OBJ_TO_TRACK) continue;
          
          // output_stream << "        Name: " << SegmentName << std::endl;

          // Get the segment parent
          std::string SegmentParentName = MyClient.GetSegmentParentName( SubjectName, SegmentName ).SegmentName;
          // output_stream << "        Parent: " << SegmentParentName << std::endl;

          // Get the segment's children
          unsigned int ChildCount = MyClient.GetSegmentChildCount( SubjectName, SegmentName ).SegmentCount;
          // output_stream << "     Children (" << ChildCount << "):" << std::endl;
          for( unsigned int ChildIndex = 0 ; ChildIndex < ChildCount ; ++ChildIndex )
          {
            std::string ChildName = MyClient.GetSegmentChildName( SubjectName, SegmentName, ChildIndex ).SegmentName;
            // output_stream << "       " << ChildName << std::endl;
          }

          // Get the global segment translation
          Output_GetSegmentGlobalTranslation _Output_GetSegmentGlobalTranslation = 
            MyClient.GetSegmentGlobalTranslation( SubjectName, SegmentName );
          // output_stream << "        Global Translation: (" << _Output_GetSegmentGlobalTranslation.Translation[ 0 ]  << ", " 
                                                       // << _Output_GetSegmentGlobalTranslation.Translation[ 1 ]  << ", " 
                                                       // << _Output_GetSegmentGlobalTranslation.Translation[ 2 ]  << ") " 
                                                       // << Adapt( _Output_GetSegmentGlobalTranslation.Occluded ) << std::endl;

          // Get the global segment rotation in helical co-ordinates
          // Output_GetSegmentGlobalRotationHelical _Output_GetSegmentGlobalRotationHelical = 
          //   MyClient.GetSegmentGlobalRotationHelical( SubjectName, SegmentName );
          // output_stream << "        Global Rotation Helical: (" << _Output_GetSegmentGlobalRotationHelical.Rotation[ 0 ]     << ", " 
                                                            // << _Output_GetSegmentGlobalRotationHelical.Rotation[ 1 ]     << ", " 
                                                            // << _Output_GetSegmentGlobalRotationHelical.Rotation[ 2 ]     << ") " 
                                                            // << Adapt( _Output_GetSegmentGlobalRotationHelical.Occluded ) << std::endl;

          // Get the global segment rotation as a matrix
          Output_GetSegmentGlobalRotationMatrix _Output_GetSegmentGlobalRotationMatrix = 
            MyClient.GetSegmentGlobalRotationMatrix( SubjectName, SegmentName );

            for (int iii = 0; iii < 9; iii ++) {
              vicon_msg.DCM[iii] = _Output_GetSegmentGlobalRotationMatrix.Rotation[ iii ];
            }
            
          // output_stream << "        Global Rotation Matrix: (" << _Output_GetSegmentGlobalRotationMatrix.Rotation[ 0 ]     << ", " 
                                                           // << _Output_GetSegmentGlobalRotationMatrix.Rotation[ 1 ]     << ", " 
                                                           // << _Output_GetSegmentGlobalRotationMatrix.Rotation[ 2 ]     << ", " 
                                                           // << _Output_GetSegmentGlobalRotationMatrix.Rotation[ 3 ]     << ", " 
                                                           // << _Output_GetSegmentGlobalRotationMatrix.Rotation[ 4 ]     << ", " 
                                                           // << _Output_GetSegmentGlobalRotationMatrix.Rotation[ 5 ]     << ", " 
                                                           // << _Output_GetSegmentGlobalRotationMatrix.Rotation[ 6 ]     << ", " 
                                                           // << _Output_GetSegmentGlobalRotationMatrix.Rotation[ 7 ]     << ", " 
                                                           // << _Output_GetSegmentGlobalRotationMatrix.Rotation[ 8 ]     << ") " 
                                                           // << Adapt( _Output_GetSegmentGlobalRotationMatrix.Occluded ) << std::endl;

          // Get the global segment rotation in quaternion co-ordinates
          Output_GetSegmentGlobalRotationQuaternion _Output_GetSegmentGlobalRotationQuaternion = 
            MyClient.GetSegmentGlobalRotationQuaternion( SubjectName, SegmentName );
          // output_stream << "        Global Rotation Quaternion: (" << _Output_GetSegmentGlobalRotationQuaternion.Rotation[ 0 ]     << ", " 
                                                               // << _Output_GetSegmentGlobalRotationQuaternion.Rotation[ 1 ]     << ", " 
                                                               // << _Output_GetSegmentGlobalRotationQuaternion.Rotation[ 2 ]     << ", " 
                                                               // << _Output_GetSegmentGlobalRotationQuaternion.Rotation[ 3 ]     << ") " 
                                                               // << Adapt( _Output_GetSegmentGlobalRotationQuaternion.Occluded ) << std::endl;

          // Get the global segment rotation in EulerXYZ co-ordinates
          Output_GetSegmentGlobalRotationEulerXYZ _Output_GetSegmentGlobalRotationEulerXYZ = 
            MyClient.GetSegmentGlobalRotationEulerXYZ( SubjectName, SegmentName );
            // std::cout<<"global: "<<_Output_GetSegmentGlobalRotationEulerXYZ.Rotation[ 0 ]<<", "<<_Output_GetSegmentGlobalRotationEulerXYZ.Rotation[ 1 ]<<", "<<_Output_GetSegmentGlobalRotationEulerXYZ.Rotation[ 2 ]<<std::endl;
          // output_stream << "        Global Rotation EulerXYZ: (" << _Output_GetSegmentGlobalRotationEulerXYZ.Rotation[ 0 ]     << ", " 
          //                                                    << _Output_GetSegmentGlobalRotationEulerXYZ.Rotation[ 1 ]     << ", " 
          //                                                    << _Output_GetSegmentGlobalRotationEulerXYZ.Rotation[ 2 ]     << ") " 
          //                                                    << Adapt( _Output_GetSegmentGlobalRotationEulerXYZ.Occluded ) << std::endl;


            number = distribution(generator);

            _Output_GetSegmentGlobalTranslation.Translation[ 0 ]+= 1000.0*number;
            _Output_GetSegmentGlobalTranslation.Translation[ 1 ]+= 1000.0*number;
            _Output_GetSegmentGlobalTranslation.Translation[ 2 ]+= 1000.0*number;
            //lcm publish data
            vicon_msg.timestamp = utime_now();
            
            ///////////////////filter position
            // std::cout <<abs(_Output_GetSegmentGlobalTranslation.Translation[ 2 ] - pre_z)<<std::endl;
            // if(abs(_Output_GetSegmentGlobalTranslation.Translation[ 0 ] - pre_x) <= POS_JUMP_THRES)
            addArr(pos_x, _Output_GetSegmentGlobalTranslation.Translation[ 0 ], NUM_SAMPLES_MED);
            // if(abs(_Output_GetSegmentGlobalTranslation.Translation[ 1 ] - pre_y) <= POS_JUMP_THRES) 
            addArr(pos_y, _Output_GetSegmentGlobalTranslation.Translation[ 1 ], NUM_SAMPLES_MED);
            // if(abs(_Output_GetSegmentGlobalTranslation.Translation[ 2 ] - pre_z) <= POS_JUMP_THRES) 
            addArr(pos_z, _Output_GetSegmentGlobalTranslation.Translation[ 2 ], NUM_SAMPLES_MED);

            addArr(pos_x_med, median(pos_x, NUM_SAMPLES_MED), NUM_SAMPLES_AVG);
            addArr(pos_y_med, median(pos_y, NUM_SAMPLES_MED), NUM_SAMPLES_AVG);
            addArr(pos_z_med, median(pos_z, NUM_SAMPLES_MED), NUM_SAMPLES_AVG);

            addArr(pos_x_avg, average(pos_x_med, NUM_SAMPLES_AVG), NUM_SAMPLES_AVG);
            addArr(pos_y_avg, average(pos_y_med, NUM_SAMPLES_AVG), NUM_SAMPLES_AVG);
            addArr(pos_z_avg, average(pos_z_med, NUM_SAMPLES_AVG), NUM_SAMPLES_AVG);


            vicon_msg.position[0] = pos_x_avg[NUM_SAMPLES_AVG - 1];
            vicon_msg.position[1] = pos_y_avg[NUM_SAMPLES_AVG - 1];
            vicon_msg.position[2] = pos_z_avg[NUM_SAMPLES_AVG - 1];

            /////////////////filter attitude
            ///
            ///it seems that attitude is refreshed with half of the frequency as position
            ///
            ///
            addArr(att_x, _Output_GetSegmentGlobalRotationEulerXYZ.Rotation[ 0 ], NUM_SAMPLES_MED);
            addArr(att_y, _Output_GetSegmentGlobalRotationEulerXYZ.Rotation[ 1 ], NUM_SAMPLES_MED);
            addArr(att_z, _Output_GetSegmentGlobalRotationEulerXYZ.Rotation[ 2 ], NUM_SAMPLES_MED);

            addArr(att_x_med, median(att_x, NUM_SAMPLES_MED), NUM_SAMPLES_AVG);
            addArr(att_y_med, median(att_y, NUM_SAMPLES_MED), NUM_SAMPLES_AVG);
            addArr(att_z_med, median(att_z, NUM_SAMPLES_MED), NUM_SAMPLES_AVG);

            addArr(att_x_avg, average(att_x_med, NUM_SAMPLES_AVG), NUM_SAMPLES_AVG);
            addArr(att_y_avg, average(att_y_med, NUM_SAMPLES_AVG), NUM_SAMPLES_AVG);
            addArr(att_z_avg, average(att_z_med, NUM_SAMPLES_AVG), NUM_SAMPLES_AVG);

            vicon_msg.attitude[0] = att_x_avg[NUM_SAMPLES_AVG - 1];
            vicon_msg.attitude[1] = att_y_avg[NUM_SAMPLES_AVG - 1];
            vicon_msg.attitude[2] = att_z_avg[NUM_SAMPLES_AVG - 1];
            // std::cout << "attitude[0]: "<<_Output_GetSegmentGlobalRotationEulerXYZ.Rotation[ 0 ]<<std::endl;
            // std::cout << "attitude[1]: "<<_Output_GetSegmentGlobalRotationEulerXYZ.Rotation[ 1 ]<<std::endl;
            // std::cout << "attitude[2]: "<<_Output_GetSegmentGlobalRotationEulerXYZ.Rotation[ 2 ]<<std::endl;

            dt = (static_cast<double>(vicon_msg.timestamp - time_tmp))/1000; //dt in mseconds

            // differentiate the positions to get velocities
            dx_in_mm = vicon_msg.position[0] - pre_x;
            dy_in_mm = vicon_msg.position[1] - pre_y;
            dz_in_mm = vicon_msg.position[2] - pre_z;

            // std::cout<<"dz: "<<dz_in_mm<<"\tdt: "<<dt <<std::endl;
 
            vx = dx_in_mm/dt;
            vy = dy_in_mm/dt;
            vz = dz_in_mm/dt;

            //differentiate the attitudes to get Euler rate
            v_rho = ((double)vicon_msg.attitude[0] - pre_rho)/(dt/1e3);
            v_theta = ((double)vicon_msg.attitude[1] - pre_theta)/(dt/1e3);
            v_psi = ((double)vicon_msg.attitude[2] - pre_psi)/(dt/1e3);

        ///////////////////////filter the velocity
        //collect data in to stack
        //
           
            //filter out unreasonable jump
            // if(abs(vx - prev_vx) <= JUMP_THRESHOLD) {

            addArr(vel_x, vx, NUM_SAMPLES_MED);
            addArr(vel_x_med, median(vel_x, NUM_SAMPLES_MED), NUM_SAMPLES_AVG);
            addArr(vel_x_avg, average(vel_x_med, NUM_SAMPLES_AVG), NUM_SAMPLES_AVG);

            vx = vel_x_avg[NUM_SAMPLES_AVG-1];

            prev_vx = vx;
              
            // } else vx = prev_vx;

            // if(abs(vy - prev_vy) <= JUMP_THRESHOLD) {
              
            addArr(vel_y, vy, NUM_SAMPLES_MED);
            addArr(vel_y_med, median(vel_y, NUM_SAMPLES_MED), NUM_SAMPLES_AVG);
            addArr(vel_y_avg, average(vel_y_med, NUM_SAMPLES_AVG), NUM_SAMPLES_AVG);

            vy = vel_y_avg[NUM_SAMPLES_AVG-1];

            prev_vy = vy;

            // } else vy = prev_vy;

            // if(abs(vz - prev_vz) <= JUMP_THRESHOLD) {

            addArr(vel_z, vz, NUM_SAMPLES_MED);
            addArr(vel_z_med, median(vel_z, NUM_SAMPLES_MED), NUM_SAMPLES_AVG);
            addArr(vel_z_avg, average(vel_z_med, NUM_SAMPLES_AVG), NUM_SAMPLES_AVG);

            vz = vel_z_avg[NUM_SAMPLES_AVG-1];

            prev_vz = vz;

            // } else vz = prev_vz;


            //filter the Euler rate
            // if(abs(v_rho - v_rho_pre) <= deg2rad(JUMP_THRESHOLD_ATTITUDE)) {

            if(abs(v_rho - v_rho_pre) <= 3) addArr(rho_vel, v_rho, NUM_SAMPLES_MED); // Jump between -180 to 180

            addArr(rho_vel_med, median(rho_vel, NUM_SAMPLES_MED), NUM_SAMPLES_AVG);
            addArr(rho_vel_avg, average(rho_vel_med, NUM_SAMPLES_AVG), NUM_SAMPLES_AVG);
            
            v_rho = rho_vel_avg[NUM_SAMPLES_AVG - 1];

            v_rho_pre = v_rho;
 
            // } else v_rho = v_rho_pre;

            // if(abs(v_theta - v_theta_pre) <= deg2rad(JUMP_THRESHOLD_ATTITUDE)) {

            if(abs(v_theta - v_theta_pre) <= 3) addArr(theta_vel, v_theta, NUM_SAMPLES_MED);

            addArr(theta_vel_med, median(theta_vel, NUM_SAMPLES_MED), NUM_SAMPLES_AVG);
            addArr(theta_vel_avg, average(theta_vel_med, NUM_SAMPLES_AVG), NUM_SAMPLES_AVG);

            v_theta = theta_vel_avg[NUM_SAMPLES_AVG - 1];

            v_theta_pre = v_theta;
              
            // } else v_theta = v_theta_pre;

            // if(abs(v_psi - v_psi_pre) <= deg2rad(JUMP_THRESHOLD_ATTITUDE)) {

            if(abs(v_psi - v_psi_pre) <= 3) addArr(psi_vel, v_psi, NUM_SAMPLES_MED);
            
            addArr(psi_vel_med, median(psi_vel, NUM_SAMPLES_MED), NUM_SAMPLES_AVG);
            addArr(psi_vel_avg, average(psi_vel_med, NUM_SAMPLES_AVG), NUM_SAMPLES_AVG);
            v_psi = psi_vel_avg[NUM_SAMPLES_AVG - 1];

            v_psi_pre = v_psi;

            // } else v_psi = v_psi_pre;

            vicon_msg.velocity[0] = vx;
            vicon_msg.velocity[1] = vy;
            vicon_msg.velocity[2] = vz;

            vicon_msg.angular_vel[0] = v_rho;
            vicon_msg.angular_vel[1] = v_theta;
            vicon_msg.angular_vel[2] = v_psi;

            lcm.publish("vicon_state", &vicon_msg);

            //previous state
            pre_x = (double)vicon_msg.position[0];
            pre_y = (double)vicon_msg.position[1];
            pre_z = (double)vicon_msg.position[2];

            pre_rho = (double)vicon_msg.attitude[0];
            pre_theta = (double)vicon_msg.attitude[1];
            pre_psi = (double)vicon_msg.attitude[2];

            time_tmp = vicon_msg.timestamp;

            //usleep is crucial for accuracy
            usleep(1e4);
        }

      }


      if( bReadCentroids )
      {
        unsigned int CameraCount = MyClient.GetCameraCount().CameraCount;
        output_stream << "Cameras(" << CameraCount << "):" << std::endl;

        for( unsigned int CameraIndex = 0; CameraIndex < CameraCount; ++CameraIndex )
        {
          output_stream << "  Camera #" << CameraIndex << ":" << std::endl;
        
          const std::string CameraName = MyClient.GetCameraName( CameraIndex ).CameraName;
          output_stream << "    Name: " << CameraName << std::endl;

          unsigned int CentroidCount = MyClient.GetCentroidCount( CameraName ).CentroidCount;
          output_stream << "    Centroids(" << CentroidCount << "):" << std::endl;

          for( unsigned int CentroidIndex = 0; CentroidIndex < CentroidCount; ++CentroidIndex )
          {
            output_stream << "      Centroid #" << CentroidIndex << ":" << std::endl;

            Output_GetCentroidPosition _Output_GetCentroidPosition = MyClient.GetCentroidPosition( CameraName, CentroidIndex );
            output_stream << "        Position: (" << _Output_GetCentroidPosition.CentroidPosition[0] << ", "
                                                   << _Output_GetCentroidPosition.CentroidPosition[1] << ")" << std::endl;
            output_stream << "        Radius: ("    << _Output_GetCentroidPosition.Radius   << ")" << std::endl;
            
          }
        }
      }
    }

    if( EnableMultiCast )
    {
      MyClient.StopTransmittingMulticast();
    }
    MyClient.DisableSegmentData();
    MyClient.DisableMarkerData();
    MyClient.DisableUnlabeledMarkerData();
    MyClient.DisableDeviceData();
    if( bReadCentroids )
    {
      MyClient.DisableCentroidData();
    }

    // Disconnect and dispose
    int t = clock();
    std::cout << " Disconnecting..." << std::endl;
    MyClient.Disconnect();
    int dt = clock() - t;
    double secs = (double) (dt)/(double)CLOCKS_PER_SEC;
    std::cout << " Disconnect time = " << secs << " secs" << std::endl;

  }
}
