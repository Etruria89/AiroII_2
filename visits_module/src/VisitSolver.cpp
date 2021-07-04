    /*
     <one line to give the program's name and a brief idea of what it does.>
     Copyright (C) 2015  <copyright holder> <email>
     
     This program is free software: you can redistribute it and/or modify
     it under the terms of the GNU General Public License as published by
     the Free Software Foundation, either version 3 of the License, or
     (at your option) any later version.
     
     This program is distributed in the hope that it will be useful,
     but WITHOUT ANY WARRANTY; without even the implied warranty of
     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
     GNU General Public License for more details.
     
     You should have received a copy of the GNU General Public License
     along with this program.  If not, see <http://www.gnu.org/licenses/>.
     */


#include "VisitSolver.h"
#include "ExternalSolver.h"
#include <map>
#include <string>
#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <math.h>

#include "armadillo"
#include <initializer_list>

#include <boost/algorithm/string.hpp>

using namespace std;
using namespace arma;

    //map <string, vector<double> > region_mapping;

extern "C" ExternalSolver* create_object()
{
  return new VisitSolver();
}

extern "C" void destroy_object(ExternalSolver *externalSolver)
{
  delete externalSolver;
}

VisitSolver::VisitSolver(){
}

VisitSolver::~VisitSolver(){
}

void VisitSolver::loadSolver(string *parameters, int n)
{

  starting_position = "r0";
  string Paramers = parameters[0];

  char const *x[]={"dummy"};
  char const *y[]={"act-cost","triggered"};
  parseParameters(Paramers);
  affected = list<string>(x,x+1);
  dependencies = list<string>(y,y+2);

  string waypoint_file = "/root/ai4ro2/visits_domain/waypoint.txt";
  parseWaypoint(waypoint_file);

  string landmark_file = "/root/ai4ro2/visits_domain/landmark.txt";
  parseLandmark(landmark_file);

}

map<string,double> VisitSolver::callExternalSolver(map<string,double> initialState,bool isHeuristic)
{

  map<string, double> toReturn;
  map<string, double>::iterator iSIt = initialState.begin();
  map<string, double>::iterator isEnd = initialState.end();
  double dummy;
  double act_cost;


  map<string, double> trigger;

  for(;iSIt!=isEnd;++iSIt){

    string parameter = iSIt->first;
    string function = iSIt->first;
    double value = iSIt->second;

    function.erase(0,1);
    function.erase(function.length()-1,function.length());
    
    int n=function.find(" ");

    if(n!=-1)
    {

      string arg = function;
      string tmp = function.substr(n+1,5);

      function.erase(n,function.length()-1);
      arg.erase(0, n + 1);
      if(function=="triggered")
      {
        trigger[arg] = value > 0 ? 1:0;

        if (value>0)
        {

          std::cout << endl << "function value: " << value << endl;
          string from = tmp.substr(0,2);   // from and to are regions, need to extract wps (poses)
          string to = tmp.substr(3,2);        

          localize(from, to);
          std::cout << endl << "LOCALIZE"  << endl;

        }
      }
    }
    else
    {

      

      if(function=="dummy")
      {

        dummy = value;

      }
      else if(function=="act-cost")
      {
        
        act_cost = value;
        
      } 
    }
  }


  double results = calculateExtern(dummy, act_cost);
  if (ExternalSolver::verbose)
  {
    cout << "(dummy) " << results << endl;
  }
  toReturn["(dummy)"] = results;
  return toReturn;
}

list<string> VisitSolver::getParameters()
{
  return affected;
}

list<string> VisitSolver::getDependencies()
{
  return dependencies;
}


void VisitSolver::parseParameters(string parameters)
{

  int curr, next;
  string line;
  ifstream parametersFile(parameters.c_str());
  if (parametersFile.is_open())
  {
    while (getline(parametersFile,line))
    {
      curr=line.find(" ");
      string region_name = line.substr(0,curr).c_str();
      curr=curr+1;
      while(true )
      {
        next=line.find(" ",curr);
        region_mapping[region_name].push_back(line.substr(curr,next-curr).c_str());
        if (next ==-1)
          break;
        curr=next+1;
      }               
    }
  }
}

double VisitSolver::calculateExtern(double external, double total_cost)
{
  //float random1 = static_cast <float> (rand())/static_cast <float>(RAND_MAX);
  //double cost = 4;
  
  // Cost function
    // easy one
  //double cost = dist 
    // final one
  double cost = dist + tr_factor * tr;  

  return cost;
  
}

void VisitSolver::parseWaypoint(string waypoint_file)
{

  int curr, next;
  string line;
  double pose1, pose2, pose3;
  ifstream parametersFile(waypoint_file);
  if (parametersFile.is_open())
  {
    while (getline(parametersFile,line))
    {
      curr=line.find("[");
      string waypoint_name = line.substr(0,curr).c_str();

      curr=curr+1;
      next=line.find(",",curr);

      pose1 = (double)atof(line.substr(curr,next-curr).c_str());
      curr=next+1; next=line.find(",",curr);

      pose2 = (double)atof(line.substr(curr,next-curr).c_str());
      curr=next+1; next=line.find("]",curr);

      pose3 = (double)atof(line.substr(curr,next-curr).c_str());

      waypoint[waypoint_name] = vector<double> {pose1, pose2, pose3};
    }
  }
}

void VisitSolver::parseLandmark(string landmark_file)
{

  int curr, next;
  string line;
  double pose1, pose2, pose3;
  ifstream parametersFile(landmark_file);
  if (parametersFile.is_open())
  {
    while (getline(parametersFile,line))
    {
      curr=line.find("[");
      string landmark_name = line.substr(0,curr).c_str();
      
      curr=curr+1;
      next=line.find(",",curr);

      pose1 = (double)atof(line.substr(curr,next-curr).c_str());
      curr=next+1; next=line.find(",",curr);

      pose2 = (double)atof(line.substr(curr,next-curr).c_str());
      curr=next+1; next=line.find("]",curr);

      pose3 = (double)atof(line.substr(curr,next-curr).c_str());

      landmark[landmark_name] = vector<double> {pose1, pose2, pose3};
    }
  }
}

// Utility functions
double ab_distance(arma::vec p1, arma::vec p2)
{
  return sqrt(pow(p1(0) - p2(0), 2) + pow(p1(1) - p2(1), 2));
}

double norm_angle(double angle)
{
  angle = fmod(angle, 2 * arma::datum::pi);
  if (angle < 0) {
    angle += arma::datum::pi;
  }
  return angle;
}

void VisitSolver::localize( string from, string to)
{
  // Declare the variables from and to
  string wp_start, wp_stop;
  double wp_nominal_distance = 0;
  double wp_real_distance = 0;

  // Matrix initialization, required in the EKF algorithm
  arma::mat I(3,3, arma::fill::eye);                              // Identity matrix 3x3
  arma::mat I2(2,2, arma::fill::eye);                             // Identity matrix 2x2
  arma::mat Landmarks(3, landmark.size()-1, arma::fill::zeros);   // Landmarks matrix 
  arma::mat J_ekf(3,3, arma::fill::eye);                          // Jacobian matrix, constant for this problem 
  arma::mat C_k(2, 3, arma::fill::zeros);
  arma::mat Q_gamma(2,2);                                         // Measurement error covariance matrix
  arma::mat K_k(3, 2, arma::fill::zeros);                         // Kalman gain

  // Initialize the vecotrs required in the EKF algorithm
  arma::vec X_k(3);                          // Robot initial pose   
  arma::vec X_k_old(3);                      // Robot position at the beginning of the EKF step
  arma::vec X_k_zero(3);                     // Robot reference pose                   
  arma::vec step_array(3);                   // Displacement step  
  arma::vec noise(3);                        // Movement noise
  arma::vec rand_vec(3, arma::fill::randu);  // Random vector
  arma::vec Y(2);                            // Measurement vector with artificial noise 
  arma::vec g(2);                            // Expected measurements vector

  // Counter 
  int K = 0; 

  for (string ws : region_mapping[from])      
    {
      if (ws.empty())
      {
      } 
      else
      {
        for (string wg : region_mapping[to])
        {
          if (wg.empty())
          {
          }
          else
          {
            wp_nominal_distance = ab_distance(arma::conv_to<vec>::from(waypoint[ws]), arma::conv_to<vec>::from(waypoint[wg]));
            wp_stop = wg;
            wp_start = ws;
          }
          
        }  
      }    
    }

  std::cout << "ws_: " << from << endl << "wg_: " << to << endl << "distance : " << wp_nominal_distance << endl;
 
  // Distance only
  //dist = wp_nominal_distance;
  //tr = 0.0;

/*
------------------------------------------------------
                  EKF ALGORITHM
------------------------------------------------------
*/

  // Noise covariance matrices definition
    // Odometry/Process noise covariance matrix
  arma::mat Q_a = pow(odom_noise,2) * I;
    // Error covariance matrix initial value
  arma::mat P_k =  pow(cov_matr_noise, 2) * I;


  // Define the number of steps in which you want to split your movement
  const uint steps = ceil(wp_nominal_distance / robot_vel * odometry_freq);

  //  Read the current starting position 
  X_k = arma::conv_to<arma::vec>::from(waypoint[wp_start]);

  // The reference robot state, unaffected by noise at the beginning of the macrostep (Hyp.)
  X_k_zero = X_k;    // Robot reference pose

  // Normalize the pose
  waypoint[wp_start][2] = norm_angle(waypoint[wp_start][2]);
  waypoint[wp_stop][2] = norm_angle(waypoint[wp_stop][2]);

  for (uint i = 0; i < waypoint[wp_start].size(); i++)
  {
    step_array(i) = (waypoint[wp_stop][i] - waypoint[wp_start][i])/steps;
  }


  std::cout << endl << "X_k : " << X_k << endl;
  std::cout << endl << "Steps : " << steps << endl;
  std::cout << endl << "Step array : " << step_array << endl; 
  
  // Fill the landmar matrix with the coordinates of all the landmarks
    // Random seed
  arma::arma_rng::set_seed_random();
  
  int i = 0;
  for (auto beacon : landmark)
  {
    if (i >= 1)
    {       
      Landmarks.col(i-1) = arma::conv_to<arma::vec>::from(beacon.second);
      // std::cout << endl << "Landmarks matrix : " << Landmarks << endl;      
    }
    i++;
  } 
   
  // For each step in the action movement evaluate the new position of the robot
  // using the EKF using the position of the landmarks to check to verify its position

  for (uint i = 1; i < steps; i++)
  {
    // Random noise on the robot movement   
    noise = rand_vec * odom_noise;
    
    // PREDICTION STEP
      // Robot position prediction withouth and with noise
    X_k_old = X_k;
    X_k_zero = X_k_zero + step_array;    
    X_k = X_k + step_array + noise;
    std::cout << endl << "X_k_zero distance : " << X_k_zero << endl;
    std::cout << endl << "X_k distance : " << X_k << endl;
    // A priori estimate error covarince matrix
    P_k = J_ekf * P_k * J_ekf.t() + Q_a;
    
    for(arma::uword c=0; c < Landmarks.n_cols; ++c)
    {
      // Extract each landmark coordinate
      arma::vec land_col = Landmarks.col(c);
        
      // Estimate the robot theoretical distance from the landmark (Imagine the robot in its correct theoretical postion)
      double land_bot_distance = ab_distance(X_k_zero, land_col);
      
      // If the robot can detect the landmarks its position can be updated and 
      // the uncertainty in the covariance matrix reduced 
      if (land_bot_distance < landmarks_th)
      {
        // Measured values
        Y(0) = pow(ab_distance(X_k_zero, land_col), 2);                                       // landmark-robot measured squared distance
        Y(1) = std::atan2(land_col(1)-X_k_zero(1), land_col(0)-X_k_zero(0)) - X_k_zero(2);    // landmark-robot measured orientation
        Y = Y + arma::randn<vec>(2) * meas_noise;                                             // add some noise on the measure          
        //std::cout << endl << "Y distance : " << Y << endl;

        // Expected values from the prediction     
        g(0) = pow(ab_distance(X_k, land_col),2);                            // landmark-robot expected squared distance
        g(1) = std::atan2(land_col(1)-X_k(1), land_col(0)-X_k(0)) - X_k(2);  // landmark-robot expected orientation
        //std::cout << endl << "X_k distance : " << X_k << endl;

        //  Jacobian matrix of measurement vector, function of state varaibles only                  
        C_k(0, 0) = 2 * (land_col(0) - X_k(0));
        C_k(0, 1) = 2 * (land_col(1) - X_k(1));
        C_k(0, 2) = 0;
        C_k(1, 0) = (land_col(1) - X_k(1))/g(0);
        C_k(1, 1) = (land_col(0) - X_k(0))/g(0);
        C_k(1, 2) = -1;
       
        Q_gamma = std::pow(meas_noise,2) * I2;

        // Kalman gain        
        K_k = P_k * C_k.t() * arma::inv(C_k * P_k * C_k.t() + Q_gamma);
        std::cout << endl << "K_k kalman : " << K_k << endl;

        // Update step of the state variables
        X_k = X_k + K_k * (Y - g);
        // Update step of the model covariance matrix
        P_k = (I - K_k * C_k) * P_k;

        // Update the 
        K = K + 1 ;

      }  
      else
      {
        // If the landmarks are not available the robot can rely on the 
        // prediction step hence the system noise covariance matrix can only grow
      }   
    }

    // Update the travelled distance at the end of the EKF step
    double real_distance = ab_distance(X_k, X_k_old);
    wp_real_distance = wp_real_distance + real_distance;

  }

  std::cout << endl << "Nominal distance : " << wp_nominal_distance << endl;
  std::cout << endl << "Real distance : " << wp_real_distance << endl;
  std::cout << endl << "Counter  : " << K << endl;
  dist = wp_real_distance;
  tr = arma::trace(P_k);  
  //std::cout << endl << "P_k : " << P_k << endl;

} 
