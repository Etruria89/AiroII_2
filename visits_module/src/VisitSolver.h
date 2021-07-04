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


#ifndef TESTSOLVER_H
#define TESTSOLVER_H

#include "ExternalSolver.h"
#include <string>
#include <vector>
#include <sstream>
#include <iomanip>
#include <queue>
#include <unordered_map>

#include "armadillo"

using namespace std;

class VisitSolver : public ExternalSolver
{
public:
    VisitSolver();
    ~VisitSolver();
    virtual void loadSolver(string* parameters, int n);
    virtual map<string,double> callExternalSolver(map<string,double> initialState, bool isHeuristic);
    virtual  list<string> getParameters();
    virtual  list<string> getDependencies();
    map<string, vector<double>> waypoint;
    map<string, vector<double>> landmark;
   
    void parseWaypoint(string waypoint_file);
    void parseLandmark(string landmark_file);
     
    map<string, vector<string>> region_mapping;
    vector <string> source, target; 
    string starting_position;
     
   
   
    void parseParameters(string parameters);
 

private:
    list<string> affected;
    list<string> dependencies;
      
    double calculateExtern(double external, double total_cost);
    vector<string> findParameters(string line, int&n);

    // Extra functions and methods for the solution of the assignment
    void localize(string from, string to);    

    // Global variables 
        //Cost function components
    double dist;
    double tr;
        // Parameters
    const double robot_vel = 0.2;           // Robot velocity                                           [m/s]
    const double odometry_freq = 10;        // Frequency of environment scan                            [Hz]
    const double odom_noise = 0.01;         // Odometry/Process noise                                   [m]  
    const double cov_matr_noise = 0.2;      // Estimate error covariance matrix   
    const double landmarks_th = 1.0;        // Maximum distance to detect a landmark                    [m]
    const double meas_noise = 0.05;          // Measurement noise
    const double tr_factor = 20.0;          // Trace factor to weight the trace in the cost function    [-] 
                                            // set to have the effect of the trace comparable with the 
                                            // path lenght    
};

#endif // TESTSOLVER_H
