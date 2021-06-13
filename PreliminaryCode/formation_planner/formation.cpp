#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <opencv4/opencv2/core.hpp>

double CostFunction(double pose_x, double x, double offset_x)
{   
    //  Quadratic optimization function
    //  Offset determines a robot's position
    return pow((x-(pose_x + offset_x)),2);
}
double GradientFunction(double pose_x, double x, double offset_x)
{
    //  Gradient of the cost function
    //  Offset determines a robot's position
    return 2*(x-(pose_x + offset_x));
}
double CalculateUpdate(double pose_x, double x_previous,double offset_x)
{
    //  Gradient descent update
    //  Offset determines a robot's position
    double alpha {0.5}; //step parameter
    double gradient;
    double x_updated;
    
    gradient     = GradientFunction(pose_x, x_previous,offset_x);
    x_updated    = x_previous - gradient*alpha;
    return  x_updated;
}

double FindGoToPoint(double current_cost, double pose_x, double x_previous, double offset_x)
{
    // Main loop for finding a go_to point
    // While cost function is greated than threshold, the state will be updating
    // In case cost function is lower than threshold, the state is preserved
    double x_updated;
    
    if (std::abs(current_cost)<=0.1)
    {
        x_updated = x_previous;
    }
    while(std::abs(current_cost)>0.1)
    {
        x_updated       = CalculateUpdate(pose_x,x_previous,offset_x);
        current_cost    = CostFunction(pose_x, x_updated,offset_x);       
        x_previous      = x_updated;
    }
    return x_updated;

}


int main(int argc, char** argv)
{
    std::vector<cv::Point3f>    points;
    cv::Point3f                 initial_state    = cv::Point3f(1.0,1.0,0);
    cv::Point3f                 final_state      = cv::Point3f(1.0,1.0,0);

    // SIMULATION
    for (int i= 0; i< 50; i++)
    {
        //this creates a vector of points of simulated observations
        points.push_back(cv::Point3f(5+i/10.0,5+i/10.0,2+i/10));
    }
    // End of simulation

    // Gradient Descent Parameters
    double x_previous = initial_state.x;
    double y_previous = initial_state.y;
    double z_previous = initial_state.z;
    
    double x_updated, y_updated, z_updated;
    double current_cost_x, current_cost_y, current_cost_z;
    
    // Offset determines the position of a robot 
    // relatively to observation
    double offset_x {5};
    double offset_y {5};
    double offset_z {1};

    for (int i = 0;i<50;i++)
    {   
        // Main loop
        // Calculation of cost function values
        current_cost_x = CostFunction(points[i].x, x_previous,offset_x);
        current_cost_y = CostFunction(points[i].y, y_previous,offset_y);
        current_cost_z = CostFunction(points[i].z, z_previous,offset_z);

        // Determining the optimal state
        x_updated = FindGoToPoint(current_cost_x, points[i].x, x_previous,offset_x);
        y_updated = FindGoToPoint(current_cost_y, points[i].y, y_previous,offset_y);
        z_updated = FindGoToPoint(current_cost_z, points[i].z, z_previous,offset_z);

        std::cout<<"Observed point        "<<" x: "<<points[i].x<<" y: "<<points[i].y<<" z: "<<points[i].z ;
        std::cout<<"\t\tRobot's go_to position"<<" x: "<<x_updated<<" y: "  <<y_updated<<" z: "  <<z_updated    <<'\n';
        
    }
    
    // Final go_to state
    final_state.x = x_updated;
    final_state.y = y_updated;
    final_state.z = z_updated;

    std::cout<<"************************************"<<'\n';
    std::cout<<"Simulation of goal steps is complete"<<'\n';
    std::cout<<"Initial state: "<<initial_state<<'\n';
    std::cout<<"Final state:   "<<final_state<<'\n';

    return 0;
}