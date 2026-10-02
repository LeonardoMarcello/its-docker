#ifndef TACTIPGKD_H
#define TACTIPGKD_H

#include <string>
#include <cmath>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/LU>
#include <opencv2/opencv.hpp>

#include "its_msgs/TacTipMarkers.h"
#include "its_msgs/TacTipDensity.h"

namespace tactipGKD {

    /* 
    *   TO DO: Descrizione libreria
    */

    // ===========================================================================
    // Fitting type
    // ===========================================================================
    enum class FitType: unsigned short int {
        Linear = 1,        // y(x) = a + b*x
        Quadratic = 2,     // y(x) = a*x + b*x^2   
        Power = 3,         // y(x) = a*x^b
        Logarithmic = 4    // y(x) = a*ln(b*x)
    };
    // ===========================================================================
    // GKD params
    // ===========================================================================
    struct GKDparams{
        std::string id;                              // Tactip frame id
        int resolution;                // Resolution of pixels for Density estimation
        double h;                                     // Gaussian Kernel std. dev [pixel] 
        double mm2pxl;                                // conversion millimeter to pixel in fingertip's plane [pixel/mm]
        std::vector<double> density0;                 // density at rest [marker/mm^2]

        double density_threshold;                     // Density threshold for contact detection
         
        FitType fit_type;                             // Type of curve used in deformation estimation
        std::vector<double> fit_coeff = {0.0, 0.0};   // Fitting curve parameters (a = fit_coeff[0], b = fit_coeff[1], ...)

        int width;                     // Tactip image width in pixels
        int height;                    // Tactip image height in pixels
        double depth;                                 // fingertip's plane distance from camera [mm]
        double fx;                                    // intrinsic camera param x focal lenght
        double fy;                                    // intrinsic camera param y focal lenght
        double cx;                                    // intrinsic camera param x optical center
        double cy;                                    // intrinsic camera param y optical center
                
        bool verbose;                                 // Print routine elapsed time
    };



    // ===========================================================================
    // GKD routine
    // ===========================================================================
    void GaussianKernelDensity(its_msgs::TacTipDensity &density, its_msgs::TacTipMarkers markers, GKDparams params);
    /* GaussianKernelDensity:
    *       Evaluate the Gaussian Kernel Density given markers position.
    */
    //cv::Mat contactRegion(its_msgs::TacTipDensity density, params params);
    void Density2deformation(double &deformation, its_msgs::TacTipDensity density, GKDparams params);
    /* Density2deformation:
    *       Estimate the tip displacement by using the integral of Gaussian Kernel Density variation over the contact region.
    */
    void Density2cv(cv::Mat &heatmap, its_msgs::TacTipDensity density);
    /* Density2cv:
    *       Convert the Gaussian Kernel Density to a heatmap cv image.
    */

}  // namespace tactipGKD
#endif // TACTIPGKD_H