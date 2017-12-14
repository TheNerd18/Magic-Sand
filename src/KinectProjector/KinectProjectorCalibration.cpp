/***********************************************************************
KinectProjectorCalibration.cpp - KinectProjectorCalibration compute
the calibration of the kinect and projector.
Copyright (c) 2016 Thomas Wolf

--- Adapted from ofxKinectProjectorToolkit by Gene Kogan:
https://github.com/genekogan/ofxKinectProjectorToolkit
Copyright (c) 2014 Gene Kogan

This file is part of the Magic Sand.

The Magic Sand is free software; you can redistribute it
and/or modify it under the terms of the GNU General Public License as
published by the Free Software Foundation; either version 2 of the
License, or (at your option) any later version.

The Magic Sand is distributed in the hope that it will be
useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
General Public License for more details.

You should have received a copy of the GNU General Public License along
with the Magic Sand; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
***********************************************************************/

#include "KinectProjectorCalibration.h"
#include "dlib/statistics.h"
#include "levmar/levmar.h"

void levmar_proj_func(double *p, double *hx, int m, int n, void *adata);
void project_points(double *p, int num_parameters, double *projected, int num_pairs, double *rawPairs);
dlib::matrix<double, 3, 4> build_extrinsics_matrix(double *p);
dlib::matrix<double, 3, 3> build_intrinsics_matrix(double *p);
dlib::matrix<double, 3, 4> build_proj_matrix(double *p);

ofxKinectProjectorToolkit::ofxKinectProjectorToolkit(ofVec2f sprojRes, ofVec2f skinectRes) {
	projRes = sprojRes;
	kinectRes = skinectRes;
    calibrated = false;
}

void ofxKinectProjectorToolkit::build_A(std::vector<ofVec3f> pairsKinect, std::vector<ofVec2f> pairsProjector)
{
   int nPairs = pairsKinect.size();
    A.set_size(nPairs*2, 11);
    
    for (int i=0; i<nPairs; i++) {
        A(2*i, 0) = pairsKinect[i].x;
        A(2*i, 1) = pairsKinect[i].y;
        A(2*i, 2) = pairsKinect[i].z;
        A(2*i, 3) = 1;
        A(2*i, 4) = 0;
        A(2*i, 5) = 0;
        A(2*i, 6) = 0;
        A(2*i, 7) = 0;
        A(2*i, 8) = -pairsKinect[i].x * pairsProjector[i].x;
        A(2*i, 9) = -pairsKinect[i].y * pairsProjector[i].x;
        A(2*i, 10) = -pairsKinect[i].z * pairsProjector[i].x;
        
        A(2*i+1, 0) = 0;
        A(2*i+1, 1) = 0;
        A(2*i+1, 2) = 0;
        A(2*i+1, 3) = 0;
        A(2*i+1, 4) = pairsKinect[i].x;
        A(2*i+1, 5) = pairsKinect[i].y;
        A(2*i+1, 6) = pairsKinect[i].z;
        A(2*i+1, 7) = 1;
        A(2*i+1, 8) = -pairsKinect[i].x * pairsProjector[i].y;
        A(2*i+1, 9) = -pairsKinect[i].y * pairsProjector[i].y;
        A(2*i+1, 10) = -pairsKinect[i].z * pairsProjector[i].y;
    }
}

void ofxKinectProjectorToolkit::build_y(std::vector<ofVec3f> pairsKinect, std::vector<ofVec2f> pairsProjector)
{
    int nPairs = pairsKinect.size();
    y.set_size(nPairs*2, 1);
    
    for (int i=0; i<nPairs; i++) {
        y(2*i, 0)  = pairsProjector[i].x;
        y(2*i+1, 0) = pairsProjector[i].y;
    }
}

void ofxKinectProjectorToolkit::calibrate(vector<ofVec3f> pairsKinect,
                                          vector<ofVec2f> pairsProjector) {
    int nPairs = pairsKinect.size();

    double mean_x = 0, mean_y = 0, mean_z = 0;

    for (int i = 0 ; i < nPairs; i++)
    {
        mean_x += pairsKinect[i].x;
        mean_y += pairsKinect[i].y;
        mean_z += pairsKinect[i].z;
    }

    mean_x /= nPairs;
    mean_y /= nPairs;
    mean_z /= nPairs;

    dlib::running_stats<double> radii_stats;
    dlib::matrix<double, 0, 0> radii;
    radii.set_size(1, nPairs);

    for (int i = 0 ; i < nPairs; i++)
    {
        radii(0, i) = sqrt(pow(pairsKinect[i].x - mean_x, 2) + pow(pairsKinect[i].y - mean_y, 2) + pow(pairsKinect[i].z - mean_z, 2));
        radii_stats.add(radii(0, i));
    }

    {
        double r_stddev = radii_stats.stddev();
        int idx = 0;
        int j = 0;
        int removed = 0;
        while (idx < nPairs)
        {
            if (radii(0, j) > 2*r_stddev)
            {
                removed++;
                pairsKinect.erase(pairsKinect.begin() + idx);
                pairsProjector.erase(pairsProjector.begin() + idx);
            }
            else
            {
                idx++;
            }

            j++;
        }
        std::cout << "Removed " << removed << " pairs" <<std::endl;
    }

    nPairs = pairsKinect.size();

    build_A(pairsKinect, pairsProjector);
    build_y(pairsKinect, pairsProjector);
    //     y(2*i+1, 0) = pairsProjector[i].y;
    // }
    
    dlib::qr_decomposition<dlib::matrix<double, 0, 11> > qrd(A);
    dlib::matrix<double> qr_x = qrd.solve(y);
    // cout << "x: "<< x << endl;
    ofMatrix4x4 qr_projMatrice = ofMatrix4x4(x(0,0), x(1,0), x(2,0), x(3,0),
                              x(4,0), x(5,0), x(6,0), x(7,0),
                              x(8,0), x(9,0), x(10,0), 1,
                              0, 0, 0, 1);
    cout << "QR error = " << ComputeReprojectionError(qr_projMatrice, pairsKinect, pairsProjector) << x << endl;

    int r = 11;

    while (ComputeReprojectionError(calibrate_svd(pairsKinect, pairsProjector, r-1), pairsKinect, pairsProjector) < ComputeReprojectionError(calibrate_svd(pairsKinect, pairsProjector, r), pairsKinect, pairsProjector))
    {
        r -= 1;
    }

    ofMatrix4x4 projMatriceSVD = calibrate_svd(pairsKinect, pairsProjector, r);

    cout << "SVD error = " << ComputeReprojectionError(projMatriceSVD, pairsKinect, pairsProjector) << endl;

    parameters_struct parameters = get_parameters(projMatriceSVD);
    
    // Find extrinsic parameters to use as first estimate for LM
    double theta_y1 = -asin(parameters.R(2, 0));
    double theta_y2 = PI + asin(parameters.R(2, 0));
    double theta_x1 = atan2(parameters.R(2, 1) / cos(theta_y1), parameters.R(2, 2) / cos(theta_y1)) * 180.0 / PI;
    double theta_x2 = atan2(parameters.R(2, 1) / cos(theta_y2), parameters.R(2, 2) / cos(theta_y2)) * 180.0 / PI;
    double theta_z1 = atan2(parameters.R(1, 0) / cos(theta_y1), parameters.R(0, 0) / cos(theta_y1)) * 180.0 / PI;
    double theta_z2 = atan2(parameters.R(1, 0) / cos(theta_y2), parameters.R(0, 0) / cos(theta_y2)) * 180.0 / PI;

    double params[] = {parameters.K(0, 0), parameters.K(0, 1), parameters.K(0, 2),
        parameters.K(1, 1), parameters.K(1, 2),
        theta_x1, theta_y1*180/PI, theta_z1,
        parameters.t(0, 0), parameters.t(0, 1), parameters.t(0, 2),
        0.0, 0.0, // k1, k2
        0.0, 0.0 // p1, p2
    };


    // Flatten out the projector and kinect pairs
    std::vector<double> measurements;
    for (auto it = pairsProjector.begin(); it < pairsProjector.end(); it++)
    {
        measurements.push_back((*it).x);
        measurements.push_back((*it).y);
    }
    
    std::vector<double> kinect_pairs_vect;
    for (int i = 0; i < pairsKinect.size(); i++)
    {
        kinect_pairs_vect.push_back(pairsKinect[i].x);
        kinect_pairs_vect.push_back(pairsKinect[i].y);
        kinect_pairs_vect.push_back(pairsKinect[i].z);
        kinect_pairs_vect.push_back(1);
    }

    double info[LM_INFO_SZ];
                    /* O: information regarding the minimization. Set to NULL if don't care
                     * info[0]= ||e||_2 at initial p.
                     * info[1-4]=[ ||e||_2, ||J^T e||_inf,  ||Dp||_2, \mu/max[J^T J]_ii ], all computed at estimated p.
                     * info[5]= # iterations,
                     * info[6]=reason for terminating: 1 - stopped by small gradient J^T e
                     *                                 2 - stopped by small Dp
                     *                                 3 - stopped by itmax
                     *                                 4 - singular matrix. Restart from current p with increased \mu
                     *                                 5 - no further error reduction is possible. Restart with increased mu
                     *                                 6 - stopped by small ||e||_2
                     *                                 7 - stopped by invalid (i.e. NaN or Inf) "func" values; a user error
                     * info[7]= # function evaluations
                     * info[8]= # Jacobian evaluations
                     * info[9]= # linear systems solved, i.e. # attempts for reducing error
                     */
    int total_iterations = dlevmar_dif(levmar_proj_func, params, &measurements[0],
        11, measurements.size(), 10000,
        NULL, info, NULL, NULL, &kinect_pairs_vect[0]);

    std::cout << "LM error at initial " << info[0] << std::endl;
    std::cout << "LM error at estimated " << info[1] << std::endl;
    std::cout << "LM num iterations " << info[5] << std::endl;
    std::cout << "LM reason for stopping " << info[6] << std::endl;

    dlib::matrix<double, 3, 4> P_levmar_dlib = build_proj_matrix(params);
    projMatrice = ofMatrix4x4(
        P_levmar_dlib(0,0), P_levmar_dlib(0,1), P_levmar_dlib(0,2), P_levmar_dlib(0,3),
        P_levmar_dlib(1,0), P_levmar_dlib(1,1), P_levmar_dlib(1,2), P_levmar_dlib(1,3),
        P_levmar_dlib(2,0), P_levmar_dlib(2,1), P_levmar_dlib(2,2), 1,
        0, 0, 0, 1);

    dlib::matrix<double, 3, 3> levmar_K;
    levmar_K(0, 0) = params[0];
    levmar_K(0, 1) = params[1];
    levmar_K(0, 2) = params[2];
    levmar_K(1, 0) = 0;
    levmar_K(1, 1) = params[3];
    levmar_K(1, 2) = params[4];
    levmar_K(2, 0) = 0;
    levmar_K(2, 1) = 0;
    levmar_K(2, 2) = 1;

    total_iterations = dlevmar_dif(levmar_proj_func, params, &measurements[0],
        sizeof(params) / sizeof(params[0]), measurements.size(), 10000,
        NULL, info, NULL, NULL, &kinect_pairs_vect[0]);

    std::cout << "LM error at initial " << info[0] << std::endl;
    std::cout << "LM error at estimated " << info[1] << std::endl;
    std::cout << "LM num iterations " << info[5] << std::endl;
    std::cout << "LM reason for stopping " << info[6] << std::endl;

    // double lm_error = ComputeReprojectionErrorWithDistortion(params, 11, pairsKinect, pairsProjector);

    // total_iterations = dlevmar_dif(levmar_proj_func, params, &measurements[0],
    //     sizeof(params) / sizeof(params[0]), measurements.size(), 3000,
    //     // 11, measurements.size() / 2, 3000,
    //     NULL, info, NULL, NULL, &kinect_pairs_vect[0]);

    // // // std::cout << "projPairs " << std::endl << pairsProjector[0] << std::endl;

    // if (humans)
    // {
    //     std::cout << "LM error at initial " << info[0] << std::endl;
    //     std::cout << "LM error at estimated " << info[1] << std::endl;
    //     std::cout << "LM num iterations " << info[5] << std::endl;
    //     std::cout << "LM reason for stopping " << info[6] << std::endl;
    // }

    // dlib::matrix<double, 3, 3> levmar_K;
    // levmar_K(0, 0) = params[0];
    // levmar_K(0, 1) = params[1];
    // levmar_K(0, 2) = params[2];
    // levmar_K(1, 0) = 0;
    // levmar_K(1, 1) = params[3];
    // levmar_K(1, 2) = params[4];
    // levmar_K(2, 0) = 0;
    // levmar_K(2, 1) = 0;
    // levmar_K(2, 2) = 1;

    // if (humans)
    // {
    //     std::cout << "levmar_K = " << std::endl << levmar_K << std::endl;
    //     std::cout << "levmar theta (" << params[5] << ", " << params[6] << ", " << params[7] << ")" << std::endl;
    //     std::cout << "levmar t = " << Vec3f(params[8], params[9], params[10]) << std::endl;
    //     std::cout << "k1 = " << params[11] << ", k2 = " << params[12] << std::endl;
    //     std::cout << "p1 = " << params[11] << ", p2 = " << params[12] << std::endl;
    // }
    // else
    // {
    //     std::cout << levmar_K(0, 0) << ", " << levmar_K(0, 1) << ", " << levmar_K(0, 2) << ", "
    //          << levmar_K(1, 1) << ", " << levmar_K(1, 2) << ", ";
    //     std::cout << params[5] << ", "  << params[6] << ", "  << params[7] << ", ";
    //     std::cout << params[8] << ", "  << params[9] << ", "  << params[10] << ", ";
    //     std::cout << params[11] << ", "  << params[12] << ", "  << params[13] << ", " << params[13] << ", ";
    // }

    // std::cout << "params[0] = " << params[0] << std::endl;
    // std::cout << "params[1] = " << params[1] << std::endl;
    // std::cout << "params[2] = " << params[2] << std::endl;
    // std::cout << "params[3] = " << params[3] << std::endl;
    // std::cout << "params[4] = " << params[4] << std::endl;
    // std::cout << "params[5] = " << params[5] << std::endl;
    // std::cout << "params[6] = " << params[6] << std::endl;
    // std::cout << "params[7] = " << params[7] << std::endl;
    // std::cout << "params[8] = " << params[8] << std::endl;
    // std::cout << "params[9] = " << params[9] << std::endl;
    // std::cout << "params[10] = " << params[10] << std::endl;
    std::cout << "params[11] = " << params[11] << std::endl;
    std::cout << "params[12] = " << params[12] << std::endl;
    std::cout << "params[13] = " << params[13] << std::endl;
    std::cout << "params[14] = " << params[13] << std::endl;

    std::cout << "LM error, no distortion = " << ComputeReprojectionError(projMatrice, pairsKinect, pairsProjector) << std::endl;
    std::cout << "LM error, distortion = " << ComputeReprojectionErrorWithDistortion(params, sizeof(params) / sizeof(params[0]),
        pairsKinect, pairsProjector) << std::endl;

    if (ComputeReprojectionError(projMatriceSVD, pairsKinect, pairsProjector) < ComputeReprojectionError(projMatriceSVD, pairsKinect, pairsProjector))
    {
        projMatrice = projMatriceSVD;
    }

    calibrated = true;
}

parameters_struct ofxKinectProjectorToolkit::get_parameters(ofMatrix4x4 projection_matrix)
{
    dlib::matrix<double, 3, 3> p_left;
    p_left(0, 0) = projection_matrix(0, 0);
    p_left(0, 1) = projection_matrix(0, 1);
    p_left(0, 2) = projection_matrix(0, 2);
    p_left(1, 0) = projection_matrix(1, 0);
    p_left(1, 1) = projection_matrix(1, 1);
    p_left(1, 2) = projection_matrix(1, 2);
    p_left(2, 0) = projection_matrix(2, 0);
    p_left(2, 1) = projection_matrix(2, 1);
    p_left(2, 2) = projection_matrix(2, 2);

    dlib::matrix<double, 3, 1> p_right;
    p_right(0, 0) = projection_matrix(0, 3);
    p_right(1, 0) = projection_matrix(1, 3);
    p_right(2, 0) = projection_matrix(2, 3);

    dlib::matrix<double, 3, 3> p_left_inv = inv(p_left);

    dlib::qr_decomposition<dlib::matrix<double, 3, 3> > qrd(p_left_inv);

    parameters_struct output;

    output.R = inv(qrd.get_q());
    output.K = inv(qrd.get_r());

    if (output.K(0, 0) < 0)
    {
        output.K(0, 0) *= -1;
        output.R(0, 0) *= -1;
        output.R(0, 1) *= -1;
        output.R(0, 2) *= -1;
    }
    if (output.K(1, 1) < 0)
    {
        output.K(1, 1) *= -1;
        output.K(0, 2) *= -1;
        output.R(1, 0) *= -1;
        output.R(1, 1) *= -1;
        output.R(1, 2) *= -1;
    }
    if (output.K(2, 2) < 0)
    {
        output.K(0, 2) *= -1;
        output.K(1, 2) *= -1;
        output.K(2, 2) *= -1;

        output.R(2, 0) *= -1;
        output.R(2, 1) *= -1;
        output.R(2, 2) *= -1;
    }

    float tmp = output.K(2, 2);

    output.K = output.K / tmp;
    
    output.t = inv(output.K) * (p_right / tmp);

    return output;
}


ofMatrix4x4 ofxKinectProjectorToolkit::calibrate_svd(std::vector<ofVec3f> pairsKinect, std::vector<ofVec2f> pairsProjector, int r) 
{
    build_A(pairsKinect, pairsProjector);
    build_y(pairsKinect, pairsProjector);
    dlib::matrix<double, 11, 1> x;

    dlib::matrix<double, 0, 0> U;
    dlib::matrix<double, 0, 0> W;
    dlib::matrix<double, 0, 0> V;
    
    svd(A, U, W, V);
    dlib::matrix<double, 0, 0> s = dlib::diag(W);

    dlib::matrix<double, 0, 0> d = trans(U)*y;

    dlib::matrix<double, 0, 1> sliced_d;
    sliced_d.set_size(A.nc(), 1);

    for (int i = 0; i < r; i++)
    {
        sliced_d(i, 0) = d(i, 0) / s(i, 0);
    }

    for (int i = r; i < A.nc(); i++)
    {
        sliced_d(i, 0) = 0;
    }

    x = V * sliced_d;

    return ofMatrix4x4(x(0,0), x(1,0), x(2,0), x(3,0),
                              x(4,0), x(5,0), x(6,0), x(7,0),
                              x(8,0), x(9,0), x(10,0), 1,
                              0, 0, 0, 1);
}


double ofxKinectProjectorToolkit::ComputeReprojectionError(ofMatrix4x4 projMatrix, std::vector<ofVec3f> pairsKinect, std::vector<ofVec2f> pairsProjector)
{
    double PError = 0;

    for (int i = 0; i < pairsKinect.size(); i++)
    {
        ofVec4f wc = pairsKinect[i];
        wc.w = 1;

        ofVec4f screenPos = projMatrix*wc;

        ofVec2f projectedPoint(screenPos.x / screenPos.z, screenPos.y / screenPos.z);
        ofVec2f projP = pairsProjector[i];

        double D = sqrt((projectedPoint.x - projP.x) * (projectedPoint.x - projP.x) + (projectedPoint.y - projP.y) * (projectedPoint.y - projP.y));

        PError += D;
    }
    PError /= (double)pairsKinect.size();

    return PError;
}

double ofxKinectProjectorToolkit::ComputeReprojectionErrorWithDistortion(double *parameters, int num_parameters,
    std::vector<ofVec3f> pairsKinect, std::vector<ofVec2f> pairsProjector)
{
    double PError = 0;

    int n = pairsKinect.size();

    std::vector<double> kinectPairsFlattened;
    for (int i = 0; i < pairsKinect.size(); i++)
    {
        kinectPairsFlattened.push_back(pairsKinect[i].x);
        kinectPairsFlattened.push_back(pairsKinect[i].y);
        kinectPairsFlattened.push_back(pairsKinect[i].z);
        kinectPairsFlattened.push_back(1);
    }

    std::vector<double> projected_points;
    projected_points.resize(n * 2);

    project_points(parameters, num_parameters, &projected_points[0], pairsKinect.size(), &kinectPairsFlattened[0]);

    dlib::matrix<double, 0, 0> imagePoints;
    imagePoints.set_size(n, 2);
    imagePoints = trans(dlib::mat(&projected_points[0], n, 2));
    // // std::cout << "imagePoints = " << std::endl << imagePoints << std::endl;

    for (int i = 0; i < n; i++)
    {
        double ud = imagePoints(0, i);
        double vd = imagePoints(1, i);

        ofVec2f projP = pairsProjector[i];

        double D = sqrt((ud - projP.x) * (ud - projP.x) + (vd - projP.y) * (vd - projP.y));

        PError += D;
    }
    PError /= (double)pairsKinect.size();

    return PError;
}

ofMatrix4x4 ofxKinectProjectorToolkit::getProjectionMatrix() {
    return projMatrice;
}

ofVec2f ofxKinectProjectorToolkit::getProjectedPoint(ofVec3f worldPoint) {
    ofVec4f pts = ofVec4f(worldPoint);
    pts.w = 1;
    ofVec4f rst = projMatrice*(pts);
    ofVec2f projectedPoint(rst.x/rst.z, rst.y/rst.z);
    return projectedPoint;
}

vector<double> ofxKinectProjectorToolkit::getCalibration()
{
    vector<double> coefficients;
    for (int i=0; i<11; i++) {
        coefficients.push_back(x(i, 0));
    }
    return coefficients;
}

bool ofxKinectProjectorToolkit::loadCalibration(string path){
    ofXml xml;
    if (!xml.load(path))
        return false;
	xml.setTo("RESOLUTIONS");
	ofVec2f sprojRes = xml.getValue<ofVec2f>("PROJECTOR");
	ofVec2f skinectRes = xml.getValue<ofVec2f>("KINECT");
	if (sprojRes!=projRes || skinectRes!=kinectRes)
		return false;
    xml.setTo("//CALIBRATION/COEFFICIENTS");
    for (int i=0; i<11; i++) {
        x(i, 0) = xml.getValue<float>("COEFF"+ofToString(i));
    }
    projMatrice = ofMatrix4x4(x(0,0), x(1,0), x(2,0), x(3,0),
                              x(4,0), x(5,0), x(6,0), x(7,0),
                              x(8,0), x(9,0), x(10,0), 1,
                              0, 0, 0, 0);
    calibrated = true;
    return true;
}

bool ofxKinectProjectorToolkit::saveCalibration(string path){
    ofXml xml;
	xml.addChild("CALIBRATION");
	xml.setTo("//CALIBRATION");
	xml.addChild("RESOLUTIONS");
	xml.setTo("RESOLUTIONS");
	xml.addValue("PROJECTOR", projRes);
	xml.addValue("KINECT", kinectRes);
	xml.setTo("//CALIBRATION");
	xml.addChild("COEFFICIENTS");
	xml.setTo("COEFFICIENTS");
	for (int i=0; i<11; i++) {
        ofXml coeff;
        coeff.addValue("COEFF"+ofToString(i), x(i, 0));
        xml.addXml(coeff);
    }
    xml.setToParent();
    return xml.save(path);
}

dlib::matrix<double, 3, 4> build_extrinsics_matrix(double *p)
{
    double tx = p[8];
    double ty = p[9];
    double tz = p[10];
    
    dlib::matrix<double, 3, 3> R;
    dlib::matrix<double, 3, 3> R_x;
    dlib::matrix<double, 3, 3> R_y;
    dlib::matrix<double, 3, 3> R_z;
    dlib::matrix<double, 3, 1> t;
    
    double theta_x = p[5] * PI / 180;
    double theta_y = p[6] * PI / 180;
    double theta_z = p[7] * PI / 180;

    R_x(0, 0) = 1;
    R_x(0, 1) = 0;
    R_x(0, 2) = 0;
    R_x(1, 0) = 0;
    R_x(1, 1) = cos(theta_x);
    R_x(1, 2) = -sin(theta_x);
    R_x(2, 0) = 0;
    R_x(2, 1) = sin(theta_x);
    R_x(2, 2) = cos(theta_x);

    R_y(0, 0) = cos(theta_y);
    R_y(0, 1) = 0;
    R_y(0, 2) = sin(theta_y);
    R_y(1, 0) = 0;
    R_y(1, 1) = 1;
    R_y(1, 2) = 0;
    R_y(2, 0) = -sin(theta_y);
    R_y(2, 1) = 0;
    R_y(2, 2) = cos(theta_y);

    R_z(0, 0) = cos(theta_z);
    R_z(0, 1) = -sin(theta_z);
    R_z(0, 2) = 0;
    R_z(1, 0) = sin(theta_z);
    R_z(1, 1) = cos(theta_z);
    R_z(1, 2) = 0;
    R_z(2, 0) = 0;
    R_z(2, 1) = 0;
    R_z(2, 2) = 1;

    R = R_z*R_y*R_x;
    // std::cout << "R = " << std::endl << R << std::endl;

    t(0) = p[8];
    t(1) = p[9];
    t(2) = p[10];

    dlib::matrix<double, 3, 4> E = join_rows(R, t);

    return E;
}

dlib::matrix<double, 3, 3> build_intrinsics_matrix(double *p)
{
    dlib::matrix<double, 3, 3> K;

    K(0, 0) = p[0];
    K(0, 1) = p[1];
    K(0, 2) = p[2];
    K(1, 0) = 0;
    K(1, 1) = p[3];
    K(1, 2) = p[4];
    K(2, 0) = 0;
    K(2, 1) = 0;
    K(2, 2) = 1;

    return K;
}

dlib::matrix<double, 3, 4> build_proj_matrix(double *p)
{
    // std::cout << "K = " << std::endl << K << std::endl;

    // std::cout << "t = " << t << std::endl;

    // std::cout << test_K << std::endl;
    // std::cout << R_x << std::endl;
    // std::cout << R_y << std::endl;
    // std::cout << R_z << std::endl;
    // std::cout << R_z << std::endl;
    // std::cout << R << std::endl;
    // std::cout << t << std::endl;

    dlib::matrix<double, 3, 4> P_d = build_intrinsics_matrix(p)*build_extrinsics_matrix(p);

    P_d /= P_d(2, 3);

    return P_d;
}

void project_points(double *p, int num_parameters, double *projected, int num_pairs, double *rawPairs)
{
    double k1 = 0, k2 = 0;
    double p1 = 0, p2 = 0;
    if (num_parameters > 11)
    {
        k1 = p[11];
        k2 = p[12];
    }
    if (num_parameters > 13)
    {
        p1 = p[13];
        p2 = p[14];
    }

    dlib::matrix<double, 0, 0> pairsKinect; // = reinterpret_cast<dlib::matrix<double, 0, 0>*>(adata);
    pairsKinect.set_size(num_pairs, 4);

    pairsKinect = dlib::mat(rawPairs, num_pairs, 4);

    dlib::matrix<double, 3, 4> P_d = build_proj_matrix(p);
    dlib::matrix<double, 3, 4> E = build_extrinsics_matrix(p);
    dlib::matrix<double, 3, 3> K = build_intrinsics_matrix(p);

    dlib::matrix<double, 0, 0> X_p;
    X_p.set_size(3, num_pairs);

    X_p = E * trans(pairsKinect);

    dlib::matrix<double, 3, 0> X_pv;
    X_pv.set_size(3, num_pairs);
    
    set_rowm(X_pv, 0) = pointwise_multiply(rowm(X_p, 0), reciprocal(rowm(X_p, 2)));
    set_rowm(X_pv, 1) = pointwise_multiply(rowm(X_p, 1), reciprocal(rowm(X_p, 2)));
    set_rowm(X_pv, 2) = dlib::ones_matrix<double>(1, num_pairs);

    dlib::matrix<double, 1, 0> r2;
    r2.set_size(1, num_pairs);
    r2 = dlib::squared(rowm(X_pv, 0)) + dlib::squared(rowm(X_pv, 1));

    // dlib::matrix<double, 1, 0> r;
    // r.set_size(1, num_pairs);
    // r = dlib::sqrt(r2);
    
    dlib::matrix<double, 1, 0> r4;
    r4.set_size(1, num_pairs);
    r4 = dlib::squared(r2);

    dlib::matrix<double, 0, 0> d;

    d = dlib::ones_matrix<double>(1, num_pairs) + k1*r2 + k2*r4;
    // d = dlib::zeros_matrix<double>(1, num_pairs);
    // d = dlib::ones_matrix<double>(1, num_pairs);

    set_rowm(X_pv, 0) = pointwise_multiply(rowm(X_pv, 0), d);
    set_rowm(X_pv, 1) = pointwise_multiply(rowm(X_pv, 1), d);

    dlib::matrix<double, 0, 0> imagePoints;
    imagePoints.set_size(3, num_pairs);
    imagePoints = K * X_pv;
    
    dlib::matrix<double, 0, 0> uv = 2*pointwise_multiply(rowm(imagePoints, 0), rowm(imagePoints, 1));

    set_rowm(imagePoints, 0) = rowm(imagePoints, 0) + p1*uv + p2*(r2 + dlib::squared(rowm(imagePoints, 0)));
    set_rowm(imagePoints, 1) = rowm(imagePoints, 1) + p2*uv + p1*(r2 + dlib::squared(rowm(imagePoints, 1)));
    
    // set_rowm(imagePoints, 0) = pointwise_multiply(rowm(imagePoints, 0), 1.0 / rowm(imagePoints, 2));
    // set_rowm(imagePoints, 1) = pointwise_multiply(rowm(imagePoints, 1), 1.0 / rowm(imagePoints, 2));

    for (int i = 0; i < num_pairs; i++)
    {
        // projected[i*2] = K(0, 2) + (imagePoints(0, i) - K(0, 2))*d(0, i);
        // projected[(i*2)+1] = K(1, 2) + (imagePoints(1, i) - K(1, 2))*d(0, i);
        projected[i*2] = imagePoints(0, i);
        projected[(i*2)+1] = imagePoints(1, i);
    }
}

void levmar_proj_func(double *p, double *hx, int m, int n, void *adata)
{
    int num_pairs = n / 2;

    dlib::matrix<double, 0, 0> pairsKinect;
    pairsKinect.set_size(num_pairs, 4);

    double *kinectPairsRawData = reinterpret_cast<double*>(adata);

    project_points(p, m, hx, num_pairs, kinectPairsRawData);
}

