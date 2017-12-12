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
    // A.set_size(nPairs*2, 11);
    // y.set_size(nPairs*2, 1);
    
    // for (int i=0; i<nPairs; i++) {
    //     A(2*i, 0) = pairsKinect[i].x;
    //     A(2*i, 1) = pairsKinect[i].y;
    //     A(2*i, 2) = pairsKinect[i].z;
    //     A(2*i, 3) = 1;
    //     A(2*i, 4) = 0;
    //     A(2*i, 5) = 0;
    //     A(2*i, 6) = 0;
    //     A(2*i, 7) = 0;
    //     A(2*i, 8) = -pairsKinect[i].x * pairsProjector[i].x;
    //     A(2*i, 9) = -pairsKinect[i].y * pairsProjector[i].x;
    //     A(2*i, 10) = -pairsKinect[i].z * pairsProjector[i].x;
        
    //     A(2*i+1, 0) = 0;
    //     A(2*i+1, 1) = 0;
    //     A(2*i+1, 2) = 0;
    //     A(2*i+1, 3) = 0;
    //     A(2*i+1, 4) = pairsKinect[i].x;
    //     A(2*i+1, 5) = pairsKinect[i].y;
    //     A(2*i+1, 6) = pairsKinect[i].z;
    //     A(2*i+1, 7) = 1;
    //     A(2*i+1, 8) = -pairsKinect[i].x * pairsProjector[i].y;
    //     A(2*i+1, 9) = -pairsKinect[i].y * pairsProjector[i].y;
    //     A(2*i+1, 10) = -pairsKinect[i].z * pairsProjector[i].y;
        
    //     y(2*i, 0) = pairsProjector[i].x;
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

    projMatrice = calibrate_svd(pairsKinect, pairsProjector, r);

    cout << "SVD error = " << ComputeReprojectionError(projMatrice, pairsKinect, pairsProjector) << endl;
    calibrated = true;
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


