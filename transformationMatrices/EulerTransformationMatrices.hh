//------------------------------------------------------------------------
//
//  Copyright (C) 2010, 2012 Manuel Wopfner, Matthias Lutz
//
//        lutz@hs-ulm.de
//
//        Christian Schlegel (schlegel@hs-ulm.de)
//        University of Applied Sciences
//        Prittwitzstr. 10
//        89075 Ulm (Germany)
//
//  This file contains functions which creates the corresponding 
//  rotation matrixes for different euler angles
//
//  This library is free software; you can redistribute it and/or
//  modify it under the terms of the GNU Lesser General Public
//  License as published by the Free Software Foundation; either
//  version 2.1 of the License, or (at your option) any later version.
//
//  This library is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//  Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public
//  License along with this library; if not, write to the Free Software
//  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//--------------------------------------------------------------------------
//  Code for function angle_axis_to_euler_angels_ZYX is covered by the BSD License
//--------------------------------------------------------------------------





#ifndef EULER_TANSFORMATION_MATRICES
#define EULER_TANSFORMATION_MATRICES

#include <armadillo.hh>

#if defined (__GNUC__) && defined(__unix__)

#elif defined (WIN32)
#define _USE_MATH_DEFINES
#include <math.h>
#endif


class EulerTransformationMatrices {
public:
	static void normalize_angle_axis(double x, double y, double z, double& norm, double& nx, double& ny, double& nz)
	{
		norm = sqrt(z*z + y*y + x*x); //calculate 2 norm (length) of angle axis vector
		nx = x/norm; //normalize the vector
                ny = y/norm;
                nz = z/norm;
	}

	static void denormalize_angle_axis(double x, double y, double z, double norm, double& dnx, double& dny, double& dnz)
	{
		dnx = x*norm;
                dny = y*norm;
                dnz = z*norm;
	}

	static bool euler_angels_ZYX_to_angle_axis(double yaw, double pitch, double roll, double& x,double& y,double& z,double& angle){

		//Code taken from John Fuller:
                //Code covered by the BSD License
                //http://www.mathworks.com/matlabcentral/fileexchange/27653-euler-angle-dcm-quaternion-and-euler-vector-conversionteaching-gui
                //and ported to c++


                //Copyright (c) 2010, John Fuller
                //Copyright (c) 2006, Douglas M. Schwarz
                //Copyright (c) 2011, John Fuller
                //All rights reserved.
                //
                //Redistribution and use in source and binary forms, with or without 
                //modification, are permitted provided that the following conditions are 
                //met:
                //
                //    * Redistributions of source code must retain the above copyright 
                //      notice, this list of conditions and the following disclaimer.
                //    * Redistributions in binary form must reproduce the above copyright 
                //      notice, this list of conditions and the following disclaimer in 
                //      the documentation and/or other materials provided with the distribution
                //      
                //THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
                //AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
                //IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
                //ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
                //LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
                //CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
                //SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
                //INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
                //CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
                //ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
                //POSSIBILITY OF SUCH DAMAGE.

                //more info about angleEulerTransformations under
                //http://www.euclideanspace.com/maths/geometry/rotations/conversions/angleToEuler/index.htm

		if(fabs(pitch)>=M_PI/2)
		{
	                std::cerr<<"[EulerTransformationMatrices::euler_angels_ZYX_to_angle_axis] ERROR: \n"
			    <<"Second input Euler angle outside -90 to 90 degree range"<<std::endl;
			return false;
		}
		else if(fabs(pitch)>1.535889) //88deg
		{
			std::cerr<<"[EulerTransformationMatrices::euler_angels_ZYX_to_angle_axis] WARNING: \n"
				<<" Input Euler angle rotation near a singularity. Second angle near -90 or 90 degrees.";
		}
			

		double c1=cos(yaw/2.0);
                double c2=cos(pitch/2.0); 
                double c3=cos(roll/2.0);
		double s1=sin(yaw/2.0); 
		double s2=sin(pitch/2.0); 
		double s3=sin(roll/2.0);

		arma::mat Q(1,4); //convert angle axis to quaternion

	        Q(0,0) = c1*c2*s3-s1*s2*c3;
	        Q(0,1) = c1*s2*c3+s1*c2*s3;
	        Q(0,2) = s1*c2*c3-c1*s2*s3;
	        Q(0,3) = c1*c2*c3+s1*s2*s3;
	
	        //Normalize quaternions in case of deviation from unity.
	        double Qnorms = sqrt(Q(0,0)*Q(0,0) + Q(0,1)*Q(0,1) + Q(0,2)*Q(0,2) + Q(0,3)*Q(0,3));
	        Q(0,0) = Q(0,0)/Qnorms;
	        Q(0,1) = Q(0,1)/Qnorms;
	        Q(0,2) = Q(0,2)/Qnorms;
	        Q(0,3) = Q(0,3)/Qnorms;


		double MU=2*atan2(sqrt(pow(Q(0,0),2) + pow(Q(0,1),2) + pow(Q(0,2),2)),Q(0,3));

		if(sin(MU/2)!=0)
		{
 			x = Q(0,0)/sin(MU/2);
 			y = Q(0,1)/sin(MU/2);
 			z = Q(0,2)/sin(MU/2);
			angle = MU;

		}	
		else
		{
	                std::cerr<<"[EulerTransformationMatrices::euler_angels_ZYX_to_angle_axis] WARNING: \n"
			    <<"UNTESTED CASE USED!"<<std::endl;
			x = 1;
			y = 0;
			z = 0;
			angle = MU;
		}

		return true;
	}



	static bool angle_axis_to_euler_angels_ZYX(double x,double y,double z,double angle, double& yaw, double& pitch, double& roll){
		//Code taken from John Fuller:
		//Code covered by the BSD License
		//http://www.mathworks.com/matlabcentral/fileexchange/27653-euler-angle-dcm-quaternion-and-euler-vector-conversionteaching-gui
		//and ported to c++


		//Copyright (c) 2010, John Fuller
		//Copyright (c) 2006, Douglas M. Schwarz
		//Copyright (c) 2011, John Fuller
		//All rights reserved.
		//
		//Redistribution and use in source and binary forms, with or without 
		//modification, are permitted provided that the following conditions are 
		//met:
		//
		//    * Redistributions of source code must retain the above copyright 
		//      notice, this list of conditions and the following disclaimer.
		//    * Redistributions in binary form must reproduce the above copyright 
		//      notice, this list of conditions and the following disclaimer in 
		//      the documentation and/or other materials provided with the distribution
		//      
		//THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
		//AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
		//IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
		//ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE 
		//LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
		//CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
		//SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
		//INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
		//CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
		//ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
		//POSSIBILITY OF SUCH DAMAGE.

		//more info about angleEulerTransformations under
		//http://www.euclideanspace.com/maths/geometry/rotations/conversions/angleToEuler/index.htm

		double tol =0.001;

	        if(sqrt(pow(x,2)+pow(y,2)+pow(z,2))-1>tol*1) //check that input vector constitute a unit vector
		{
	           std::cerr<<"[EulerTransformationMatrices::angle_axis_to_euler_angels_ZYX] ERROR: \n"
			    <<"Input euler vector(s) components do not constitute a unit vector!"<<std::endl;
		   return false;
		}

		if(angle<0 || angle > 2*M_PI) //check if rotation about euler vector is between 0 and 360
		{
		   std::cerr<<"[EulerTransformationMatrices::angle_axis_to_euler_angels_ZYX] ERROR: \n"
			    <<"Input euler rotation angle(s) not between 0 and 360 degrees!"<<std::endl;
		   return false;
		}
	
		arma::mat Q(1,4); //convert angle axis to quaternion
	        Q(0,0) = x * sin(angle/2);
	        Q(0,1) = y * sin(angle/2);
	        Q(0,2) = z * sin(angle/2);
	        Q(0,3) = cos(angle/2);
	
	        //Normalize quaternions in case of deviation from unity.
	        double Qnorms = sqrt(Q(0,0)*Q(0,0) + Q(0,1)*Q(0,1) + Q(0,2)*Q(0,2) + Q(0,3)*Q(0,3));
	        Q(0,0) = Q(0,0)/Qnorms;
	        Q(0,1) = Q(0,1)/Qnorms;
	        Q(0,2) = Q(0,2)/Qnorms;
	        Q(0,3) = Q(0,3)/Qnorms;
	
	
	        double psi=atan2( 2*(Q(0,0)*Q(0,1) + Q(0,2)*Q(0,3)) , (pow(Q(0,3),2)+pow(Q(0,0),2)-pow(Q(0,1),2)-pow(Q(0,2),2))) ;
	        double theta=asin(2*(Q(0,1)*Q(0,3)-Q(0,0)*Q(0,2)));
	        double phi=atan2(2*(Q(0,0)*Q(0,3)+Q(0,2)*Q(0,1)) , (pow(Q(0,3),2)-pow(Q(0,0),2)-pow(Q(0,1),2)+pow(Q(0,2),2)));
	
	//      if(isreal([psi,theta,phi]))==0,
	//                 error('Error: Unreal Euler output.  Input resides too close to singularity.  Please choose different output type.')
	//      	end
	
	        //OUTPUT=mod([psi,theta,phi]*180/pi,360);  %deg
	
		//      sing_chk=find(abs(theta)*180/pi>89.9);
		//      sing_chk=sort(sing_chk(sing_chk>0));
		//      if size(sing_chk,1)>=1,

		if((abs(theta)>1.5690)) //89.9 deg
		{
		   std::cerr<<"[EulerTransformationMatrices::angle_axis_to_euler_angels_ZYX] ERROR:\n Input rotation resides too close to Type 1 Euler singularity.\n"
			    <<"Type 1 Euler singularity occurs when second angle is -90 or 90 degrees.\n"<<std::endl;
		   return false;	
		}


        	yaw = psi;
	        pitch = theta;
	        roll = phi;
		return true;

	}


	/**
	 * This method creates an rotation matrix for the zxz euler angle.
	 * The matrix size should be set at least to 3x3, otherwise the matrix will be
	 * automatically set to 3x3.
	 */
	static void create_zxz_matrix(double phi, double theta, double psi, arma::mat& matrix) {
		double c_phi = cos(phi);
		double s_phi = sin(phi);
		double c_theta = cos(theta);
		double s_theta = sin(theta);
		double c_psi = cos(psi);
		double s_psi = sin(psi);

		// If matrix is to small resize it
		if (matrix.n_cols < 3 || matrix.n_rows < 3)
			matrix.set_size(3, 3);

		matrix(0, 0) = c_phi * c_psi - s_phi * c_theta * s_psi;
		matrix(0, 1) = -c_phi * s_psi - s_phi * c_theta * c_psi;
		matrix(0, 2) = s_phi * s_theta;

		matrix(1, 0) = s_phi * c_psi + c_phi * c_theta * s_psi;
		matrix(1, 1) = -s_phi * s_psi + c_phi * c_theta * c_psi;
		matrix(1, 2) = -c_phi * s_theta;

		matrix(2, 0) = s_theta * s_psi;
		matrix(2, 1) = s_theta * c_psi;
		matrix(2, 2) = c_theta;

	}

	/**
	 * This method creates an homogenous matrix for the zxz euler angle and a translation.
	 * The matrix size should be set at least to 4x4, otherwise the matrix will be
	 * automatically set to 4x4.
	 */
	static void create_zxz_matrix(double x, double y, double z, double phi, double theta, double psi, arma::mat& matrix) {
		double c_phi = cos(phi);
		double s_phi = sin(phi);
		double c_theta = cos(theta);
		double s_theta = sin(theta);
		double c_psi = cos(psi);
		double s_psi = sin(psi);

		// If matrix is to small resize it
		if (matrix.n_cols < 4 || matrix.n_rows < 4) {
			matrix.set_size(4, 4);
			matrix.zeros();
		}

		matrix(0, 0) = c_phi * c_psi - s_phi * c_theta * s_psi;
		matrix(0, 1) = -c_phi * s_psi - s_phi * c_theta * c_psi;
		matrix(0, 2) = s_phi * s_theta;
		matrix(0, 3) = x;

		matrix(1, 0) = s_phi * c_psi + c_phi * c_theta * s_psi;
		matrix(1, 1) = -s_phi * s_psi + c_phi * c_theta * c_psi;
		matrix(1, 2) = -c_phi * s_theta;
		matrix(1, 3) = y;

		matrix(2, 0) = s_theta * s_psi;
		matrix(2, 1) = s_theta * c_psi;
		matrix(2, 2) = c_theta;
		matrix(2, 3) = z;

		matrix(3, 3) = 1;

	}

	/**
	 * This method creates an rotation matrix for the zyx euler angle.
	 * The matrix size should be set at least to 3x3, otherwise the matrix will be
	 * automatically set to 3x3.
	 */
	static void create_zyx_matrix(double phi, double theta, double psi, arma::mat& matrix) {
		double c_phi = cos(phi);
		double s_phi = sin(phi);
		double c_theta = cos(theta);
		double s_theta = sin(theta);
		double c_psi = cos(psi);
		double s_psi = sin(psi);

		// If matrix is to small resize it
		if (matrix.n_cols < 3 || matrix.n_rows < 3)
			matrix.set_size(3, 3);

		matrix(0, 0) = c_theta * c_phi;
		matrix(0, 1) = -c_psi * s_phi + s_psi * s_theta * c_phi;
		matrix(0, 2) = s_psi * s_phi + c_psi * s_theta * c_phi;

		matrix(1, 0) = c_theta * s_phi;
		matrix(1, 1) = c_psi * c_phi + s_psi * s_theta * s_phi;
		matrix(1, 2) = -s_psi * c_phi + c_psi * s_theta * s_phi;

		matrix(2, 0) = -s_theta;
		matrix(2, 1) = s_psi * c_theta;
		matrix(2, 2) = c_psi * c_theta;
	}

	/**
	 * This method creates an homogenous matrix for the zyx euler angle and a translation.
	 * The matrix size should be set at least to 4x4, otherwise the matrix will be
	 * automatically set to 4x4.
	 */
	static void create_zyx_matrix(double x, double y, double z, double phi, double theta, double psi, arma::mat& matrix) {
		double c_phi = cos(phi);
		double s_phi = sin(phi);
		double c_theta = cos(theta);
		double s_theta = sin(theta);
		double c_psi = cos(psi);
		double s_psi = sin(psi);

		// If matrix is to small resize it
		if (matrix.n_cols < 4 || matrix.n_rows < 4) {
			matrix.set_size(4, 4);
			matrix.zeros();
		}
		matrix(0, 0) = c_theta * c_phi;
		matrix(0, 1) = -c_psi * s_phi + s_psi * s_theta * c_phi;
		matrix(0, 2) = s_psi * s_phi + c_psi * s_theta * c_phi;
		matrix(0, 3) = x;

		matrix(1, 0) = c_theta * s_phi;
		matrix(1, 1) = c_psi * c_phi + s_psi * s_theta * s_phi;
		matrix(1, 2) = -s_psi * c_phi + c_psi * s_theta * s_phi;
		matrix(1, 3) = y;

		matrix(2, 0) = -s_theta;
		matrix(2, 1) = s_psi * c_theta;
		matrix(2, 2) = c_psi * c_theta;
		matrix(2, 3) = z;

		matrix(3, 3) = 1;
	}

    static void zxz_to_zyx_angles(const double in_a, const double in_e, const double in_r, double& out_a, double& out_e, double& out_r) {
        // ZXZ rotation matrix
		arma::mat r(3, 3);
		EulerTransformationMatrices::create_zxz_matrix(in_a, in_e, in_r, r);

		// elevation is in the range [-pi/2, pi/2 ], so it is enough to calculate:
		out_e = atan2(-r(2, 0), hypot(r(0, 0), r(1, 0)));

		// roll:
		if (fabs(r(2, 1)) + fabs(r(2, 2)) < 10 * std::numeric_limits<double>::epsilon()) {
			out_r = 0.0;
			if (out_e > 0)
				out_a = -atan2(-r(1, 2), r(0, 2));
			else
				out_a = atan2(-r(1, 2), -r(0, 2));
		} else {
			out_r = atan2(r(2, 1), r(2, 2));
			out_a = atan2(r(1, 0), r(0, 0));
		}
    }    

    static void zyx_to_zxz_angles(const double in_a, const double in_e, const double in_r, double& out_a, double& out_e, double& out_r) {
        // ZYX rotation matrix
		arma::mat r(3, 3);
		EulerTransformationMatrices::create_zyx_matrix(in_a, in_e, in_r, r);

		out_e = acos(r(2, 2));

		if (fabs(sin(out_e)) > 1E-8) {
			out_r = atan2(r(2, 0) / sin(out_e), r(2, 1) / sin(out_e));
			out_a = -atan2(r(0, 2) / sin(out_e), r(1, 2) / sin(out_e)) + M_PI;
		} else {
			out_a = 0;
			out_r = atan2(r(1, 0), r(1, 1)) + M_PI;
		}
    } 

    static void zyx_from_matrix(const arma::mat& matrix, double& out_x, double& out_y, double& out_z, double& out_azimuth, double& out_elevation, double& out_roll) {
        double azimuth, elevation, roll;

		// elevation is in the range [-pi/2, pi/2 ], so it is enough to calculate:
		elevation = atan2(-matrix(2, 0), hypot(matrix(0, 0), matrix(1, 0)));

		// roll:
		if ((fabs(matrix(2, 1)) + fabs(matrix(2, 2))) < 10 * std::numeric_limits<double>::epsilon()) {
			roll = 0.0;
			if (elevation > 0)
				azimuth = -atan2(-matrix(1, 2), matrix(0, 2));
			else
				azimuth = atan2(-matrix(1, 2), -matrix(0, 2));
		} else {
			roll = atan2(matrix(2, 1), matrix(2, 2));
			azimuth = atan2(matrix(1, 0), matrix(0, 0));
		}

		out_x = matrix(0, 3);
		out_y = matrix(1, 3);
		out_z = matrix(2, 3);
		out_azimuth = azimuth;
		out_elevation = elevation;
		out_roll = roll;
    }

    static void zyx_from_matrix(const arma::mat& matrix, double& out_azimuth, double& out_elevation, double& out_roll) {
        double azimuth, elevation, roll;

		// elevation is in the range [-pi/2, pi/2 ], so it is enough to calculate:
		elevation = atan2(-matrix(2, 0), hypot(matrix(0, 0), matrix(1, 0)));

		// roll:
		if ((fabs(matrix(2, 1)) + fabs(matrix(2, 2))) < 10 * std::numeric_limits<double>::epsilon()) {
			roll = 0.0;
			if (elevation > 0)
				azimuth = -atan2(-matrix(1, 2), matrix(0, 2));
			else
				azimuth = atan2(-matrix(1, 2), -matrix(0, 2));
		} else {
			roll = atan2(matrix(2, 1), matrix(2, 2));
			azimuth = atan2(matrix(1, 0), matrix(0, 0));
		}

		out_azimuth = azimuth;
		out_elevation = elevation;
		out_roll = roll;
    }

};

#endif
