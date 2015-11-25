/*
 * IKFast Demo
 * 
 * Shows how to calculate FK from joint angles.
 * Calculates IK from rotation-translation matrix, or translation-quaternion pose.
 * Performance timing tests.
 *
 * Run the program to view command line parameters.
 * 
 * 
 * To compile, run:
 * g++ -lstdc++ -llapack -o compute ikfastdemo.cpp -lrt
 * (need to link with 'rt' for gettime(), it must come after the source file name)
 *
 * 
 * Tested with Ubuntu 11.10 (Oneiric)
 * IKFast54 from OpenRAVE 0.6.0
 * IKFast56/61 from OpenRave 0.8.2
 *
 * Author: David Butterworth, KAIST
 *         Based on code by Rosen Diankov
 * Date: November 2012
 */

/*
 * Copyright (c) 2012, David Butterworth, KAIST
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#define IKFAST_HAS_LIBRARY // Build IKFast with API functions
#define IKFAST_NO_MAIN // Don't include main() from IKFast

/*
Set which IKFast version you are using
The API calls are slightly different for versions > 54
*/

#define IK_VERSION 61
#include "output_ikfast61.cpp"
//#include "ikfast61.Transform6D.0_1_2_3_4_5.cpp"

//#define IK_VERSION 56
//#include "ikfast56.Transform6D.0_1_2_3_4_5.cpp"

//#define IK_VERSION 54
//#include "output_ikfast54.cpp"


//----------------------------------------------------------------------------//

#include <stdio.h>
#include <stdlib.h>
#include <time.h> // for clock_gettime()

float SIGN(float x);
float NORM(float a, float b, float c, float d);

#if IK_VERSION > 54
#define IKREAL_TYPE IkReal // for IKFast 56,61
#else
#define IKREAL_TYPE IKReal // for IKFast 54
#endif

int main(int argc, char** argv)
{
    IKREAL_TYPE eerot[9],eetrans[3];

#if IK_VERSION > 54
    // for IKFast 56,61
    unsigned int num_of_joints = GetNumJoints();
    unsigned int num_free_parameters = GetNumFreeParameters();
#else
    // for IKFast 54
    unsigned int num_of_joints = getNumJoints();
    unsigned int num_free_parameters = getNumFreeParameters();
#endif
 
    std::string cmd;
    if (argv[1]) cmd = argv[1];

    //printf("command: %s \n\n", cmd.c_str() );

    if (cmd.compare("ik") == 0) // ik mode
    {
        if( argc == 1+7+num_free_parameters+1 )  // ik, given translation vector and quaternion pose
        {
#if IK_VERSION > 54
            // for IKFast 56,61
            IkSolutionList<IKREAL_TYPE> solutions;
#else
            // for IKFast 54
            std::vector<IKSolution> vsolutions;
#endif
            std::vector<IKREAL_TYPE> vfree(num_free_parameters);

            eetrans[0] = atof(argv[2]);
            eetrans[1] = atof(argv[3]);
            eetrans[2] = atof(argv[4]);

            // Convert input effector pose, in w x y z quaternion notation, to rotation matrix. 
            // Must use doubles, else lose precision compared to directly inputting the rotation matrix.
            double qw = atof(argv[5]);
            double qx = atof(argv[6]);
            double qy = atof(argv[7]);
            double qz = atof(argv[8]);
            const double n = 1.0f/sqrt(qx*qx+qy*qy+qz*qz+qw*qw);
            qw *= n;
            qx *= n;
            qy *= n;
            qz *= n;
            eerot[0] = 1.0f - 2.0f*qy*qy - 2.0f*qz*qz;  eerot[1] = 2.0f*qx*qy - 2.0f*qz*qw;         eerot[2] = 2.0f*qx*qz + 2.0f*qy*qw;
            eerot[3] = 2.0f*qx*qy + 2.0f*qz*qw;         eerot[4] = 1.0f - 2.0f*qx*qx - 2.0f*qz*qz;  eerot[5] = 2.0f*qy*qz - 2.0f*qx*qw;
            eerot[6] = 2.0f*qx*qz - 2.0f*qy*qw;         eerot[7] = 2.0f*qy*qz + 2.0f*qx*qw;         eerot[8] = 1.0f - 2.0f*qx*qx - 2.0f*qy*qy;

            // For debugging, output the matrix
            /*
            for (unsigned char i=0; i<=8; i++)
            {   // detect -0.0 and replace with 0.0
                if ( ((int&)(eerot[i]) & 0xFFFFFFFF) == 0) eerot[i] = 0.0;
            }
            printf("     Rotation     %f   %f   %f  \n", eerot[0], eerot[1], eerot[2] );
            printf("                  %f   %f   %f  \n", eerot[3], eerot[4], eerot[5] );
            printf("                  %f   %f   %f  \n", eerot[6], eerot[7], eerot[8] );
            printf("\n");
            */

            for(std::size_t i = 0; i < vfree.size(); ++i)
                vfree[i] = atof(argv[13+i]);

#if IK_VERSION > 54
            // for IKFast 56,61
            bool bSuccess = ComputeIk(eetrans, eerot, vfree.size() > 0 ? &vfree[0] : NULL, solutions);
#else
            // for IKFast 54
            bool bSuccess = ik(eetrans, eerot, vfree.size() > 0 ? &vfree[0] : NULL, vsolutions);
#endif
            if( !bSuccess ) {
                fprintf(stderr,"Failed to get ik solution\n");
                return -1;
            }

#if IK_VERSION > 54
            // for IKFast 56,61
            unsigned int num_of_solutions = (int)solutions.GetNumSolutions();
#else
            // for IKFast 54
            unsigned int num_of_solutions = (int)vsolutions.size();
#endif
            printf("Found %d ik solutions:\n", num_of_solutions ); 

            std::vector<IKREAL_TYPE> solvalues(num_of_joints);
            for(std::size_t i = 0; i < num_of_solutions; ++i) {
#if IK_VERSION > 54
                // for IKFast 56,61
                const IkSolutionBase<IKREAL_TYPE>& sol = solutions.GetSolution(i);
                int this_sol_free_params = (int)sol.GetFree().size(); 
#else
                // for IKFast 54
                int this_sol_free_params = (int)vsolutions[i].GetFree().size();
#endif
                printf("sol%d (free=%d): ", (int)i, this_sol_free_params );
                std::vector<IKREAL_TYPE> vsolfree(this_sol_free_params);

#if IK_VERSION > 54
                // for IKFast 56,61
                sol.GetSolution(&solvalues[0],vsolfree.size()>0?&vsolfree[0]:NULL);
#else
                // for IKFast 54
                vsolutions[i].GetSolution(&solvalues[0],vsolfree.size()>0?&vsolfree[0]:NULL);
#endif
                for( std::size_t j = 0; j < solvalues.size(); ++j)
                    printf("%.15f, ", solvalues[j]);
                printf("\n");
            }

        } 
        else if( argc == 1+12+num_free_parameters+1 )  // ik, given rotation-translation matrix
        {
#if IK_VERSION > 54
            // for IKFast 56,61
            IkSolutionList<IKREAL_TYPE> solutions;
#else
            // for IKFast 54
            std::vector<IKSolution> vsolutions;
#endif
            std::vector<IKREAL_TYPE> vfree(num_free_parameters);

            eerot[0] = atof(argv[2]);  eerot[1] = atof(argv[3]);  eerot[2] = atof(argv[4]);  eetrans[0] = atof(argv[5]);
            eerot[3] = atof(argv[6]);  eerot[4] = atof(argv[7]);  eerot[5] = atof(argv[8]);  eetrans[1] = atof(argv[9]);
            eerot[6] = atof(argv[10]); eerot[7] = atof(argv[11]); eerot[8] = atof(argv[12]); eetrans[2] = atof(argv[13]);
            for(std::size_t i = 0; i < vfree.size(); ++i)
                vfree[i] = atof(argv[13+i]);

#if IK_VERSION > 54
            // for IKFast 56,61
            bool bSuccess = ComputeIk(eetrans, eerot, vfree.size() > 0 ? &vfree[0] : NULL, solutions);
#else
            // for IKFast 54
            bool bSuccess = ik(eetrans, eerot, vfree.size() > 0 ? &vfree[0] : NULL, vsolutions);
#endif
            if( !bSuccess ) {
                fprintf(stderr,"Failed to get ik solution\n");
                return -1;
            }

#if IK_VERSION > 54
            // for IKFast 56,61
            unsigned int num_of_solutions = (int)solutions.GetNumSolutions();
#else
            // for IKFast 54
            unsigned int num_of_solutions = (int)vsolutions.size();
#endif
            printf("Found %d ik solutions:\n", num_of_solutions ); 

            std::vector<IKREAL_TYPE> solvalues(num_of_joints);
            for(std::size_t i = 0; i < num_of_solutions; ++i) {
#if IK_VERSION > 54
                // for IKFast 56,61
                const IkSolutionBase<IKREAL_TYPE>& sol = solutions.GetSolution(i);
                int this_sol_free_params = (int)sol.GetFree().size(); 
#else
                // for IKFast 54
                int this_sol_free_params = (int)vsolutions[i].GetFree().size();
#endif
                printf("sol%d (free=%d): ", (int)i, this_sol_free_params );
                std::vector<IKREAL_TYPE> vsolfree(this_sol_free_params);

#if IK_VERSION > 54
                // for IKFast 56,61
                sol.GetSolution(&solvalues[0],vsolfree.size()>0?&vsolfree[0]:NULL);
#else
                // for IKFast 54
                vsolutions[i].GetSolution(&solvalues[0],vsolfree.size()>0?&vsolfree[0]:NULL);
#endif
                for( std::size_t j = 0; j < solvalues.size(); ++j)
                    printf("%.15f, ", solvalues[j]);
                printf("\n");
            }

        }
        else {
            printf("\n "
                   "Usage: \n\n " 
                   "         ./compute ik  t0 t1 t2  qw qi qj qk  free0 ...\n\n "
                   "         Returns the ik solutions given the transformation of the end effector specified by \n "
                   "         a 3x1 translation (tX), and a 1x4 quaternion (w + i + j + k). \n "
                   "         There are %d free parameters that have to be specified.\n\n", num_free_parameters );
            printf(" \n "
                   "         ./compute ik  r00 r01 r02 t0  r10 r11 r12 t1  r20 r21 r22 t2  free0 ...\n\n "
                   "         Returns the ik solutions given the transformation of the end effector specified by \n "
                   "         a 3x3 rotation R (rXX), and a 3x1 translation (tX). \n "
                   "         There are %d free parameters that have to be specified.\n\n", num_free_parameters );
            return 1;

        }
    } // endif ik mode

    else if (cmd.compare("fk") == 0) // fk mode
    {
        if( argc != num_of_joints+2 ) {
            printf("\n "
                   "Usage: \n\n "
                   "         ./compute fk j0 j1 ... j%d \n\n"
                   "         Returns the forward kinematic solution given the joint angles (in radians). \n\n", num_of_joints-1 );
            return 1;
        }
        printf("\n\n");

        // Put input joint values into array
        IKREAL_TYPE joints[num_of_joints];
        for (unsigned int i=0; i<num_of_joints; i++)
        {
            joints[i] = atof(argv[i+2]);
        }

#if IK_VERSION > 54
        // for IKFast 56,61
        ComputeFk(joints, eetrans, eerot); // void return
#else
        // for IKFast 54
        fk(joints, eetrans, eerot); // void return
#endif
        printf("Found fk solution for end frame: \n\n");

        printf("  Translation:  x: %f  y: %f  z: %f  \n", eetrans[0], eetrans[1], eetrans[2] );
        printf("\n");
        printf("     Rotation     %f   %f   %f  \n", eerot[0], eerot[1], eerot[2] );
        printf("       Matrix:    %f   %f   %f  \n", eerot[3], eerot[4], eerot[5] );
        printf("                  %f   %f   %f  \n", eerot[6], eerot[7], eerot[8] );
        printf("\n");

        // Display equivalent Euler angles
        float yaw;
        float pitch;
        float roll;
        if ( eerot[5] > 0.998 || eerot[5] < -0.998 ) { // singularity
            yaw = IKatan2( -eerot[6], eerot[0] );
            pitch = 0;
        } else {
            yaw = IKatan2( eerot[2], eerot[8] );
            pitch = IKatan2( eerot[3], eerot[4] );
        }
        roll = IKasin( eerot[5] );
        printf(" Euler angles: \n");
        printf("       Yaw:   %f    ", yaw ); printf("(1st: rotation around vertical blue Z-axis in ROS Rviz) \n");
        printf("       Pitch: %f  \n", pitch ); 
        printf("       Roll:  %f  \n", roll ); 
        printf("\n");

        // Convert rotation matrix to quaternion (Daisuke Miyazaki)
        float q0 = ( eerot[0] + eerot[4] + eerot[8] + 1.0f) / 4.0f;
        float q1 = ( eerot[0] - eerot[4] - eerot[8] + 1.0f) / 4.0f;
        float q2 = (-eerot[0] + eerot[4] - eerot[8] + 1.0f) / 4.0f;
        float q3 = (-eerot[0] - eerot[4] + eerot[8] + 1.0f) / 4.0f;
        if(q0 < 0.0f) q0 = 0.0f;
        if(q1 < 0.0f) q1 = 0.0f;
        if(q2 < 0.0f) q2 = 0.0f;
        if(q3 < 0.0f) q3 = 0.0f;
        q0 = sqrt(q0);
        q1 = sqrt(q1);
        q2 = sqrt(q2);
        q3 = sqrt(q3);
        if(q0 >= q1 && q0 >= q2 && q0 >= q3) {
            q0 *= +1.0f;
            q1 *= SIGN(eerot[7] - eerot[5]);
            q2 *= SIGN(eerot[2] - eerot[6]);
            q3 *= SIGN(eerot[3] - eerot[1]);
        } else if(q1 >= q0 && q1 >= q2 && q1 >= q3) {
            q0 *= SIGN(eerot[7] - eerot[5]);
            q1 *= +1.0f;
            q2 *= SIGN(eerot[3] + eerot[1]);
            q3 *= SIGN(eerot[2] + eerot[6]);
        } else if(q2 >= q0 && q2 >= q1 && q2 >= q3) {
            q0 *= SIGN(eerot[2] - eerot[6]);
            q1 *= SIGN(eerot[3] + eerot[1]);
            q2 *= +1.0f;
            q3 *= SIGN(eerot[7] + eerot[5]);
        } else if(q3 >= q0 && q3 >= q1 && q3 >= q2) {
            q0 *= SIGN(eerot[3] - eerot[1]);
            q1 *= SIGN(eerot[6] + eerot[2]);
            q2 *= SIGN(eerot[7] + eerot[5]);
            q3 *= +1.0f;
        } else {
            printf("Error while converting to quaternion! \n");
        }
        float r = NORM(q0, q1, q2, q3);
        q0 /= r;
        q1 /= r;
        q2 /= r;
        q3 /= r;
        printf("  Quaternion:  %f   %f   %f   %f   \n", q0, q1, q2, q3 );
        printf("               ");
        // print quaternion with convention and +/- signs such that it can be copy-pasted into WolframAlpha.com
        printf("%f ", q0);
        if (q1 > 0) printf("+ %fi ", q1); else if (q1 < 0) printf("- %fi ", -q1); else printf("+ 0.00000i ");
        if (q2 > 0) printf("+ %fj ", q2); else if (q2 < 0) printf("- %fj ", -q2); else printf("+ 0.00000j ");
        if (q3 > 0) printf("+ %fk ", q3); else if (q3 < 0) printf("- %fk ", -q3); else printf("+ 0.00000k ");
        printf("  (alternate convention) \n");
        printf("\n\n");


    }
    else if (cmd.compare("iktiming") == 0) // generate random ik and check time performance
    {
        if( argc != 2 ) {
            printf("\n "
                   "Usage: \n\n "
                   "         ./compute iktiming \n\n"
                   "         For fixed number of iterations, generates random joint angles, then  \n"
                   "         calculates fk, calculates ik, measures average time taken. \n\n", num_of_joints-1 );
            return 1;
        }
        printf("\n\n");

#if IK_VERSION > 54
        // for IKFast 56,61
        IkSolutionList<IKREAL_TYPE> solutions;
#else
        // for IKFast 54
        std::vector<IKSolution> vsolutions;
#endif
        std::vector<IKREAL_TYPE> vfree(num_free_parameters);

        //for(std::size_t i = 0; i < vfree.size(); ++i)
        //    vfree[i] = atof(argv[13+i]);

        srand( (unsigned)time(0) ); // seed random number generator
        float min = -3.14;
        float max = 3.14;
        float rnd;

        IKREAL_TYPE joints[num_of_joints];

        timespec start_time, end_time;
        unsigned int elapsed_time = 0;
        unsigned int sum_time = 0;

#if IK_VERSION > 54
        // for IKFast 56,61
        unsigned int num_of_tests = 1000000; 
#else
        // for IKFast 54
        unsigned int num_of_tests = 100000; 
#endif

        for (unsigned int i=0; i < num_of_tests; i++)
        {
            // Measure avg time for whole process
            //clock_gettime(CLOCK_REALTIME, &start_time); 

            // Put random joint values into array
            for (unsigned int i=0; i<num_of_joints; i++)
            {
                float rnd = (float)rand() / (float)RAND_MAX;
                joints[i] = min + rnd * (max - min);
            }
            /*
            printf("Joint angles:  ");
            for (unsigned int i=0; i<num_of_joints; i++)
            {
                printf("%f  ", joints[i] );
            }
            printf("\n");
            */

#if IK_VERSION > 54
            // for IKFast 56,61
            ComputeFk(joints, eetrans, eerot); // void return
#else
            // for IKFast 54
            fk(joints, eetrans, eerot); // void return
#endif

            // Measure avg time for IK
            clock_gettime(CLOCK_REALTIME, &start_time);
#if IK_VERSION > 54
            // for IKFast 56,61
            ComputeIk(eetrans, eerot, vfree.size() > 0 ? &vfree[0] : NULL, solutions);
#else
            // for IKFast 54
            ik(eetrans, eerot, vfree.size() > 0 ? &vfree[0] : NULL, vsolutions);
#endif

            /*
#if IK_VERSION > 54
            // for IKFast 56,61
            unsigned int num_of_solutions = (int)solutions.GetNumSolutions();
#else
            // for IKFast 54
            unsigned int num_of_solutions = (int)vsolutions.size();
#endif
            printf("Found %d ik solutions:\n", num_of_solutions ); 
            */

            clock_gettime(CLOCK_REALTIME, &end_time); 
            elapsed_time = (unsigned int)(end_time.tv_nsec - start_time.tv_nsec);
            sum_time += elapsed_time;
        } // endfor

        unsigned int avg_time = (unsigned int)sum_time / (unsigned int)num_of_tests; 
        printf("avg time: %f ms   over %d tests \n", (float)avg_time/1000.0, num_of_tests );

        return 1;
  
    }
    else if (cmd.compare("iktiming2") == 0) // for fixed joint values, check time performance of ik
    {
        if( argc != 2 ) {
            printf("\n "
                   "Usage: \n\n "
                   "         ./compute iktiming2 \n\n"
                   "         For fixed number of iterations, with one set of joint variables, this  \n"
                   "         finds the ik solutions and measures the average time taken. \n\n", num_of_joints-1 );
            return 1;
        }
        printf("\n\n");

#if IK_VERSION > 54
        // for IKFast 56,61
        IkSolutionList<IKREAL_TYPE> solutions;
#else
        // for IKFast 54
        std::vector<IKSolution> vsolutions;
#endif
        std::vector<IKREAL_TYPE> vfree(num_free_parameters);

        //for(std::size_t i = 0; i < vfree.size(); ++i)
        //    vfree[i] = atof(argv[13+i]);

        IKREAL_TYPE joints[num_of_joints];

        timespec start_time, end_time;
        unsigned int elapsed_time = 0;
        unsigned int sum_time = 0;

#if IK_VERSION > 54
        // for IKFast 56,61
        unsigned int num_of_tests = 1000000; 
#else
        // for IKFast 54
        unsigned int num_of_tests = 100000; 
#endif

        // fixed rotation-translation matrix corresponding to an unusual robot pose
        eerot[0] = 0.002569;  eerot[1] = -0.658044;  eerot[2] = -0.752975;  eetrans[0] = 0.121937;
        eerot[3] = 0.001347;  eerot[4] = -0.752975;  eerot[5] = 0.658048;  eetrans[1] = -0.276022;
        eerot[6] = -0.999996; eerot[7] = -0.002705; eerot[8] = -0.001047; eetrans[2] = 0.005685;

        for (unsigned int i=0; i < num_of_tests; i++)
        {
            clock_gettime(CLOCK_REALTIME, &start_time);

#if IK_VERSION > 54
            // for IKFast 56,61
            ComputeIk(eetrans, eerot, vfree.size() > 0 ? &vfree[0] : NULL, solutions);
#else
            // for IKFast 54
            ik(eetrans, eerot, vfree.size() > 0 ? &vfree[0] : NULL, vsolutions);
#endif

            /*
#if IK_VERSION > 54
            // for IKFast 56,61
            unsigned int num_of_solutions = (int)solutions.GetNumSolutions();
#else
            // for IKFast 54
            unsigned int num_of_solutions = (int)vsolutions.size();
#endif
            printf("Found %d ik solutions:\n", num_of_solutions ); 
            */

            clock_gettime(CLOCK_REALTIME, &end_time); 
            elapsed_time = (unsigned int)(end_time.tv_nsec - start_time.tv_nsec);
            sum_time += elapsed_time;
        } // endfor

        unsigned int avg_time = (unsigned int)sum_time / (unsigned int)num_of_tests; 
        printf("avg time: %f ms   over %d tests \n", (float)avg_time/1000.0, num_of_tests );

        return 1;

    } else {
        printf("\n"
               "Usage: \n\n");
        printf("         ./compute fk j0 j1 ... j%d \n\n"
               "         Returns the forward kinematic solution given the joint angles (in radians). \n\n", num_of_joints-1 );
        printf("\n"
               "         ./compute ik  t0 t1 t2  qw qi qj qk  free0 ... \n\n"
               "         Returns the ik solutions given the transformation of the end effector specified by \n"
               "         a 3x1 translation (tX), and a 1x4 quaternion (w + i + j + k). \n"
               "         There are %d free parameters that have to be specified. \n\n", num_free_parameters );
        printf(" \n"
               "         ./compute ik  r00 r01 r02 t0  r10 r11 r12 t1  r20 r21 r22 t2  free0 ...\n\n"
               "         Returns the ik solutions given the transformation of the end effector specified by \n"
               "         a 3x3 rotation R (rXX), and a 3x1 translation (tX). \n"
               "         There are %d free parameters that have to be specified. \n\n", num_free_parameters );
        printf("\n"
               "         ./compute iktiming \n\n"
               "         For fixed number of iterations, generates random joint angles, then \n"
               "         calculates fk, calculates ik, measures average time taken. \n\n", num_of_joints-1 );
        printf("\n"
               "         ./compute iktiming2 \n\n"
               "         For fixed number of iterations, with one set of joint variables, this \n"
               "         finds the ik solutions and measures the average time taken. \n\n", num_of_joints-1 );

        return 1;
    }

    return 0;
}

float SIGN(float x) {
    return (x >= 0.0f) ? +1.0f : -1.0f;
}

float NORM(float a, float b, float c, float d) {
    return sqrt(a * a + b * b + c * c + d * d);
}


