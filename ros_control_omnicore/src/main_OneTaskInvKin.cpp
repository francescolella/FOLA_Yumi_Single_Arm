/***********************************************************************************************************************
Copyright (c) 2021, JOiiNT LAB, Fondazione Istituto Italiano di Tecnologia, Intellimech Consorzio per la Meccatronica.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***********************************************************************************************************************
*
***********************************************************************************************************************
* 
* Authors: Gianluca Lentini, Ugo Alberto Simioni
* Date:18/01/2022
* Version 1.0
***********************************************************************************************************************
*/

#include <ros/ros.h>
#include <ros/rate.h>

// my headers
#include "../include/OneTaskInvKin.h"


//-----------------------------------------------------
//                                                 main
//-----------------------------------------------------
int main(int argc, char **argv)
{

  ros::init(argc, argv, "OneTaskInvKin_node");

  OneTaskInvKin Obj;
  double rate_200Hz = 200.0;
  ros::Rate r_200HZ(rate_200Hz);
  Obj.dt_ = 1.0/rate_200Hz;


  while(ros::ok())
  {
    Obj.run();
    ros::spinOnce();
    r_200HZ.sleep();
        
  }// end while()
return 0;
}
