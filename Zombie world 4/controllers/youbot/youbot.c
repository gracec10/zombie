/*
 * Copyright 1996-2019 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Description:   Starts with a predefined behaviors and then
 *                read the user keyboard inputs to actuate the
 *                robot
 */
 


#include <webots/keyboard.h>            
#include <webots/robot.h>
#include <webots/supervisor.h>

#include <arm.h>
#include <base.h>
#include <gripper.h>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h> 

#include <webots/camera.h>
#include <webots/accelerometer.h>
#include <webots/lidar.h>
#include <webots/compass.h>
#include <webots/gps.h>
#include <webots/range_finder.h>
#include <webots/gyro.h>
#include <webots/light_sensor.h>
#include <webots/receiver.h>
#include <webots/distance_sensor.h>

#include <youbot_zombie_1.h>


int robot_angle = 0;
#define TIME_STEP 32


//////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////// ONLY USE THE FOLLOWING FUNCTIONS TO MOVE THE ROBOT /////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////


void stop()
{
	base_reset();
}

void go_forward()
{
	base_forwards();
}

void go_backward()
{
	base_backwards();
}

void turn_left()
{
	base_turn_left();
	robot_angle = robot_angle + 90;
	if (robot_angle == 360)
		robot_angle = 0;

}

void turn_right()
{
	base_turn_right();	
	robot_angle = robot_angle - 90;
	if (robot_angle == -90)
		robot_angle = 270;
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////// CHANGE CODE BELOW HERE ONLY ////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////


/*Robot control reading in the pixels from the camera and tells us which color berry is in front of us*/
int robot_control()
{ 

	////////////// TO GET RGB FROM THE CAMERA ///////////////////////////////////////////////////
	const unsigned char *image = wb_camera_get_image(8);
	int width = wb_camera_get_width(8);
	int height = wb_camera_get_height(8);
	
	for (int x = width/3; x < 2*width/3; x++)
	{
		for (int y = 1*height/10; y < 5*height/10; y++) 
		{
			int r = wb_camera_image_get_red(image, width, x, y);
			int g = wb_camera_image_get_green(image, width, x, y);
			int b = wb_camera_image_get_blue(image, width, x, y);
			
			/* This returns a number that'll be used to index an array.
                      Each berry is assigned a number - Orange is 0,... and null is 4*/
        		if ((r > 170) && (g > 100) && (b < 110)&& (g < 160) && (b > 60)){
                		printf("orange berry");
                		return 0;
        		}
                		
                	else if ((r > 190) && (g < 80) && (b < 70)){
                		printf("red berry");
                		return 1;
        		}
                	else if ((r > 170) && (g > 100) && (b > 150) && (g < 160) && (b < 200)){
                		printf("pink berry");
                		return 2;
        		}
                	else if ((r > 170) && (g > 160) && (b < 60)){
                		printf("yellow berry");
                		return 3;
        		}
                  	else{
                      	return 4;
                  	}
                      	
                	
		}
	}
	/////////////////////////////////////////////////////////////////////////////////////////////
	return 4;
}

/*Will teach the robot between good and bad berries by updating the array each time it eats*/
int berry_learning[] = {0,0,0,0};


//////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////// CHANGE CODE ABOVE HERE ONLY ////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////


int main(int argc, char **argv) 
{

  struct Robot robot_info = {100,100};
  wb_robot_init();
  base_init();
  arm_init();
  gripper_init();
  passive_wait(0.1);

  //display_helper_message();

  int pc = 0;
  wb_keyboard_enable(TIME_STEP);
  int timer = 0;
  
  WbNodeRef robot_node = wb_supervisor_node_get_from_def("Youbot");
  WbFieldRef trans_field = wb_supervisor_node_get_field(robot_node, "translation");
  
  get_all_berry_pos();
  
  int robot_not_dead = 1;
   
  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////// CHANGE CODE BELOW HERE ONLY ////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////
    
  // wb_accelerometer_enable(1,1);
  // wb_gps_enable(2,TIME_STEP);
  // wb_compass_enable(3,TIME_STEP);
  // wb_camera_enable(4,TIME_STEP);
  // wb_camera_enable(5,TIME_STEP);
  // wb_camera_enable(6,TIME_STEP);
  // wb_camera_enable(7,TIME_STEP);
  wb_camera_enable(8,TIME_STEP);
  // wb_camera_enable(9,TIME_STEP);
  // wb_camera_enable(10,TIME_STEP);
  // wb_camera_enable(11,TIME_STEP);
  // wb_gyro_enable(12,TIME_STEP);
  // wb_light_sensor_enable(13,TIME_STEP);
  // wb_receiver_enable(14,TIME_STEP);
  // wb_range_finder_enable(15,TIME_STEP);
  wb_lidar_enable(16,1); 
  
  WbDeviceTag lidar = wb_robot_get_device("lidar");
  wb_lidar_enable_point_cloud(lidar);

  //WbDeviceTag rec = wb_robot_get_device("receiver");
 
  int i = 0;
  int current_berry;
  int current_energy;
  int berryFlag = 0;
  bool escape = false;

  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////// CHANGE CODE ABOVE HERE ONLY ////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////
  while (robot_not_dead == 1) 
  {

	if (robot_info.health < 0)
    {
		robot_not_dead = 0;
		printf("ROBOT IS OUT OF HEALTH\n");
	}
	
	if (timer % 2 == 0)
	{  
		const double *trans = wb_supervisor_field_get_sf_vec3f(trans_field);
		check_berry_collision(&robot_info, trans[0], trans[2]);
		check_zombie_collision(&robot_info, trans[0], trans[2]);
		//printf("%f\n", trans[0]);
	}
    if (timer == 16)
    {
        update_robot(&robot_info);
        timer = 0;
    }

    step();


    
    int c = keyboard(pc);
    pc = c;
    timer=timer+1;
    
    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////// CHANGE CODE BELOW HERE ONLY ////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    
    // this is called everytime step.

    if (i < 300)
    {
       //base_turn_left();
    	//base_forwards();
    	go_backward();
    	printf("O%d\n",i);
    }
    if (i == 300)
    {
    	// base_reset();
    	//base_turn_left();
    	go_backward();
    }
    if (i == 400)
    {
    	i = 0;
    }
    
    
    int berry = robot_control();
    /*If a berry is detected by the camera then it enters the state that
     updates the learning array and eats or navigates around berries if they are harmful*/
    if ((berry != 4)&& (berryFlag == 0)) {
      i = 0;
      berryFlag = 1;
    }
    
    if (berryFlag == 1)
    {
      printf("STEP%d\n", i);
      base_reset();
      if ((berry == 0)||(berry == 1)||(berry == 2)||(berry == 3)) {
        current_berry = berry;
        current_energy = robot_info.energy;  
      }
      
      
      if (berry_learning[current_berry] > -1) {
        go_backward();
      }
      else { //jump to escape
        //escape the berry
        escape = true;
        base_reset();
      } 
    
    // after eating
      if ((i == 2)&&(escape == false)){
        if (robot_info.energy < (current_energy - 5)) {
          berry_learning[current_berry] = berry_learning[current_berry] - 1;
        }
        else {
           berry_learning[current_berry] = berry_learning[current_berry] - 1;
        }
        printf("back%d", berry_learning[current_berry]);
        berryFlag = 0;
        i = 0;
      }

      if ((i <= 3)&&(escape == true)) {
        printf("back up%d", i);
        go_forward();
      }
      
      if ((i>3)&&(i<100)){
        base_turn_left();
      }
      if (i==100){
        berryFlag = 0;
        i = 0;
        escape = false;
      }
    }
    
    
    
    i++;


    // if (wb_receiver_get_queue_length(rec) > 0) 
  	// {
  		// const char *buffer = wb_receiver_get_data(rec);
        // printf("Communicating: received \"%s\"\n", buffer);
    	// wb_receiver_next_packet(rec);
    // }

    
    //////////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////// CHANGE CODE ABOVE HERE ONLY ////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////
  }

  wb_robot_cleanup();

  return 0;
}
