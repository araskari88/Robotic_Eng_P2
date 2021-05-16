#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
// void drive_robot(float lin_x, float ang_z, int img_width, int img_height, int n_pixel, float avg_ball_location)
void drive_robot(float lin_x, float ang_z)
{
    // Request a service and pass the velocities to it to drive the robot
    // ROS_INFO("Image Width: %d, Image Length: %d, Pixels Seen: %d, Location: %1.3f", img_width, img_height, n_pixel, avg_ball_location);

    // Request centered joint angles [1.57, 1.57]
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

    // Call the ball_chaser service and pass the requested joint angles
    if (!client.call(srv))
        ROS_ERROR("Failed to call service ball_chaser");
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
    float lin_x = 0.0;
    float ang_z = 0.0;

    int white_pixel = 255;  // value for a white pixel (the color of our ball)
    int img_width = img.width;  // width of the image
    // int img_height = img.width;
    int side_limit = int(img_width / 3);  // 1/3rd of left and right regions of the image are considerd as image sides
    int mid_limit = int(img_width / 2);  // middle point in the image
    int left_limit = side_limit;  // the pixel limit of the left region of the image
    int right_limit = img_width - side_limit;  // the pixel limit of the right region of the image
    
    int n_ball_pixel_seen = 0;  // number of pixels of the balls in the robot view
    float avg_ball_location = 0;  // average location of the ball
    // float avg_ball_row = 0;
    bool ball_seen = false;  // boolean to identify if the ball is in the robot's field of view

    // Loop through each pixel in the image and check if there's a white pixel in it (we have a ball)
    for (int i = 0; i < img.height * img.step; i++)
    {
        if (img.data[i] == white_pixel)
        {   
            n_ball_pixel_seen += 1;  // increment up the number of pixels of the ball in the robot's view
            avg_ball_location += i % img_width;
            // avg_ball_row += floor(i / img_width);
        }
    }
    if (n_ball_pixel_seen > 10)
    {
        ball_seen = true;  // ball exists in the view of the robot
        avg_ball_location /= n_ball_pixel_seen;  // calculate the average position of the ball
        // avg_ball_row /= n_ball_pixel_seen;
    }
    else
    {
        ball_seen = false;
        avg_ball_location = 0;
    }

    // Can the robot see the ball?
    if (ball_seen == true)  // yes
    {
        // a linear function defines the speed of the robot rotating to the left and right ranging from 0.1 to 0.5 depending on the location of the point
        if (avg_ball_location < left_limit)
        {
            ang_z = 0.1 + 0.4 * (left_limit - avg_ball_location) / side_limit;  // rotate to the left
            lin_x = 0;  // no movement forward
        }
        else if (avg_ball_location > right_limit)
        {
            ang_z = -0.1 - 0.4 * (avg_ball_location - right_limit) / side_limit;  // rotate to the right
            lin_x = 0;  // no movement forward
        }
        else
        {
            if (avg_ball_location > mid_limit)
            {
                ang_z = -0.1 * (avg_ball_location - mid_limit) / (side_limit / 2);  // small rotation to the right
                lin_x = 0.4 - 0.3 * (avg_ball_location - mid_limit) / (side_limit / 2);  // moderate move forward
            }
            else if (avg_ball_location < mid_limit)
            {
                ang_z = 0.1 * (mid_limit - avg_ball_location) / (side_limit / 2);  // small rotation to the left
                lin_x = 0.4 - 0.3 * (mid_limit - avg_ball_location) / (side_limit / 2);  // moderate move forward
            }
            else
            {
                ang_z = 0.0;  // no rotation
                lin_x = 0.5;  // fast move forward
            }
        }
	    // ROS_INFO_STREAM("seen " + std::to_string(n_ball_pixel_seen) + " pixels at " + std::to_string(avg_ball_location) 
        // + " side and at " + std::to_string(avg_ball_row) + " row");
    }
    else
    {
        // no ball seen ... no movement needed
        lin_x = 0;
        ang_z = 0;
    }
    // drive_robot(lin_x, ang_z, img_width, img_height, n_ball_pixel_seen, avg_ball_location);  // lin_x and ang_z == 0 -> request stop of the robot else move the robot accordingly
    drive_robot(lin_x, ang_z);  // lin_x and ang_z == 0 -> request stop of the robot else move the robot accordingly
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}