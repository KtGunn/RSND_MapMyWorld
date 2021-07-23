
#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Client
ros::ServiceClient g_Client;

// Send drive commands to the drive_bot node
void drive_bot (float vel_x, float rate_z, const bool nodrive=true) {

    // Throttle the messaging to console
    static int throttle = 0;
    static int squawk = 30;
    if (throttle++ >= squawk) {
	ROS_INFO_STREAM ("Driving to target ");
	throttle = 0;
    }
    
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = vel_x;    
    srv.request.angular_z = rate_z;

    // nodrive is used todebug the ball finding algorithm
    if (!nodrive && !g_Client.call (srv)) {
	ROS_ERROR ( "process_image failed to deliver svcReq to drive_bot!");
    }
    return;
}


void image_callback (const sensor_msgs::Image image) {

    // Throttle the messaging to console
    static int throttle = 0;
    static int squawk = 20;

    // Value of pixel defining the ball we seek
    const static int white_pixel_value = 255;
    
    // State variables
    typedef enum {outofview=0, inview} enLooking;   // Ball is either 
						    // in view or not
    typedef enum {full=0, partial, none} enViewStatus; // We either
						       // see the
						       // whole ball
						       // or not
    typedef enum {nowhite, white, done} enState;    // As image is
						    // scanned we see,
						    // then don't see
						    // white pixels
    typedef enum {seeking=0, chasing} enAction; // We're either
						// seeking or chasing
						// the ball 
    static enAction action = chasing;



    // Image variables
    uint height = image.height;
    uint width = image.width;
    uint step = image.step;
    uint channels = step/width; // RGB is three channels

    uint horizontal_image_center = width / 2; // we'll seeek to place
					      // the ball at this point
    uint horizontal_ball_position = -1;
    
    if (throttle >= squawk) {
	ROS_INFO ("\n Image! w%d x h%d / step%d size%d", width, height, step, height*step );
    }
    
    ///////////////////////////////////////////////////////////////////////////////////////
    /// IMAGE scanning section
    //
    int aVert[2] = {-1,-1};
    int aWide[2] = {1000,-1000};
    enLooking viewState = outofview;
    enViewStatus ballView = none;
    
    for (int row=0; row < height; row++) {
	
	enState state = nowhite;
	int aHor[2] = {-1,-1};

	for (int hor=0; hor < step ; hor += channels) {
	    switch (state) {
	    case nowhite :
		// Looking for the switch TO white
		if (image.data[row*step + hor] == white_pixel_value &&
		    image.data[row*step + hor +1] == white_pixel_value &&
		    image.data[row*step + hor +2] == white_pixel_value) {
		    
		    state = white;
		    aHor[0] = hor/3;
		}
		break;
	    case white :
		// Looking for the switch FROM white
		if (image.data[row*step + hor] != white_pixel_value &&
		    image.data[row*step + hor +1] != white_pixel_value &&
		    image.data[row*step + hor +2] != white_pixel_value) {

		    aHor[1] = (hor-3)/3;
		    state = done;
		} 
		break;
	    }
	    if (state == done) {
		break;
	    }
	}

	if (state == white) {
	    // We went over the right edge
	    aHor[1] = (step-3)/3;
	    state = done;
	}
	if (state == done) {

	    if (aWide[0] > aHor[0]) {
		aWide[0] = aHor[0];
	    }
	    if (aWide[1] < aHor[1]) {
		aWide[1] = aHor[1];
	    }
	}
	
	if (viewState == outofview) {
	    if (aHor[0] > -1) {
		aVert[0] = row;
		viewState = inview;
	    }
	} else if (viewState == inview) {
	    // If ball was in view but no more, we bail out
	    if (aHor[0] < 0) {
		aVert[1] = row-1;
		break;
	    }
	}
    }
    
    static char* aState[2] = {"out_of_view", "in_view"};
    static char* aOp[3] = {"seeking", "chasing"};
    if (throttle++ >= squawk) {
	ROS_INFO (" VertRange %d %d (%d) :: %s", aVert[0], aVert[1], aVert[1]-aVert[0], aState[(int)viewState]);
	ROS_INFO ("  HorRange %d %d (%d) :: %s", aWide[0], aWide[1], aWide[1]-aWide[0], aOp[(int)action]);
	throttle = 0;
    }

    ///////////////////////////////////////////////////////////////////////////////////////
    /// STATUS of image scanning section
    //
    float horizontal_to_vertical_ratio;
    int offset_from_center;
    if ( viewState == inview ) {
	horizontal_to_vertical_ratio = ((float)(aWide[1]-aWide[0]))/(aVert[1]-aVert[0]);
	if ( abs (1-horizontal_to_vertical_ratio) < 0.100 ) {
	    ballView = full;
	} else {
	    ballView = partial;
	}

	/// Capture the situation where robot is up against the ball!
	//  aHor = {0, -1} indicates ball filling out the entire
	//  vertical view resulting in a very large ratio
	if ( abs (horizontal_to_vertical_ratio) > 10) {
	    ballView = full;
	}

	int horizontal_ball_position = 0.5*(aWide[0]+aWide[1]);
	offset_from_center = horizontal_image_center - horizontal_ball_position;
	if (throttle == 0) {
	    ROS_INFO ("  ratio H2V = %.3f, center offset = %d", horizontal_to_vertical_ratio, offset_from_center);
	}
    }


    ///////////////////////////////////////////////////////////////////////////////////////
    /// OPERATIONS
    //
    static const float seek_vel = 0.0;
    static const float seek_rate = 0.25;

    enAction priorAction = action;

    /// Always seek if ball is out of view
    if ( viewState == outofview ) {
	if (action != seeking) {
	    ROS_INFO ("  Seeking - 1" );
	    drive_bot (seek_vel, seek_rate, false);
	    action = seeking;
	}
	if (priorAction != action ) {
	  ROS_INFO ( "1 ACT %s  %s", aOp[priorAction], aOp[action]);
	}
	return; // Nothing to do until ball comes into view
    }

    /// If ball not in FULL view, keep seeking
    if (ballView != full && action != seeking) {
	ROS_INFO ("  Seeking - 2" );
	drive_bot (seek_vel, seek_rate, false);
	action = seeking;
	if (priorAction != action ) {
	  ROS_INFO ( "2 ACT %s  %s : %.3f", aOp[priorAction], aOp[action], horizontal_to_vertical_ratio);
	}
	return; // Nothing to do until ball is in full view
    }
    
    if (ballView == full && action == seeking) {
	// Stop the robot
	drive_bot (0.0, 0.0, false);
	action = chasing;
	if (priorAction != action ) {
	  ROS_INFO ( " 3 ACT %s  %s : %.3f", aOp[priorAction], aOp[action], horizontal_to_vertical_ratio);
	}
	return;
    }
    
    static const float chase_vel = 0.5;
    static const float rate_gain = 0.015; // Proportional control gain
    if (ballView == full) {
	// Chase the robot @ const speed but proportional rate
	float rate = 1 * (float)(rate_gain*offset_from_center);
	if (throttle == 0) {
	    ROS_INFO ("  rate = %.3f", rate);
	}
	float rate_limit = 2.5;
	if ( rate > rate_limit ) { rate = rate_limit; } else if (rate < -rate_limit) { rate = -rate_limit;}
	drive_bot (chase_vel, rate, false);
	action = chasing;
	if (priorAction != action ) {
	  ROS_INFO ( "4 ACT %s  %s", aOp[priorAction], aOp[action]);
	}
	return;
    }

    return;
}

int main (int argc, char* argv[]) {

    // Boiler plate
    ros::init (argc, argv, "process_image");
    ros::NodeHandle handle;

    ROS_INFO ( " starting process_image ..." );

    // We'll be requesting moves
    g_Client = handle.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    if (true) {
	// Wait for ROS start
	int start_time;
	while (not start_time) {
	    start_time = ros::Time::now().toSec() - start_time;
	}
    }

    // We'll subsrice to camera images
    int qsize = 10;
    ros::Subscriber sub = handle.subscribe ( "/camera/rgb/image_raw", qsize, image_callback);

    
    ROS_INFO (" ... process_image is in the loop! " );

    // Into the events loop
    ros::spin();

    return (0);
}

