//
//  AppDelegate.m
//  FreeFlight
//
//  Created by Frédéric D'HAEYER on 16/10/09.
//  Copyright Parrot SA 2009. All rights reserved.
//
#import "AppDelegate.h"
#import "EAGLView.h"
#import "MenuUpdater.h"
#import "ES1Renderer.h"

@implementation AppDelegate
@synthesize window;
@synthesize menuController;

- (void) applicationDidFinishLaunching:(UIApplication *)application
{
	NSLog(@"app did finish launching");
	application.idleTimerDisabled = YES;

	// Setup the menu controller
	menuController.delegate = self;
	NSLog(@"menu controller view %@", menuController.view);
	
	was_in_game = NO;
	
	// Setup the ARDrone
	ARDroneHUDConfiguration hudconfiguration = {YES, NO, YES};
	drone = [[ARDrone alloc] initWithFrame:menuController.view.frame withState:was_in_game withDelegate:nil withHUDConfiguration:&hudconfiguration];
	
	// Setup the OpenGL view
	glView = [[EAGLView alloc] initWithFrame:menuController.view.frame];
	[glView setDrone:drone];
	[glView changeState:was_in_game];

	[menuController.view addSubview:drone.view];
	[menuController changeState:was_in_game];

	[window addSubview:menuController.view];
	[window addSubview:glView];
	[window bringSubviewToFront:menuController.view];
 	[window makeKeyAndVisible];
}

#pragma mark -
#pragma mark Drone protocol implementation
- (void)changeState:(BOOL)inGame
{
	was_in_game = inGame;
	
	if(inGame)
	{
		[drone setScreenOrientationRight:(menuController.interfaceOrientation == UIInterfaceOrientationLandscapeRight)];
#ifdef DISPLAY_DART
		[drone executeCommandIn:ARDRONE_COMMAND_VIDEO_CHANNEL withParameter:(void *)ARDRONE_VIDEO_CHANNEL_HORI fromSender:nil];
		[drone executeCommandIn:ARDRONE_COMMAND_CAMERA_DETECTION withParameter:(void *)ARDRONE_CAMERA_DETECTION_H_ORIENTED_COCARDE fromSender:nil];
#else
		[drone executeCommandIn:ARDRONE_COMMAND_CAMERA_DETECTION withParameter:(void *)ARDRONE_CAMERA_DETECTION_NONE fromSender:nil];
#endif
		[drone executeCommandIn:ARDRONE_COMMAND_ENABLE_COMBINED_YAW withParameter:(void*)NO fromSender:nil];
	}
	
	[drone changeState:inGame];
	[glView changeState:inGame];
}

- (void) applicationWillResignActive:(UIApplication *)application
{
	// Become inactive
	if(was_in_game)
	{
		[drone changeState:NO];
		[glView changeState:NO];
	}
	else
	{
		[menuController changeState:NO];
	}
}

- (void) applicationDidBecomeActive:(UIApplication *)application
{
	if(was_in_game)
	{
		[drone changeState:YES];
		[glView changeState:YES];
	}
}

- (void)applicationWillTerminate:(UIApplication *)application
{
	printf("%s : %d\n", __FUNCTION__, was_in_game);
	
	if(was_in_game)
	{
		[drone changeState:NO];
		[glView changeState:NO];
	}
	else
	{
		[menuController changeState:NO];
	}
}

- (void)executeCommandIn:(ARDRONE_COMMAND_IN)commandId withParameter:(void*)parameter fromSender:(id)sender
{
	
}

- (BOOL)checkState
{
	BOOL result = NO;
	
	if(was_in_game)
	{
		result = [drone checkState];
	}
	else
	{
		//result = [menuController checkState];
	}
	
	return result;
}

@end
