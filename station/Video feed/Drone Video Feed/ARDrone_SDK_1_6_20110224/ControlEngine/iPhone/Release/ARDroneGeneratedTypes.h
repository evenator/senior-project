// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// !!!! THIS FILE IS GENERATED AUTOMATICALLY, DO NOT CHANGE IT !!!!
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
/*
 *  ARDroneGeneratedTypes.h
 *  ARDroneEngine
 *
 *  Automatically generated.
 *  Copyright 2010 Parrot SA. All rights reserved.
 *
 */
#ifndef _ARDRONE_GENERATED_TYPES_H_
#define _ARDRONE_GENERATED_TYPES_H_

#define ARDRONE_MAX_ENEMIES 4

typedef enum {
ARDRONE_LED_ANIMATION_BLINK_GREEN_RED,
ARDRONE_LED_ANIMATION_BLINK_GREEN,
ARDRONE_LED_ANIMATION_BLINK_RED,
ARDRONE_LED_ANIMATION_BLINK_ORANGE,
ARDRONE_LED_ANIMATION_SNAKE_GREEN_RED,
ARDRONE_LED_ANIMATION_FIRE,
ARDRONE_LED_ANIMATION_STANDARD,
ARDRONE_LED_ANIMATION_RED,
ARDRONE_LED_ANIMATION_GREEN,
ARDRONE_LED_ANIMATION_RED_SNAKE,
ARDRONE_LED_ANIMATION_BLANK,
ARDRONE_LED_ANIMATION_RIGHT_MISSILE,
ARDRONE_LED_ANIMATION_LEFT_MISSILE,
ARDRONE_LED_ANIMATION_DOUBLE_MISSILE,
ARDRONE_LED_ANIMATION_FRONT_LEFT_GREEN_OTHERS_RED,
ARDRONE_LED_ANIMATION_FRONT_RIGHT_GREEN_OTHERS_RED,
ARDRONE_LED_ANIMATION_REAR_RIGHT_GREEN_OTHERS_RED,
ARDRONE_LED_ANIMATION_REAR_LEFT_GREEN_OTHERS_RED,
ARDRONE_LED_ANIMATION_LEFT_GREEN_RIGHT_RED,
ARDRONE_LED_ANIMATION_LEFT_RED_RIGHT_GREEN,
ARDRONE_LED_ANIMATION_BLINK_STANDARD,
} ARDRONE_LED_ANIMATION;

typedef enum {
ARDRONE_ANIMATION_PHI_M30_DEG=0,
ARDRONE_ANIMATION_PHI_30_DEG,
ARDRONE_ANIMATION_THETA_M30_DEG,
ARDRONE_ANIMATION_THETA_30_DEG,
ARDRONE_ANIMATION_THETA_20DEG_YAW_200DEG,
ARDRONE_ANIMATION_THETA_20DEG_YAW_M200DEG,
ARDRONE_ANIMATION_TURNAROUND,
ARDRONE_ANIMATION_TURNAROUND_GODOWN,
ARDRONE_ANIMATION_YAW_SHAKE,
ARDRONE_ANIMATION_YAW_DANCE,
ARDRONE_ANIMATION_PHI_DANCE,
ARDRONE_ANIMATION_THETA_DANCE,
ARDRONE_ANIMATION_VZ_DANCE,
ARDRONE_ANIMATION_WAVE,
ARDRONE_ANIMATION_PHI_THETA_MIXED,
ARDRONE_ANIMATION_DOUBLE_PHI_THETA_MIXED,
} ARDRONE_ANIMATION;

typedef enum {
ARDRONE_CAMERA_DETECTION_HORIZONTAL=0,
ARDRONE_CAMERA_DETECTION_VERTICAL,
ARDRONE_CAMERA_DETECTION_VISION,
ARDRONE_CAMERA_DETECTION_NONE,
ARDRONE_CAMERA_DETECTION_COCARDE,
ARDRONE_CAMERA_DETECTION_ORIENTED_COCARDE,
ARDRONE_CAMERA_DETECTION_STRIPE,
ARDRONE_CAMERA_DETECTION_H_COCARDE,
ARDRONE_CAMERA_DETECTION_H_ORIENTED_COCARDE,
ARDRONE_CAMERA_DETECTION_STRIPE_V,
ARDRONE_CAMERA_DETECTION_MULTIPLE_DETECTION_MODE,
ARDRONE_CAMERA_DETECTION_NUM,
} ARDRONE_CAMERA_DETECTION_TYPE;

typedef enum {
ARDRONE_VIDEO_CHANNEL_FIRST=0,
ARDRONE_VIDEO_CHANNEL_HORI=ARDRONE_VIDEO_CHANNEL_FIRST,
ARDRONE_VIDEO_CHANNEL_VERT,
ARDRONE_VIDEO_CHANNEL_LARGE_HORI_SMALL_VERT,
ARDRONE_VIDEO_CHANNEL_LARGE_VERT_SMALL_HORI,
ARDRONE_VIDEO_CHANNEL_LAST=ARDRONE_VIDEO_CHANNEL_LARGE_VERT_SMALL_HORI,
ARDRONE_VIDEO_CHANNEL_NEXT,
} ARDRONE_VIDEO_CHANNEL;

typedef enum {
ARDRONE_ENEMY_COLOR_ORANGE_GREEN=1,
ARDRONE_ENEMY_COLOR_ORANGE_YELLOW,
ARDRONE_ENEMY_COLOR_ORANGE_BLUE
} ARDRONE_ENEMY_COLOR;
#endif // _ARDRONE_GENERATED_TYPES_H_
