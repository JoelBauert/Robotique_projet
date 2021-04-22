#ifndef DISTANCE_H
#define DISTANCE_H

#define FR_17 0 // front right 17'
#define FR_49 1 // front right 49'
#define RIGHT 2 // right
#define BR    3 // back right
#define BL    4 // back left
#define LEFT  5 // left
#define FL_49 6 // front left 49'
#define FL_17 7 // front right 17'

#define THRESHOLD_DIST 200

uint8_t get_stop(void);

void distance_start(void);
//void find_distance(void);

#endif
