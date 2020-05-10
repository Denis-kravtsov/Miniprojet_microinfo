#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H
/*
*	gives the line position in pixels (0 to 640)
*/
uint16_t get_line_position(void);
/*
*	Initiates the threads needed to process the image
*/
void process_image_start(void);
/*
*	tells if the rising edge was detected
*/
bool get_rising_edge(void);
/*
*	tells if the falling edge was detected
*/
bool get_falling_edge(void);
/*
*	sets the colour to detect
*/
void set_color(int color);
/*
*	tells which colour is being detected
*/
int get_colour(void);


#endif /* PROCESS_IMAGE_H */
