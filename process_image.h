#ifndef PROCESS_IMAGE_H
#define PROCESS_IMAGE_H


float get_distance_cm(void);
uint16_t get_line_position(void);
void process_image_start(void);
void colour_status(bool status);
bool get_rising_edge(void);
bool get_falling_edge(void);
bool get_colour_status(void);
uint16_t get_line_width(void);
#endif /* PROCESS_IMAGE_H */
