/*
 * Queue.h
 *
 *  Created on: Feb 19, 2023
 *      Author: Deepshikha, Mayand
 */

#ifndef _QUEUE_H
#define _QUEUE_H

/*
class Queue {
	int rear, front, size;
	int *arr;
	public:
		Queue(int s){
		front = rear = -1;
		size = s;
		arr = new int[s];
		for(int i=0; i<size; i++) arr[i] = 0;
		}
		
		void enQueue(int value);
		int deQueue();
		int* getQueue();
};
*/

class Queue {
	int rear, front, length, size;
	float avg;
	float avg_entire;
	float *arr;
	public:
		Queue(int s){
		front = rear = -1;
		length = s;
		size = 0;
		arr = new float[s];
		avg = 0.0;
		avg_entire = 0.0;
		for(int i=0; i<size; i++) arr[i] = 0.0;
		}
		
		float enQueue(float value);
		float deQueue();
		float* getQueue();
		float avg_arr();
		float avg_arr_entire();		//produce wrong output
		int get_first_index();
		int get_length();
		int get_size();
		int last();
		int second_last();
};

#endif