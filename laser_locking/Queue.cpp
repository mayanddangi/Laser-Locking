#include "Queue.h"

float Queue::enQueue(float value){
	if((front == 0 && rear == length-1) || front == rear+1){
			return -999;
	}
	
	if(front == -1 && rear == -1){
		front = 0;
		rear = 0;
		arr[rear] = value;
	}
	
	else if(rear == length-1){
		rear = 0;
		arr[rear] = value;
	}
	else{
		rear++;
		arr[rear] = value;
	}
	avg = (avg*size + value)/++size;
	// size++;
	return arr[rear];
	
}

float Queue::deQueue(){
	float first;
	if(front == -1 && rear == -1){
		return;
	}
	else if(front == rear){
		first = arr[front];
		arr[front] = 0;
		rear = -1;
		front = -1;
	}
	else if(front == length-1){
		first = arr[front];
		arr[front] = 0;
		front = 0;
	}
	else{
		first = arr[front];
		arr[front] = 0;
		front++;
	}
	avg = (avg*size - first)/--size;
	// size--;
	return first;
}

float* Queue::getQueue(){
	return arr;
}

float Queue::avg_arr(){
	return avg;
}

int Queue::get_first_index(){
	return front;
}

int Queue::get_length(){
	return length;
}

int Queue::get_size(){
	return size;
}

int Queue::last(){
	return rear;
}

int Queue::second_last(){
	if(rear != 0 ) return rear-1;
	else return length-1;
}