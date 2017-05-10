/*
  Copyright (c) 2011 Arduino.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef __RING_BUFFER_T
#define __RING_BUFFER_T

#include <stdint.h>

template<class T, int size> 
class RingBuffer
{
  public:
    RingBuffer();
    bool put(T *value);
    bool get(T *value);
    bool peek(T *value);
    void clear();
	bool isEmpty();
    bool isNotEmpty();
	bool isFull();
    bool isNotFull();
    int free();
    int count();
    int getHead(){ return _head; }
    int getTail(){ return _tail; }
    int getSize(){ return size; }
    
    T *outPtr() { return &_buffer[_tail]; }
	
  private:
	int nextIndex(int index);
    
    int  _head;
    int  _tail;
    T    _buffer[size];
    int  _bufferSize;
  int debugvar;
};

template<class T, int size> 
RingBuffer<T, size>::RingBuffer(){
    clear();
    _bufferSize = size;
    debugvar = 0;
}

template<class T, int size> 
bool RingBuffer<T, size>::put(T *value){
    if(isFull()) return false;
    _buffer[_head] = *value;
    _head = nextIndex(_head);
    debugvar++;
    return true;
}

template<class T, int size> 
bool RingBuffer<T, size>::get(T *value){
    if(isEmpty()) return false;
    *value = _buffer[_tail];
    _tail = nextIndex(_tail);
    return true;
}

template<class T, int size> 
bool RingBuffer<T, size>::peek(T *value){
    if(isEmpty()) return false;
    *value = _buffer[_tail];
    return true;
}

template<class T, int size> 
void RingBuffer<T, size>::clear(){
	_head = 0;
	_tail = 0;
}

template<class T, int size> 
int RingBuffer<T, size>::count(){
    if(_tail > _head) return _bufferSize + _head - _tail;
    else return _head - _tail;
}

template<class T, int size> 
int RingBuffer<T, size>::free(){
    return size - count();
}

template<class T, int size> 
int RingBuffer<T, size>::nextIndex(int index){
	return (uint32_t)(index + 1) % _bufferSize;
}

template<class T, int size> 
bool RingBuffer<T, size>::isFull(){
	return nextIndex(_head) == _tail;
}

template<class T, int size> 
bool RingBuffer<T, size>::isNotFull(){
	return nextIndex(_head) != _tail;
}

template<class T, int size> 
bool RingBuffer<T, size>::isEmpty(){
    return _head == _tail;
}

template<class T, int size> 
bool RingBuffer<T, size>::isNotEmpty(){
    return _head != _tail;
}


#endif /* __RING_BUFFER_T */
