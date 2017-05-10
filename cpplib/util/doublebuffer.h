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

#ifndef __DOUBLE_BUFFER_T
#define __DOUBLE_BUFFER_T

#include <stdint.h>

template<class T, int size> 
class DoubleBuffer
{
public:
    DoubleBuffer();
    
    T *A() { return mPtrA; }
    T *B() { return mPtrB; }
    
    T *current() { return mPtrA; }
    T *next() { return mPtrB; }
    
    void clearA();
    void clearCurrent(){ clearA(); }
    void clearB();
    void clearNext(){ clearB(); }
    
    void clear(){ clearA(); clearB(); }
    
    void swapBuffers();
	
private:
    T   mBuffer1[size];
    T   mBuffer2[size];
    T   *mPtrA;
    T   *mPtrB;
    T   *swapPtr;
  int debugvar;
};

template<class T, int size> 
DoubleBuffer<T, size>::DoubleBuffer()
{
    mPtrA = mBuffer1;
    mPtrB = mBuffer2;
    clear();
}

template<class T, int size> 
void DoubleBuffer<T, size>::clearA()
{
    memset((void *)mPtrA, 0, sizeof(T)*size);
}

template<class T, int size> 
void DoubleBuffer<T, size>::clearB()
{
    memset((void *)mPtrB, 0, sizeof(T)*size);
}

template<class T, int size> 
void DoubleBuffer<T, size>::swapBuffers()
{
    swapPtr = mPtrA;
    mPtrA = mPtrB;
    mPtrB = swapPtr;
}



#endif /* __DOUBLE_BUFFER_T */
