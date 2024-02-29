/* Copyright (C) 2024 David Skuddis - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the MIT license.
 *
 * You should have received a copy of the MIT license with
 * this file. If not, please write to: davidskuddis@web.de, or visit: https://opensource.org/license/mit/
 */

#ifndef RINGBUFFER_H
#define RINGBUFFER_H

#include <vector>
#include <limits>
#include <iostream>
#include <memory>

using namespace std;

template <typename T>
class RingBuffer
{
protected:
    std::vector<T> data;
    int nextIndex = 0;
    int maxElements = 0;
    int numUpdates = 0;
    bool isInitialized = false;
    bool isFilled = false;

public:
    T &at(int chronologicalIndex)
    {
        if (isFilled)
            return data[(chronologicalIndex + nextIndex) % maxElements];
        else
            return data[chronologicalIndex % maxElements];
    }

    int getNumUpdates()
    {
        return numUpdates;
    }

    int getMaxNumElems()
    {
        return maxElements;
    }

    int getNumElements()
    {
        return std::min(numUpdates, maxElements);
    }

    void init(int maxElemIn)
    {
        // init buffer
        maxElements = maxElemIn;
        nextIndex = 0;

        data.resize(maxElements);

        numUpdates = 0;
        isInitialized = true;
        isFilled = false;
    }

    void addElem(T &elem)
    {
        if (isInitialized == false)
        {
            std::cerr << "Ring buffer is used without initialization. Initialize with a maximum of 10 elements . . . \n";
            init(10);
        }

        // copy
        data[nextIndex] = elem;

        // update index
        ++nextIndex;

        if (nextIndex == maxElements)
            nextIndex = 0;

        ++numUpdates;

        if (isFilled == false && numUpdates == maxElements)
            isFilled = true;
    }

    bool isFull()
    {
        return isFilled;
    }
};

#endif
