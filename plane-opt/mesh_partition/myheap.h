/******************************************************************/
/*
  Try to create a max-heap with following features:
  - Θ(1) time search
  - Θ(n) time construction and batch insert
  - Θ(logn) time insert
  - Θ(logn) time extract top
  - Θ(logn) time change key (and check key if value was indirectly changed)

  Features:
  - Replace the self-defined storage structure 'MxDynBlock' from the original reference code 'qslim'
    with standard STL structure std::vector, since the former is too verbose and also hard to handle.
  - Easy to use: all code is in current header file.

  Usage example:
  //-----------------------------------------------------------//
  // Define your own class
  struct Obj : public MxHeapable {
    // Put your own member here
  }
  // Define heap and insert element
  MxHeap myheap;
  Obj * obj = new Obj();
  myheap.insert(obj);
  // Use any other heap operations like extract(), update(), remove()
  //-----------------------------------------------------------//

  Reference:
  - qslime code by Michael Garland: http://www.cs.cmu.edu/~./garland/quadrics/qslim.html

*/
/******************************************************************/

#ifndef MXHEAP_H
#define MXHEAP_H

#include <iostream>
#include <vector>

//! Basic element of the heap. Same as old code
class MxHeapable
{
private:
    double import;  // key (sorted by this value in the heap)
    int token;      // position in the heap

public:
    MxHeapable()
    {
        not_in_heap();
        heap_key(0.0f);
    }
    //! Determine if an element is still a heap element by simply checking the token
    inline bool is_in_heap() { return token != -47; }
    //! Remove a heap element by simply setting the token as some negative position
    inline void not_in_heap() { token = -47; }
    //! Return position in the heap
    inline int get_heap_pos() { return token; }
    //! Set position
    inline void set_heap_pos(int t) { token = t; }
    //! Update key
    inline void heap_key(double k) { import = k; }
    //! Get key
    inline double heap_key() const { return import; }
};

//! Max-Heap
class MxHeap
{
private:
    //! Update one element in a position in the heap
    void place(MxHeapable *x, unsigned int i)
    {
        data[i] = x;
        x->set_heap_pos(i);
    }
    //! Swap two elements
    void swap(unsigned int i, unsigned int j)
    {
        MxHeapable *tmp = data[i];
        place(data[j], i);
        place(tmp, j);
    }
    //! Get parent index in the heap
    unsigned int parent(unsigned int i) { return (i - 1) / 2; }
    //! Get left child index in the heap
    unsigned int left(unsigned int i) { return 2 * i + 1; }
    //! Get left child index in the heap
    unsigned int right(unsigned int i) { return 2 * i + 2; }
    //! Pull up an element in the heap tree (O(logn) time)
    void upheap(unsigned int i)
    {
        MxHeapable *moving = data[i];
        unsigned int index = i;
        unsigned int p = parent(i);
        // Move larger value to top to create a max-heap
        while (index > 0 && moving->heap_key() > data[p]->heap_key())
        {
            place(data[p], index);
            index = p;
            p = parent(p);
        }
        if (index != i)
            place(moving, index);
    }
    //! Push down an element in the heap tree (O(logn) time, similar to upheap)
    void downheap(unsigned int i)
    {
        MxHeapable *moving = data[i];
        unsigned int index = i, largest = i;
        unsigned int l = left(i), r = right(i);
        while (l < length())
        {
            // Move smaller value down to create a max-heap
            if (r < length() && data[l]->heap_key() < data[r]->heap_key())
                largest = r;
            else
                largest = l;
            if (moving->heap_key() < data[largest]->heap_key())
            {
                place(data[largest], index);
                index = largest;
                l = left(index);
                r = right(index);
            }
            else
                break;
        }
        if (index != i)
            place(moving, index);
    }

private:
    std::vector<MxHeapable *> data;  // we keep the pointers in the heap

public:
    MxHeap() { data.reserve(8); }
    MxHeap(unsigned int n) { data.resize(n); }

    //! Insert an element with a key value
    void insert(MxHeapable *t, double v)
    {
        t->heap_key(v);
        data.push_back(t);
        unsigned int i = length() - 1;
        t->set_heap_pos(i);
        upheap(i);
    }
    //! Overload function to insert an element with key already updated
    void insert(MxHeapable *t) { insert(t, t->heap_key()); }

    //! Return true and update an element if it exists in the heap, and false otherwise.
    /*!
        Note the input element MUST be already in the heap. So call this function
        ONLY if this is sure.
    */
    bool update(MxHeapable *t, double v)
    {
        if (!t->is_in_heap())
            return false;
        t->heap_key(v);
        unsigned int i = t->get_heap_pos();
        if (i > 0 && v > data[parent(i)]->heap_key())
            upheap(i);
        else
            downheap(i);
        return true;
    }
    //! Overload function of update(), if key is already inside heap element
    bool update(MxHeapable *t) { return update(t, t->heap_key()); }
    //! Data size
    unsigned int size() const { return static_cast<unsigned int>(data.size()); }
    //! Different name but same function of 'size()'
    unsigned int length() const {return static_cast<unsigned int>(data.size()); }
    //! Get item in a position
    MxHeapable *item(unsigned int i) { return data[i]; }
    const MxHeapable *item(unsigned int i) const { return data[i]; }
    //! Extract the max value from the heap
    MxHeapable *extract()
    {
        if (length() < 1)
            return nullptr;
        swap(0, length() - 1);
        MxHeapable *dead = data.back();
        data.pop_back();
        downheap(0);
        dead->not_in_heap();
        return dead;
    }
    //! Different name but same function of 'extract()'
    MxHeapable *pop() { return extract(); }
    //! Get the max value from the heap (but do not pop it)
    MxHeapable *top() { return (length() < 1 ? nullptr : data[0]); }
    //! Remove one element.
    /*!
      Return the element if it exists in the heap (will set it not in heap),
      otherwise return nullptr.
    */
    MxHeapable *remove(MxHeapable *t)
    {
        if (!t->is_in_heap())
            return nullptr;
        int i = t->get_heap_pos();
        swap(i, length() - 1);
        data.pop_back();
        t->not_in_heap();
        if (data[i]->heap_key() < t->heap_key())
            downheap(i);
        else
            upheap(i);
        return t;
    }
    //! Destroy the heap by releasing all pointers
    /*!
        Be careful: do not delete these pointers again somewhere else later.
    */
    void destroy()
    {
        for (unsigned int i = 0; i < length(); ++i)
        {
            delete data[i];
            data[i] = nullptr;
        }
        data.clear();
    }
};

#endif
