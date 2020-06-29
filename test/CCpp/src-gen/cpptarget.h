#ifndef CPP_TARGET_H
#define CPP_TARGET_H

#include <iostream>
#include "core/pqueue.c"
#include "core/reactor.c"

template <class T>
class Port {
public:
	T value;
	bool _is_present;
	T get() { return value;}
	void set_value(T value) { this->value = value; }
	bool is_present () { return _is_present;}
	T* get_pointer() { return &this->value; }
};

#endif // CPP_TARGET_H
