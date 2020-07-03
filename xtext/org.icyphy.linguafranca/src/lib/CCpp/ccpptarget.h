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
	void set(T _value) { this->value = _value; this->_is_present=true; }
	bool is_present () { return _is_present;}
	Port<T>* get_pointer() { return this; }
};

#endif // CPP_TARGET_H