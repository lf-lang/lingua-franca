// Copyright (C) 2020 TU Dresden

// A simple classes for strocing a matrix compactly and providing convenient access
template<class T>
class Matrix {
  private:
    std::vector<T> data;
    size_t size_y;

  public:
    Matrix(size_t size_x, size_t size_y) : data(size_x * size_y), size_y(size_y) {}

    const T& at(size_t x, size_t y) const { return data[x*size_y+y]; }
    T& at(size_t x, size_t y) { return data[x*size_y+y]; }
};
