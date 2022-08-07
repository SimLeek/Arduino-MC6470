#ifndef __MC6470_MATH_HPP
#define __MC6470_MATH_HPP

#include <memory>
#include <array>

// todo: why tf does the stl for arduino have this deprecated auto_ptr? Replace it whenver possible.
template <class T, class U = T, class R = T>
std::auto_ptr<std::array<R, 3>> CrossProduct(T const a[3], U const b[3])
{
    R *c = new R[3];
    c[0] = (R)a[1] * (R)b[2] - (R)a[2] * (R)b[1];
    c[1] = (R)a[2] * (R)b[0] - (R)a[0] * (R)b[2];
    c[2] = (R)a[0] * (R)b[1] - (R)a[1] * (R)b[0];

    std::array<R, 3> *A = reinterpret_cast<std::array<R, 3> *>(c);
    std::auto_ptr<std::array<R, 3>> a_ptr(A);
    return a_ptr;
}

template <class T, class R = T>
R TwoNorm(T const a[3])
{
    R c;
    // avoid overflows from things like int8 or int16s
    c = (R)a[0] * (R)a[0];
    c += (R)a[1] * (R)a[1];
    c += (R)a[2] * (R)a[2];
    return c;
}

template <class T, class R = T>
R TwoNorm(const std::array<T, 3> &a)
{
    R c;
    // avoid overflows from things like int8 or int16s
    c = (R)a[0] * (R)a[0];
    c += (R)a[1] * (R)a[1];
    c += (R)a[2] * (R)a[2];
    return c;
}

template <class T, size_t MAX_BITS, class U = T, class R = T>
R add_with_limits_signed(T a, U b)
{
    func_print;
    R max_val = (R)(pow(2, MAX_BITS - 1) - 1);
    R min_val = -max_val - 1;
    // https://stackoverflow.com/a/1514309
    if (b > 0 && a > max_val - b)
    {
        return max_val;
    }
    else if (b < 0 && a < min_val - b)
    {
        return min_val;
    }
    else
    {
        return a + b;
    }
}

template <class T, size_t MAX_BITS, class U = T, class R = T>
R multiply_with_limits_unsigned(T a, U b)
{
    func_print;
    R max_val = (R)(pow(2, MAX_BITS) - 1);
    // https://stackoverflow.com/a/1514309
    if (b != 0 && a > max_val / b)
    {
        return max_val;
    }
    else if (b != 0 && a < 0)
    {
        return 0;
    }
    else
    {
        return a * b;
    }
}

#endif