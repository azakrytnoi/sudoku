#pragma once

#include <cmath>
#include <cstdint>
#include <iostream>
#include <iomanip>

namespace geometry {

    template<typename T>
    class vector2 {
    public:
        vector2() :
                x(0), y(0)
        {
        }
        vector2(T xx) :
                x(xx), y(xx)
        {
        }
        vector2(T xx, T yy) :
                x(xx), y(yy)
        {
        }

        vector2 operator +(const vector2 &v) const
        {
            return vector2(x + v.x, y + v.y);
        }
        vector2 operator /(const T &r) const
        {
            return vector2(x / r, y / r);
        }
        vector2 operator *(const T &r) const
        {
            return vector2(x * r, y * r);
        }
        vector2& operator /=(const T &r)
        {
            x /= r, y /= r;
            return *this;
        }
        vector2& operator *=(const T &r)
        {
            x *= r, y *= r;
            return *this;
        }

        friend std::ostream& operator <<(std::ostream &s, const vector2<T> &v)
        {
            return s << '[' << v.x << ' ' << v.y << ']';
        }
        friend vector2 operator *(const T &r, const vector2<T> &v)
        {
            return vector2(v.x * r, v.y * r);
        }

        T x, y;
    };

    template<typename T>
    class vector3 {
    public:
        vector3() :
                x(0), y(0), z(0)
        {
        }
        vector3(T xx) :
                x(xx), y(xx), z(xx)
        {
        }
        vector3(T xx, T yy, T zz) :
                x(xx), y(yy), z(zz)
        {
        }
        vector3(const vector3& v) :
                x(v.x), y(v.y), z(v.z)
        {
        }

        vector3 operator +(const vector3 &v) const
        {
            return vector3(x + v.x, y + v.y, z + v.z);
        }
        vector3 operator -(const vector3 &v) const
        {
            return vector3(x - v.x, y - v.y, z - v.z);
        }
        vector3 operator *(const T &r) const
        {
            return vector3(x * r, y * r, z * r);
        }

        T dotProduct(const vector3<T> &v) const
        {
            return x * v.x + y * v.y + z * v.z;
        }
        T crossProduct(const vector3<T> &v) const
        {
            return vector3<T>(y * v.z - z * v.y, z * v.x - x * v.z, x * v.y - y * v.x);
        }
        T norm() const
        {
            return x * x + y * y + z * z;
        }

        T length() const
        {
            return std::sqrt(norm());
        }

        const T& operator [](uint8_t i) const
        {
            return (&x)[i];
        }
        T& operator [](uint8_t i)
        {
            return (&x)[i];
        }

        vector3& normalize()
        {
            T n = norm();
            if (n > 0)
            {
                T factor = 1 / std::sqrt(n);
                x *= factor, y *= factor, z *= factor;
            }

            return *this;
        }

        friend vector3 operator *(const T &r, const vector3 &v)
        {
            return vector3<T>(v.x * r, v.y * r, v.z * r);
        }
        friend vector3 operator /(const T &r, const vector3 &v)
        {
            return vector3<T>(r / v.x, r / v.y, r / v.z);
        }
        friend vector3 operator /(const vector3 &v, const T r)
        {
            return vector3<T>(v.x / r, v.y / r, v.z / r);
        }

        vector3& operator /=(const T r)
        {
            x /= r;
            y /= r;
            z /= r;
            return *this;
        }

        friend std::ostream& operator <<(std::ostream &s, const vector3<T> &v)
        {
            return s << '(' << v.x << ' ' << v.y << ' ' << v.z << ')';
        }

        T x, y, z;
    };

    template<typename T>
    class matrix44 {
    public:

        T x[4][4] = { { 1, 0, 0, 0 }, { 0, 1, 0, 0 }, { 0, 0, 1, 0 }, { 0, 0, 0, 1 } };

        matrix44()
        {
        }

        matrix44(T a, T b, T c, T d, T e, T f, T g, T h, T i, T j, T k, T l, T m, T n, T o, T p)
        {
            x[0][0] = a;
            x[0][1] = b;
            x[0][2] = c;
            x[0][3] = d;
            x[1][0] = e;
            x[1][1] = f;
            x[1][2] = g;
            x[1][3] = h;
            x[2][0] = i;
            x[2][1] = j;
            x[2][2] = k;
            x[2][3] = l;
            x[3][0] = m;
            x[3][1] = n;
            x[3][2] = o;
            x[3][3] = p;
        }

        const T* operator [](uint8_t i) const
        {
            return x[i];
        }
        T* operator [](uint8_t i)
        {
            return x[i];
        }

        matrix44 operator *(const matrix44& v) const
        {
            matrix44 tmp;
            multiply(*this, v, tmp);

            return tmp;
        }

        static void multiply(const matrix44<T> &a, const matrix44& b, matrix44 &c)
        {
#if 0
            for (uint8_t i = 0; i < 4; ++i)
            {
                for (uint8_t j = 0; j < 4; ++j)
                {
                    c[i][j] = a[i][0] * b[0][j] + a[i][1] * b[1][j] +
                    a[i][2] * b[2][j] + a[i][3] * b[3][j];
                }
            }
#else
            const T * __restrict ap = &a.x[0][0];
            const T * __restrict bp = &b.x[0][0];
            T * __restrict cp = &c.x[0][0];

            T a0, a1, a2, a3;

            a0 = ap[0];
            a1 = ap[1];
            a2 = ap[2];
            a3 = ap[3];

            cp[0] = a0 * bp[0] + a1 * bp[4] + a2 * bp[8] + a3 * bp[12];
            cp[1] = a0 * bp[1] + a1 * bp[5] + a2 * bp[9] + a3 * bp[13];
            cp[2] = a0 * bp[2] + a1 * bp[6] + a2 * bp[10] + a3 * bp[14];
            cp[3] = a0 * bp[3] + a1 * bp[7] + a2 * bp[11] + a3 * bp[15];

            a0 = ap[4];
            a1 = ap[5];
            a2 = ap[6];
            a3 = ap[7];

            cp[4] = a0 * bp[0] + a1 * bp[4] + a2 * bp[8] + a3 * bp[12];
            cp[5] = a0 * bp[1] + a1 * bp[5] + a2 * bp[9] + a3 * bp[13];
            cp[6] = a0 * bp[2] + a1 * bp[6] + a2 * bp[10] + a3 * bp[14];
            cp[7] = a0 * bp[3] + a1 * bp[7] + a2 * bp[11] + a3 * bp[15];

            a0 = ap[8];
            a1 = ap[9];
            a2 = ap[10];
            a3 = ap[11];

            cp[8] = a0 * bp[0] + a1 * bp[4] + a2 * bp[8] + a3 * bp[12];
            cp[9] = a0 * bp[1] + a1 * bp[5] + a2 * bp[9] + a3 * bp[13];
            cp[10] = a0 * bp[2] + a1 * bp[6] + a2 * bp[10] + a3 * bp[14];
            cp[11] = a0 * bp[3] + a1 * bp[7] + a2 * bp[11] + a3 * bp[15];

            a0 = ap[12];
            a1 = ap[13];
            a2 = ap[14];
            a3 = ap[15];

            cp[12] = a0 * bp[0] + a1 * bp[4] + a2 * bp[8] + a3 * bp[12];
            cp[13] = a0 * bp[1] + a1 * bp[5] + a2 * bp[9] + a3 * bp[13];
            cp[14] = a0 * bp[2] + a1 * bp[6] + a2 * bp[10] + a3 * bp[14];
            cp[15] = a0 * bp[3] + a1 * bp[7] + a2 * bp[11] + a3 * bp[15];
#endif
        }

        // \brief return a transposed copy of the current matrix as a new matrix
        matrix44 transposed() const
        {
#if 0
            matrix44 t;
            for (uint8_t i = 0; i < 4; ++i)
            {
                for (uint8_t j = 0; j < 4; ++j)
                {
                    t[i][j] = x[j][i];
                }
            }

            return t;
#else
            return matrix44(x[0][0], x[1][0], x[2][0], x[3][0], x[0][1], x[1][1], x[2][1], x[3][1], x[0][2], x[1][2], x[2][2], x[3][2],
                    x[0][3], x[1][3], x[2][3], x[3][3]);
#endif
        }

        // \brief transpose itself
        matrix44& transpose()
        {
            matrix44 tmp(x[0][0], x[1][0], x[2][0], x[3][0], x[0][1], x[1][1], x[2][1], x[3][1], x[0][2], x[1][2], x[2][2], x[3][2],
                    x[0][3], x[1][3], x[2][3], x[3][3]);
            *this = tmp;

            return *this;
        }

        template<typename S>
        void multVecMatrix(const vector3<S> &src, vector3<S> &dst) const
        {
            S a, b, c, w;

            a = src[0] * x[0][0] + src[1] * x[1][0] + src[2] * x[2][0] + x[3][0];
            b = src[0] * x[0][1] + src[1] * x[1][1] + src[2] * x[2][1] + x[3][1];
            c = src[0] * x[0][2] + src[1] * x[1][2] + src[2] * x[2][2] + x[3][2];
            w = src[0] * x[0][3] + src[1] * x[1][3] + src[2] * x[2][3] + x[3][3];

            dst.x = a / w;
            dst.y = b / w;
            dst.z = c / w;
        }

        template<typename S>
        void multDirMatrix(const vector3<S> &src, vector3<S> &dst) const
        {
            S a, b, c;

            a = src[0] * x[0][0] + src[1] * x[1][0] + src[2] * x[2][0];
            b = src[0] * x[0][1] + src[1] * x[1][1] + src[2] * x[2][1];
            c = src[0] * x[0][2] + src[1] * x[1][2] + src[2] * x[2][2];

            dst.x = a;
            dst.y = b;
            dst.z = c;
        }

        matrix44 inverse()
        {
            matrix44 s;
            matrix44 t(*this);

            // Forward elimination
            for (int i = 0; i < 3; i++)
            {
                int pivot = i;

                T pivotsize = t[i][i];

                if (pivotsize < 0)
                    pivotsize = -pivotsize;

                for (int j = i + 1; j < 4; j++)
                {
                    T tmp = t[j][i];

                    if (tmp < 0)
                        tmp = -tmp;

                    if (tmp > pivotsize)
                    {
                        pivot = j;
                        pivotsize = tmp;
                    }
                }

                if (pivotsize == 0)
                {
                    // Cannot invert singular matrix
                    return matrix44();
                }

                if (pivot != i)
                {
                    for (int j = 0; j < 4; j++)
                    {
                        T tmp;

                        tmp = t[i][j];
                        t[i][j] = t[pivot][j];
                        t[pivot][j] = tmp;

                        tmp = s[i][j];
                        s[i][j] = s[pivot][j];
                        s[pivot][j] = tmp;
                    }
                }

                for (int j = i + 1; j < 4; j++)
                {
                    T f = t[j][i] / t[i][i];

                    for (int k = 0; k < 4; k++)
                    {
                        t[j][k] -= f * t[i][k];
                        s[j][k] -= f * s[i][k];
                    }
                }
            }

            // Backward substitution
            for (int i = 3; i >= 0; --i)
            {
                T f;

                if ((f = t[i][i]) == 0)
                {
                    // Cannot invert singular matrix
                    return matrix44();
                }

                for (int j = 0; j < 4; j++)
                {
                    t[i][j] /= f;
                    s[i][j] /= f;
                }

                for (int j = 0; j < i; j++)
                {
                    f = t[j][i];

                    for (int k = 0; k < 4; k++)
                    {
                        t[j][k] -= f * t[i][k];
                        s[j][k] -= f * s[i][k];
                    }
                }
            }

            return s;
        }

        // \brief set current matrix to its inverse
        const matrix44<T>& invert()
        {
            *this = inverse();
            return *this;
        }

        friend std::ostream& operator <<(std::ostream &s, const matrix44 &m)
        {
            std::ios_base::fmtflags oldFlags = s.flags();
            int width = 12; // total with of the displayed number
            s.precision(5); // control the number of displayed decimals
            s.setf(std::ios_base::fixed);

            s << "(" << std::setw(width) << m[0][0] << " " << std::setw(width) << m[0][1] << " " << std::setw(width) << m[0][2] << " "
                    << std::setw(width) << m[0][3] << "\n" <<

                    " " << std::setw(width) << m[1][0] << " " << std::setw(width) << m[1][1] << " " << std::setw(width) << m[1][2] << " "
                    << std::setw(width) << m[1][3] << "\n" <<

                    " " << std::setw(width) << m[2][0] << " " << std::setw(width) << m[2][1] << " " << std::setw(width) << m[2][2] << " "
                    << std::setw(width) << m[2][3] << "\n" <<

                    " " << std::setw(width) << m[3][0] << " " << std::setw(width) << m[3][1] << " " << std::setw(width) << m[3][2] << " "
                    << std::setw(width) << m[3][3] << ")\n";

            s.flags(oldFlags);
            return s;
        }
    };

    template<typename T, typename S>
    vector3<S> operator *(const vector3<S>& v, const matrix44<T>& m)
    {
        vector3<S> dst;
        m.multVecMatrix(v, dst);
        return dst;
    }

    typedef vector2<int> vector2i;
    typedef vector2<double> vector2d;
    typedef vector3<double> vector3d;
    typedef vector3<int> vector3i;
    typedef vector2<long double> vector2ld;
    typedef vector3<long double> vector3ld;
    typedef matrix44<double> matrix44d;
    typedef matrix44<long double> matrix44ld;

}
