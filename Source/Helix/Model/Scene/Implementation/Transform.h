#pragma once

#include <Helix/Core/Basics/Types.h>

#include <cmath>

// 4x4 matrices in double, column major (m[column * 4 + row]), for the transforms the readers bake.
namespace helix::model::detail
{
    struct Mat4
    {
        double m[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};

        double& operator()(int row, int col) { return m[col * 4 + row]; }
        double operator()(int row, int col) const { return m[col * 4 + row]; }

        static Mat4 Translation(double x, double y, double z)
        {
            Mat4 r;
            r.m[12] = x, r.m[13] = y, r.m[14] = z;
            return r;
        }
        static Mat4 Scale(double x, double y, double z)
        {
            Mat4 r;
            r.m[0] = x, r.m[5] = y, r.m[10] = z;
            return r;
        }
        // A rotation about one axis (0 x, 1 y, 2 z) by degrees.
        static Mat4 Axis(int axis, double degrees)
        {
            double a = degrees * 3.14159265358979323846 / 180, c = std::cos(a), s = std::sin(a);
            Mat4 r;
            int i = (axis + 1) % 3, j = (axis + 2) % 3;
            r(i, i) = c, r(i, j) = -s, r(j, i) = s, r(j, j) = c;
            return r;
        }
        static Mat4 Quaternion(double x, double y, double z, double w)
        {
            Mat4 r;
            r(0, 0) = 1 - 2 * (y * y + z * z), r(0, 1) = 2 * (x * y - z * w), r(0, 2) = 2 * (x * z + y * w);
            r(1, 0) = 2 * (x * y + z * w), r(1, 1) = 1 - 2 * (x * x + z * z), r(1, 2) = 2 * (y * z - x * w);
            r(2, 0) = 2 * (x * z - y * w), r(2, 1) = 2 * (y * z + x * w), r(2, 2) = 1 - 2 * (x * x + y * y);
            return r;
        }
    };

    inline Mat4 operator*(const Mat4& a, const Mat4& b)
    {
        Mat4 r;
        for (int c = 0; c < 4; c++)
            for (int row = 0; row < 4; row++)
            {
                double s = 0;
                for (int k = 0; k < 4; k++)
                    s += a(row, k) * b(k, c);
                r(row, c) = s;
            }
        return r;
    }

    // The inverse of a general matrix; the identity when it has none.
    inline Mat4 Inverse(const Mat4& a)
    {
        double inv[16], s[16];
        for (int i = 0; i < 16; i++)
            s[i] = a.m[i];
        inv[0] = s[5] * s[10] * s[15] - s[5] * s[11] * s[14] - s[9] * s[6] * s[15] + s[9] * s[7] * s[14] + s[13] * s[6] * s[11] - s[13] * s[7] * s[10];
        inv[4] = -s[4] * s[10] * s[15] + s[4] * s[11] * s[14] + s[8] * s[6] * s[15] - s[8] * s[7] * s[14] - s[12] * s[6] * s[11] + s[12] * s[7] * s[10];
        inv[8] = s[4] * s[9] * s[15] - s[4] * s[11] * s[13] - s[8] * s[5] * s[15] + s[8] * s[7] * s[13] + s[12] * s[5] * s[11] - s[12] * s[7] * s[9];
        inv[12] = -s[4] * s[9] * s[14] + s[4] * s[10] * s[13] + s[8] * s[5] * s[14] - s[8] * s[6] * s[13] - s[12] * s[5] * s[10] + s[12] * s[6] * s[9];
        inv[1] = -s[1] * s[10] * s[15] + s[1] * s[11] * s[14] + s[9] * s[2] * s[15] - s[9] * s[3] * s[14] - s[13] * s[2] * s[11] + s[13] * s[3] * s[10];
        inv[5] = s[0] * s[10] * s[15] - s[0] * s[11] * s[14] - s[8] * s[2] * s[15] + s[8] * s[3] * s[14] + s[12] * s[2] * s[11] - s[12] * s[3] * s[10];
        inv[9] = -s[0] * s[9] * s[15] + s[0] * s[11] * s[13] + s[8] * s[1] * s[15] - s[8] * s[3] * s[13] - s[12] * s[1] * s[11] + s[12] * s[3] * s[9];
        inv[13] = s[0] * s[9] * s[14] - s[0] * s[10] * s[13] - s[8] * s[1] * s[14] + s[8] * s[2] * s[13] + s[12] * s[1] * s[10] - s[12] * s[2] * s[9];
        inv[2] = s[1] * s[6] * s[15] - s[1] * s[7] * s[14] - s[5] * s[2] * s[15] + s[5] * s[3] * s[14] + s[13] * s[2] * s[7] - s[13] * s[3] * s[6];
        inv[6] = -s[0] * s[6] * s[15] + s[0] * s[7] * s[14] + s[4] * s[2] * s[15] - s[4] * s[3] * s[14] - s[12] * s[2] * s[7] + s[12] * s[3] * s[6];
        inv[10] = s[0] * s[5] * s[15] - s[0] * s[7] * s[13] - s[4] * s[1] * s[15] + s[4] * s[3] * s[13] + s[12] * s[1] * s[7] - s[12] * s[3] * s[5];
        inv[14] = -s[0] * s[5] * s[14] + s[0] * s[6] * s[13] + s[4] * s[1] * s[14] - s[4] * s[2] * s[13] - s[12] * s[1] * s[6] + s[12] * s[2] * s[5];
        inv[3] = -s[1] * s[6] * s[11] + s[1] * s[7] * s[10] + s[5] * s[2] * s[11] - s[5] * s[3] * s[10] - s[9] * s[2] * s[7] + s[9] * s[3] * s[6];
        inv[7] = s[0] * s[6] * s[11] - s[0] * s[7] * s[10] - s[4] * s[2] * s[11] + s[4] * s[3] * s[10] + s[8] * s[2] * s[7] - s[8] * s[3] * s[6];
        inv[11] = -s[0] * s[5] * s[11] + s[0] * s[7] * s[9] + s[4] * s[1] * s[11] - s[4] * s[3] * s[9] - s[8] * s[1] * s[7] + s[8] * s[3] * s[5];
        inv[15] = s[0] * s[5] * s[10] - s[0] * s[6] * s[9] - s[4] * s[1] * s[10] + s[4] * s[2] * s[9] + s[8] * s[1] * s[6] - s[8] * s[2] * s[5];
        double det = s[0] * inv[0] + s[1] * inv[4] + s[2] * inv[8] + s[3] * inv[12];
        Mat4 r;
        if (det == 0 || !std::isfinite(det))
            return r;
        for (int i = 0; i < 16; i++)
            r.m[i] = inv[i] / det;
        return r;
    }

    // Translation, rotation (x, y, z, w) and scale of a matrix without shear; a mirror shows as a negative x scale.
    inline void Decompose(const Mat4& a, double* t, double* q, double* s)
    {
        t[0] = a.m[12], t[1] = a.m[13], t[2] = a.m[14];
        for (int c = 0; c < 3; c++)
            s[c] = std::sqrt(a.m[c * 4] * a.m[c * 4] + a.m[c * 4 + 1] * a.m[c * 4 + 1] + a.m[c * 4 + 2] * a.m[c * 4 + 2]);
        double det = a(0, 0) * (a(1, 1) * a(2, 2) - a(1, 2) * a(2, 1)) - a(0, 1) * (a(1, 0) * a(2, 2) - a(1, 2) * a(2, 0)) +
                     a(0, 2) * (a(1, 0) * a(2, 1) - a(1, 1) * a(2, 0));
        if (det < 0)
            s[0] = -s[0];
        double r[3][3];
        for (int c = 0; c < 3; c++)
            for (int row = 0; row < 3; row++)
                r[row][c] = s[c] != 0 ? a(row, c) / s[c] : (row == c ? 1 : 0);
        double trace = r[0][0] + r[1][1] + r[2][2];
        if (trace > 0)
        {
            double k = std::sqrt(trace + 1) * 2;
            q[3] = 0.25 * k, q[0] = (r[2][1] - r[1][2]) / k, q[1] = (r[0][2] - r[2][0]) / k, q[2] = (r[1][0] - r[0][1]) / k;
        }
        else if (r[0][0] > r[1][1] && r[0][0] > r[2][2])
        {
            double k = std::sqrt(1 + r[0][0] - r[1][1] - r[2][2]) * 2;
            q[3] = (r[2][1] - r[1][2]) / k, q[0] = 0.25 * k, q[1] = (r[0][1] + r[1][0]) / k, q[2] = (r[0][2] + r[2][0]) / k;
        }
        else if (r[1][1] > r[2][2])
        {
            double k = std::sqrt(1 + r[1][1] - r[0][0] - r[2][2]) * 2;
            q[3] = (r[0][2] - r[2][0]) / k, q[0] = (r[0][1] + r[1][0]) / k, q[1] = 0.25 * k, q[2] = (r[1][2] + r[2][1]) / k;
        }
        else
        {
            double k = std::sqrt(1 + r[2][2] - r[0][0] - r[1][1]) * 2;
            q[3] = (r[1][0] - r[0][1]) / k, q[0] = (r[0][2] + r[2][0]) / k, q[1] = (r[1][2] + r[2][1]) / k, q[2] = 0.25 * k;
        }
        double n = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
        if (n > 0)
            for (int i = 0; i < 4; i++)
                q[i] /= n;
    }
}
