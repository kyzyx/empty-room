/* Source file for the GAPS quaternion class */



/* Include files */

#include "R3Shapes/R3Shapes.h"



/* Public variables */

const R3Quaternion R3null_quaternion(0.0, 0.0, 0.0, 0.0);
const R3Quaternion R3zero_quaternion(0.0, 0.0, 0.0, 0.0);
const R3Quaternion R3identity_quaternion(1.0, 0.0, 0.0, 0.0);



/* Public functions */

int 
R3InitQuaternion()
{
    /* Return success */
    return TRUE;
}



void 
R3StopQuaternion()
{
}



R3Quaternion::
R3Quaternion(void)
{
}



R3Quaternion::
R3Quaternion(RNScalar a, RNScalar b, RNScalar c, RNScalar d)
{
    v[0] = a; 
    v[1] = b; 
    v[2] = c;
    v[3] = d;
    Normalize();
}



R3Quaternion::
R3Quaternion(const R3Quaternion& quaternion)
{
    v[0] = quaternion.v[0]; 
    v[1] = quaternion.v[1]; 
    v[2] = quaternion.v[2]; 
    v[3] = quaternion.v[3]; 
}



R3Quaternion::
R3Quaternion(const R3Quaternion& quaternion1, const R3Quaternion& quaternion2, RNScalar t)
{
  *this = R3QuaternionSlerp(quaternion1, quaternion2, t);
}



R3Quaternion::
R3Quaternion(const R3Vector& axis, RNAngle theta)
{
    RNAngle half_theta = 0.5 * theta;
    RNScalar sine_half_theta = sin(half_theta);
    v[0] = cos(half_theta);
    v[1] = axis[0] * sine_half_theta; 
    v[2] = axis[1] * sine_half_theta; 
    v[3] = axis[2] * sine_half_theta; 
    Normalize();
}



#if 0

R3Quaternion::
R3Quaternion(const R4Matrix& m, int /* dummy */)
{
    // http://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation
    RNScalar matrix[16];
    matrix[0] = m[0][0] - m[1][1] - m[2][2];
    matrix[1] = m[1][0] + m[0][1];
    matrix[2] = m[2][0] + m[0][2];
    matrix[3] = m[1][2] - m[2][1];
    matrix[4] = matrix[1]; 
    matrix[5] = m[1][1] - m[0][0] - m[2][2];
    matrix[6] = m[2][1] + m[1][2];
    matrix[7] = m[2][0] - m[0][2];
    matrix[8] = matrix[2];
    matrix[9] = matrix[6];
    matrix[10] = m[2][2] - m[0][0] - m[1][1];
    matrix[11] = m[0][1] - m[1][0];
    matrix[12] = matrix[3];
    matrix[13] = matrix[7];
    matrix[14] = matrix[11];
    matrix[15] = m[0][0] + m[1][1] + m[2][2];

    // Find eigenvector corresponding to largest eigenvalue
    RNAbort("Not implemented");

    // Normalize
    Normalize();
}

#endif



R3Quaternion::
R3Quaternion(RNAngle pitch, RNAngle yaw, RNAngle roll)
{
  *this = R3Quaternion(R3Vector(pitch, yaw), roll);
}



R3Quaternion::
R3Quaternion(const RNScalar array[4])
{
    v[0] = array[0]; 
    v[1] = array[1]; 
    v[2] = array[2];
    v[3] = array[3];
    Normalize();
}



const R4Matrix R3Quaternion::
Matrix(void) const
{
    // Return rotation matrix
    RNScalar a = v[0];
    RNScalar b = v[1];
    RNScalar c = v[2];
    RNScalar d = v[3];
    R4Matrix m = R4identity_matrix;
    m[0][0] = a*a + b*b - c*c - d*d;
    m[0][1] = 2*b*c - 2*a*d;
    m[0][2] = 2*b*d + 2*a*c;
    m[1][0] = 2*b*c + 2*a*d;
    m[1][1] = a*a - b*b + c*c - d*d;
    m[1][2] = 2*c*d - 2*a*b;
    m[2][0] = 2*b*d - 2*a*c;
    m[2][1] = 2*c*d + 2*a*b;
    m[2][2] = a*a - b*b - c*c + d*d;
    return m;
}



const R3Quaternion R3Quaternion::
Inverse(void) const
{
    // Return inverse rotation
    return R3Quaternion(v[0], -v[1], -v[2], -v[3]);
}



const RNBoolean R3Quaternion::
operator==(const R3Quaternion& quaternion) const
{
    // Return whether quaternion is equal
    return ((v[0] == quaternion.v[0]) && 
            (v[1] == quaternion.v[1]) && 
            (v[2] == quaternion.v[2]) && 
            (v[3] == quaternion.v[3]));
}



const RNBoolean R3Quaternion::
operator!=(const R3Quaternion& quaternion) const
{
    // Return whether quaternion is not equal
    return ((v[0] != quaternion.v[0]) || 
            (v[1] != quaternion.v[1]) || 
            (v[2] != quaternion.v[2]) || 
            (v[3] != quaternion.v[3]));
}



void R3Quaternion::
Flip(void)
{
    // Negate all numbers (does not change roation)
    v[0] = -v[0];
    v[1] = -v[1];
    v[2] = -v[2];
    v[3] = -v[3];
}



void R3Quaternion::
Multiply(const R3Quaternion& q2)
{
    // Multiply quaternion (equivalent to concatenating rotations)
    RNScalar q1[4]; q1[0] = v[0]; q1[1] = v[1]; q1[2] = v[2]; q1[3] = v[3];
    v[0] = q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3];
    v[1] = q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2];
    v[2] = q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1];
    v[3] = q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0];
}



void R3Quaternion::
Rotate(const R3Quaternion& q2)
{
    // Multiply quaternion (equivalent to concatenating rotations)
    Multiply(q2);
}



R3Quaternion& R3Quaternion::
operator=(const R3Quaternion& quaternion)
{
    // Assign quaternion
    v[0] = quaternion.v[0];
    v[1] = quaternion.v[1];
    v[2] = quaternion.v[2];
    v[3] = quaternion.v[3];
    return *this;
}



R3Quaternion& R3Quaternion::
operator*=(const R3Quaternion& quaternion)
{
    Multiply(quaternion);
    return *this;
}



R3Quaternion 
operator*(const R3Quaternion& quaternion1, const R3Quaternion& quaternion2)
{
    R3Quaternion q(quaternion1);
    q *= quaternion2;
    return q;
}



R3Quaternion 
R3QuaternionSlerp(const R3Quaternion& quaternion1, const R3Quaternion& quaternion2, RNScalar t)
{
  // Get convenient variables
  RNScalar q1[4] = { quaternion1[0], quaternion1[1],  quaternion1[2], quaternion1[3] };  
  RNScalar q2[4] = { quaternion2[0], quaternion2[1],  quaternion2[2], quaternion2[3] };  

  // Make sure going short way around sphere (q2 is same rotation as -q2)
  RNScalar dot = q1[0]*q2[0] + q1[1]*q2[1] + q1[2]*q2[2] + q1[3]*q2[3];
  if (dot < 0) { 
    q2[0] = -q2[0]; 
    q2[1] = -q2[1]; 
    q2[2] = -q2[2]; 
    q2[3] = -q2[3]; 
    dot = -dot; 
  }

  // Compute/check angle on 4D sphere
  RNAngle theta = (dot < 1) ? acos(dot) : 0;
  RNScalar denom = sin(theta);
  if (RNIsZero(denom)) {
    // Small angle, use linear interpolation
    RNScalar q[4];
    q[0] = (1-t)*q1[0] + t*q2[0];
    q[1] = (1-t)*q1[1] + t*q2[1];
    q[2] = (1-t)*q1[2] + t*q2[2];
    q[3] = (1-t)*q1[3] + t*q2[3];
    return R3Quaternion(q);
  }
  else {
    // Spherical interpolation
    RNScalar q[4];
    RNScalar t1 = sin((1.0-t)*theta) / denom;
    RNScalar t2 = sin(t*theta) / denom;
    q[0] = t1*q1[0] + t2*q2[0];
    q[1] = t1*q1[1] + t2*q2[1];
    q[2] = t1*q1[2] + t2*q2[2];
    q[3] = t1*q1[3] + t2*q2[3];
    return R3Quaternion(q);
  }
}



R3Vector
operator*(const R3Quaternion& q, const R3Vector& v)
{
    // Compute quaternion representation of vector
    RNLength length = v.Length();
    if (RNIsZero(length)) return v;
    R3Quaternion vector(0, v[0] / length, v[1] / length, v[2] / length);

    // Compute quaternion representation of rotated vector
    R3Quaternion conjugate(q[0], -q[1], -q[2], -q[3]);
    R3Quaternion result = q;
    result.Multiply(vector);
    result.Multiply(conjugate);
    assert(RNIsZero(result[0]));

    // Return rotated vector
    return R3Vector(length * result[1], length * result[2], length * result[3]); 
}



void R3Quaternion::
Normalize(void)
{
  // Normalize -- internal function because all R3Quaternions are normalized
  RNScalar sum = 0;
  sum += v[0]*v[0];
  sum += v[1]*v[1];
  sum += v[2]*v[2];
  sum += v[3]*v[3];
  RNScalar length = sqrt(sum);
  if (RNIsZero(length)) return;
  v[0] /= length;
  v[1] /= length;
  v[2] /= length;
  v[3] /= length;
}



void R3Quaternion::
Conjugate(void)
{
  // Take conjugate
  v[1] = -v[1];
  v[2] = -v[2];
  v[3] = -v[3];
}




