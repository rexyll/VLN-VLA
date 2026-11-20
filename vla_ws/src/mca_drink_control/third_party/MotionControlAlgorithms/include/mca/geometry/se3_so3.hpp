#pragma once
#include <array>
#include <cmath>
#include <limits>
namespace mca::geometry {
struct Vec3 { double x{0}, y{0}, z{0}; };
struct Quaternion { double w{1}, x{0}, y{0}, z{0}; };
struct Mat3 { std::array<double,9> v{1,0,0,0,1,0,0,0,1}; double& operator()(int r,int c) noexcept{return v[static_cast<std::size_t>(r*3+c)];} double operator()(int r,int c) const noexcept{return v[static_cast<std::size_t>(r*3+c)];} };
struct Pose3D { Vec3 position{}; Quaternion rotation{}; };
inline Vec3 operator+(Vec3 a,Vec3 b) noexcept{return {a.x+b.x,a.y+b.y,a.z+b.z};}
inline Vec3 operator-(Vec3 a,Vec3 b) noexcept{return {a.x-b.x,a.y-b.y,a.z-b.z};}
inline Vec3 operator*(double s,Vec3 a) noexcept{return {s*a.x,s*a.y,s*a.z};}
inline double dot(Vec3 a,Vec3 b) noexcept{return a.x*b.x+a.y*b.y+a.z*b.z;}
inline Vec3 cross(Vec3 a,Vec3 b) noexcept{return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};}
inline double norm(Vec3 a) noexcept{return std::sqrt(dot(a,a));}
inline double quaternion_norm(Quaternion q) noexcept{return std::sqrt(q.w*q.w+q.x*q.x+q.y*q.y+q.z*q.z);}
inline bool normalize(Quaternion& q) noexcept{const double n=quaternion_norm(q);if(!std::isfinite(n)||n<=std::numeric_limits<double>::epsilon())return false;q.w/=n;q.x/=n;q.y/=n;q.z/=n;return true;}
inline Quaternion normalized(Quaternion q) noexcept{if(!normalize(q))return {};return q;}
inline Quaternion conjugate(Quaternion q) noexcept{return {q.w,-q.x,-q.y,-q.z};}
inline Quaternion multiply(Quaternion a,Quaternion b) noexcept{return {a.w*b.w-a.x*b.x-a.y*b.y-a.z*b.z,a.w*b.x+a.x*b.w+a.y*b.z-a.z*b.y,a.w*b.y-a.x*b.z+a.y*b.w+a.z*b.x,a.w*b.z+a.x*b.y-a.y*b.x+a.z*b.w};}
inline Vec3 rotate(Quaternion qr,Vec3 p) noexcept{const Quaternion q=normalized(qr);const Vec3 u{q.x,q.y,q.z};return p+2.0*cross(u,cross(u,p)+q.w*p);}
inline Mat3 to_rotation_matrix(Quaternion qr) noexcept{const Quaternion q=normalized(qr);const double xx=q.x*q.x,yy=q.y*q.y,zz=q.z*q.z,xy=q.x*q.y,xz=q.x*q.z,yz=q.y*q.z,wx=q.w*q.x,wy=q.w*q.y,wz=q.w*q.z;Mat3 R;R(0,0)=1-2*(yy+zz);R(0,1)=2*(xy-wz);R(0,2)=2*(xz+wy);R(1,0)=2*(xy+wz);R(1,1)=1-2*(xx+zz);R(1,2)=2*(yz-wx);R(2,0)=2*(xz-wy);R(2,1)=2*(yz+wx);R(2,2)=1-2*(xx+yy);return R;}
inline Quaternion from_rotation_matrix(const Mat3& R) noexcept{const double t=R(0,0)+R(1,1)+R(2,2);Quaternion q;if(t>0){const double s=2*std::sqrt(t+1);q={.25*s,(R(2,1)-R(1,2))/s,(R(0,2)-R(2,0))/s,(R(1,0)-R(0,1))/s};}else if(R(0,0)>R(1,1)&&R(0,0)>R(2,2)){const double s=2*std::sqrt(1+R(0,0)-R(1,1)-R(2,2));q={(R(2,1)-R(1,2))/s,.25*s,(R(0,1)+R(1,0))/s,(R(0,2)+R(2,0))/s};}else if(R(1,1)>R(2,2)){const double s=2*std::sqrt(1+R(1,1)-R(0,0)-R(2,2));q={(R(0,2)-R(2,0))/s,(R(0,1)+R(1,0))/s,.25*s,(R(1,2)+R(2,1))/s};}else{const double s=2*std::sqrt(1+R(2,2)-R(0,0)-R(1,1));q={(R(1,0)-R(0,1))/s,(R(0,2)+R(2,0))/s,(R(1,2)+R(2,1))/s,.25*s};}return normalized(q);}
inline Quaternion axis_angle(Vec3 axis,double angle) noexcept{const double n=norm(axis);if(!std::isfinite(n)||n<=std::numeric_limits<double>::epsilon())return {};const double s=std::sin(.5*angle)/n;return normalized({std::cos(.5*angle),axis.x*s,axis.y*s,axis.z*s});}
inline Quaternion exp_so3(Vec3 v) noexcept{const double a=norm(v);if(a<1e-12)return normalized({1,.5*v.x,.5*v.y,.5*v.z});return axis_angle(v,a);}
inline Vec3 log_so3(Quaternion qr) noexcept{Quaternion q=normalized(qr);if(q.w<0){q.w=-q.w;q.x=-q.x;q.y=-q.y;q.z=-q.z;}const double h=std::acos(std::max(-1.0,std::min(1.0,q.w)));const double s=std::sin(h);if(std::abs(s)<1e-12)return {2*q.x,2*q.y,2*q.z};const double k=2*h/s;return {k*q.x,k*q.y,k*q.z};}
inline Pose3D compose(Pose3D a,Pose3D b) noexcept{return {a.position+rotate(a.rotation,b.position),normalized(multiply(a.rotation,b.rotation))};}
inline Pose3D inverse(Pose3D p) noexcept{const Quaternion r=normalized(p.rotation),ri=conjugate(r);return {rotate(ri,-1.0*p.position),ri};}
inline Vec3 transform_point(Pose3D p,Vec3 x) noexcept{return p.position+rotate(p.rotation,x);}
inline Pose3D relative_pose(Pose3D a,Pose3D b) noexcept{return compose(inverse(a),b);}
}