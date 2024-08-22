#include <iostream>

#include "include/kinematics.h"
#include "include/geometry.h"

Kinematics* Kinematics::addDisplacement(double x,
                                        double y,
                                        double z,
                                        double a,
                                        double b,
                                        double c)
{
  Vec6d disp;
  disp << x, y, z, a, b, c;
  _disps.push_back(disp);

  return this;
}

Vec6d Kinematics::getDisplacement(int i)
{
  return _disps[i % _disps.size()];
}

Kinematics* Kinematics::addJointInfo(double min, double max)
{
  _joint_limits.push_back(Vec2d(min, max));

  return this;
}

void Kinematics::setInvKin(std::function<VecXd(CVec6dRef, CVecXdRef, const double&)> invKin)
{
  _inv_kin = invKin;
}

VecXd Kinematics::xToQ(CVec6dRef pose, const double& wrist)
{
  return xToQ(pose, VecXd::Zero(_disps.size() - 1), wrist);
}

VecXd Kinematics::xToQ(CVec6dRef pose, CVecXdRef qinit, const double& wrist)
{
  VecXd q(qinit.size());
  q = _inv_kin(pose, qinit, wrist);
  return q;
}

Vec6d Kinematics::qToX(CVecXdRef q)
{
  if (q.size() >= _disps.size())
    throw Exception("Too many joint values for number of displacements.");

  Vec6d qvec;
  Vec6d x(Vec6d::Zero());
  for (int i = q.size() - 1; i >= 0; i--)
  {
    qvec << 0, 0, 0, 0, 0, q[i];
    Geometry::apply(qvec, x);
    Geometry::apply(_disps[i], x);
  }

  return x;
}

Mat6XdRef Kinematics::calcJacobian(CVecXdRef q)
{
  if (q.size() >= _disps.size())
    throw Exception("Too many joint values for number of displacements.");

  Mat6Xd jacobian(6, q.size());

  Vec3d axis(0, 0, 1);
  Vec3d rot(0, 0, 0);
  Vec3d pos;

  Geometry::apply(_disps[0].tail(3), axis);
  // TODO: check indices
  for (int i = 0; i < q.size(); i++)
  {
    if (i >= 0)
    {
      rot[2] = q[i];
      Geometry::apply(rot, axis);
    }
    pos << qToX(q.head(i)).head(3);
    jacobian.col(i) << axis, pos.cross(axis);
  }

  return jacobian;
}

Kinematics::Exception::Exception(std::string message)
{
  _message = message;
}

Kinematics::Exception::~Exception()
{
}

std::string Kinematics::Exception::what()
{
  return "Kinematics Exception: " + _message;
}
