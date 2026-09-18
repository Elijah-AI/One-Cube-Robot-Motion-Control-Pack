#include "motion_control_pkg/wbc/model.hpp"
#include <urdf_parser/urdf_parser.h>
#include <algorithm>
#include <cmath>
#include <cctype>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <set>
#include <stdexcept>

namespace motion_control_pkg::wbc {
namespace {
namespace fs = std::filesystem;
std::uint32_t littleU32(const char* p) {
  const auto* b = reinterpret_cast<const unsigned char*>(p);
  return std::uint32_t(b[0]) | (std::uint32_t(b[1]) << 8) |
         (std::uint32_t(b[2]) << 16) | (std::uint32_t(b[3]) << 24);
}
double littleFloat(const char* p) {
  const std::uint32_t bits = littleU32(p);
  float result;
  std::memcpy(&result, &bits, sizeof(result));
  return result;
}
fs::path resolveMesh(const std::string& uri, const fs::path& urdf_path,
                     const std::vector<std::string>& package_dirs) {
  fs::path path;
  if (uri.rfind("package://", 0) == 0) {
    const std::string relative = uri.substr(10);
    for (const auto& dir : package_dirs) {
      if (fs::is_regular_file(fs::path(dir) / relative)) return fs::canonical(fs::path(dir) / relative);
    }
    // Standard source (src/package/urdf) and install (share/package/urdf) layouts.
    path = urdf_path.parent_path().parent_path().parent_path() / relative;
  } else if (uri.rfind("file://", 0) == 0) {
    path = uri.substr(7);
  } else {
    path = fs::path(uri).is_absolute() ? fs::path(uri) : urdf_path.parent_path() / uri;
  }
  if (!fs::is_regular_file(path)) throw std::invalid_argument("cannot resolve URDF collision mesh: " + uri);
  return fs::canonical(path);
}
std::vector<Eigen::Vector3d> readStl(const fs::path& path) {
  auto extension = path.extension().string();
  std::transform(extension.begin(), extension.end(), extension.begin(), [](unsigned char c) { return std::tolower(c); });
  if (extension != ".stl") throw std::invalid_argument("support mesh must be STL in stage 1: " + path.string());
  const auto size = fs::file_size(path);
  if (size > 512U * 1024U * 1024U) throw std::invalid_argument("support STL exceeds 512 MiB");
  std::ifstream in(path, std::ios::binary);
  std::vector<char> bytes(size);
  if (!in.read(bytes.data(), static_cast<std::streamsize>(size))) throw std::invalid_argument("cannot read STL");
  std::vector<Eigen::Vector3d> points;
  const std::uint64_t triangles = size >= 84 ? littleU32(bytes.data() + 80) : 0;
  if (size >= 84 && size == 84 + 50 * triangles) {
    points.reserve(3 * triangles);
    for (std::uint64_t i = 0; i < triangles; ++i) {
      for (int j = 0; j < 3; ++j) {
        const char* p = bytes.data() + 84 + 50 * i + 12 + 12 * j;
        points.emplace_back(littleFloat(p), littleFloat(p + 4), littleFloat(p + 8));
      }
    }
  } else {
    std::ifstream ascii(path);
    std::string token;
    while (ascii >> token) {
      if (token == "vertex") {
        double x, y, z;
        if (!(ascii >> x >> y >> z)) throw std::invalid_argument("malformed ASCII STL vertex");
        points.emplace_back(x, y, z);
      }
    }
    if (points.size() % 3 != 0) throw std::invalid_argument("incomplete ASCII STL triangle");
  }
  if (points.empty()) throw std::invalid_argument("empty or malformed support mesh");
  for (const auto& p : points) if (!p.allFinite()) throw std::invalid_argument("non-finite STL vertex");
  return points;
}
bool lessPoint(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
  for (int i = 0; i < 3; ++i) {
    if (a[i] != b[i]) return a[i] < b[i];
  }
  return false;
}
double cross(const Eigen::Vector2d& a, const Eigen::Vector2d& b, const Eigen::Vector2d& c) {
  const Eigen::Vector2d u = b - a, v = c - a;
  return u.x() * v.y() - u.y() * v.x();
}
std::vector<Eigen::Vector2d> hull(std::vector<Eigen::Vector2d> p) {
  std::sort(p.begin(), p.end(), [](const auto& a, const auto& b) {
    return a.x() == b.x() ? a.y() < b.y() : a.x() < b.x();
  });
  p.erase(std::unique(p.begin(), p.end(), [](const auto& a, const auto& b) {
    return (a - b).norm() < 1e-9;
  }), p.end());
  if (p.size() < 3) return {};
  std::vector<Eigen::Vector2d> h(2 * p.size());
  std::size_t k = 0;
  for (const auto& v : p) {
    while (k >= 2 && cross(h[k-2], h[k-1], v) <= 1e-12) --k;
    h[k++] = v;
  }
  const auto lower = k + 1;
  for (std::size_t i = p.size() - 1; i > 0; --i) {
    const auto& v = p[i-1];
    while (k >= lower && cross(h[k-2], h[k-1], v) <= 1e-12) --k;
    h[k++] = v;
  }
  h.resize(k-1);
  return h.size() >= 3 ? h : std::vector<Eigen::Vector2d>{};
}
}  // namespace

SupportModel::SupportModel(const std::string& path, const std::vector<std::string>& links,
                           const std::vector<std::string>& package_dirs) : urdf_path_(path) {
  if (links.size() < 3) throw std::invalid_argument("at least three support links required");
  const auto urdf = urdf::parseURDFFile(path);
  if (!urdf) throw std::invalid_argument("cannot parse support URDF");
  const auto absolute = fs::absolute(path);
  for (const auto& name : links) {
    const auto link = urdf->getLink(name);
    if (!link || vertices_.count(name)) throw std::invalid_argument("unknown/duplicate support link: " + name);
    auto& vertices = vertices_[name];
    auto collisions = link->collision_array;
    if (collisions.empty() && link->collision) collisions.push_back(link->collision);
    for (const auto& collision : collisions) {
      if (!collision || !collision->geometry || collision->geometry->type != urdf::Geometry::MESH)
        throw std::invalid_argument("stage 1 support requires URDF collision meshes: " + name);
      const auto* mesh = static_cast<const urdf::Mesh*>(collision->geometry.get());
      const Eigen::Vector3d scale(mesh->scale.x, mesh->scale.y, mesh->scale.z);
      if (!scale.allFinite() || scale.minCoeff() <= 0) throw std::invalid_argument("invalid mesh scale");
      const auto& o = collision->origin;
      const Eigen::Quaterniond rotation(o.rotation.w, o.rotation.x, o.rotation.y, o.rotation.z);
      const Eigen::Vector3d translation(o.position.x, o.position.y, o.position.z);
      if (!rotation.coeffs().allFinite() || std::abs(rotation.norm() - 1.0) > 1e-6 || !translation.allFinite())
        throw std::invalid_argument("invalid collision origin");
      for (const auto& p : readStl(resolveMesh(mesh->filename, absolute, package_dirs))) {
        const Eigen::Vector3d transformed = rotation * p.cwiseProduct(scale) + translation;
        if (!transformed.allFinite()) throw std::invalid_argument("invalid transformed mesh vertex");
        vertices.push_back(transformed);
      }
    }
    if (vertices.empty()) throw std::invalid_argument("support link has no collision geometry: " + name);
    std::sort(vertices.begin(), vertices.end(), lessPoint);
    vertices.erase(std::unique(vertices.begin(), vertices.end(), [](const auto& a, const auto& b) {
      return a == b;
    }), vertices.end());
  }
}

SteeringBounds SupportModel::steeringBounds(const CentroidalModel& model,
                                           const std::vector<std::string>& unknown) const {
  const auto urdf = urdf::parseURDFFile(urdf_path_);
  const auto neutral = model.evaluate(model.neutralSnapshot());
  const std::set<std::string> unknown_set(unknown.begin(), unknown.end());
  if (unknown_set.size() != unknown.size()) throw std::invalid_argument("duplicate unknown steering joint");
  // The support frame must be rigid relative to the root after masking these
  // steering joints. This permits caching A,b during arm/body IK.
  for (const auto& item : vertices_) {
    auto link = urdf->getLink(item.first);
    while (link->parent_joint) {
      const auto j = link->parent_joint;
      if (j->type != urdf::Joint::FIXED && !unknown_set.count(j->name) &&
          (!j->limits || j->limits->lower != j->limits->upper))
        throw std::invalid_argument("support has unmasked moving joint: " + j->name);
      link = urdf->getLink(j->parent_link_name);
    }
  }
  SteeringBounds bound;
  for (const auto& name : unknown) {
    const auto joint = urdf->getJoint(name);
    if (!joint || joint->type != urdf::Joint::REVOLUTE)
      throw std::invalid_argument("unknown steering must be a bounded revolute joint: " + name);
    auto parent = urdf->getLink(joint->parent_link_name);
    while (parent->parent_joint) {
      if (parent->parent_joint->type != urdf::Joint::FIXED)
        throw std::invalid_argument("unknown steering must have fixed ancestors: " + name);
      parent = urdf->getLink(parent->parent_joint->parent_link_name);
    }
    const auto& origin = joint->parent_to_joint_origin_transform;
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    T.translation() = Eigen::Vector3d(origin.position.x, origin.position.y, origin.position.z);
    T.linear() = Eigen::Quaterniond(origin.rotation.w, origin.rotation.x, origin.rotation.y, origin.rotation.z).toRotationMatrix();
    T = neutral.link_poses.at(joint->parent_link_name) * T;
    const Eigen::Vector3d axis = (T.linear() * Eigen::Vector3d(joint->axis.x, joint->axis.y, joint->axis.z)).normalized();
    const double tilt = axis.head<2>().norm();
    if (!axis.allFinite() || tilt > 1e-4) throw std::invalid_argument("steering axis is not vertical: " + name);
    const Eigen::Vector3d pivot = T.translation();
    bool found_support = false;
    std::function<void(const urdf::LinkConstSharedPtr&)> visit = [&](const auto& link) {
      if (link->name != joint->child_link_name && link->parent_joint->type != urdf::Joint::FIXED &&
          (!link->parent_joint->limits || link->parent_joint->limits->lower != link->parent_joint->limits->upper))
        throw std::invalid_argument("steering subtree must be rigid: " + name);
      const auto& pose = neutral.link_poses.at(link->name);
      if (link->inertial) {
        const auto& p = link->inertial->origin.position;
        const Eigen::Vector3d r = pose * Eigen::Vector3d(p.x, p.y, p.z) - pivot;
        bound.com_xy += 2.0 * link->inertial->mass * axis.cross(r).norm() / model.mass();
      }
      const auto mesh = vertices_.find(link->name);
      if (mesh != vertices_.end()) {
        found_support = true;
        double low = std::numeric_limits<double>::infinity(), radius = 0;
        for (const auto& v : mesh->second) {
          const Eigen::Vector3d p = pose * v;
          low = std::min(low, p.z()); radius = std::max(radius, axis.cross(p-pivot).norm());
        }
        const double dz = 2.0 * tilt * radius;
        if (dz > 0.0001) throw std::invalid_argument("steering ground-height uncertainty too large");
        double bottom_radius = 0, smin = std::numeric_limits<double>::infinity(), smax = -smin;
        for (const auto& v : mesh->second) {
          const Eigen::Vector3d p = pose * v;
          // Contains every possible lowest-vertex candidate under any rotation:
          // z0 <= min(z0) + 2*maximum_vertical_motion + mesh bottom tolerance.
          if (p.z() <= low + 2.0 * dz + 1e-5) {
            const Eigen::Vector3d r = p - pivot;
            bottom_radius = std::max(bottom_radius, axis.cross(r).norm());
            smin = std::min(smin, axis.dot(r)); smax = std::max(smax, axis.dot(r));
          }
        }
        bound.support_xy = std::max(bound.support_xy, 2.0 * bottom_radius + tilt * (smax-smin));
      }
      for (const auto& child : link->child_links) visit(child);
    };
    visit(urdf->getLink(joint->child_link_name));
    if (!found_support) throw std::invalid_argument("unknown joint is not a support steering joint: " + name);
  }
  return bound;
}

SupportResult SupportModel::evaluate(const CentroidalResult& model, double margin, double tolerance) const {
  if (!std::isfinite(margin) || margin < 0 || !std::isfinite(tolerance) || tolerance <= 0 || !model.com.allFinite())
    throw std::invalid_argument("invalid support query");
  SupportResult out;
  out.signed_margin = std::numeric_limits<double>::quiet_NaN();
  double minimum = std::numeric_limits<double>::infinity(), maximum = -minimum;
  std::vector<Eigen::Vector2d> points;
  for (const auto& entry : vertices_) {
    auto pose = model.link_poses.find(entry.first);
    if (pose == model.link_poses.end() || !pose->second.matrix().allFinite())
      throw std::invalid_argument("missing/invalid support frame: " + entry.first);
    const auto& T = pose->second;
    double lowest = std::numeric_limits<double>::infinity();
    for (const auto& v : entry.second) lowest = std::min(lowest, (T * v).z());
    // Mean of UNIQUE near-lowest vertices is a conservative point-contact
    // approximation, not a wheel-axis origin or an assumed contact force.
    Eigen::Vector3d contact = Eigen::Vector3d::Zero();
    std::size_t count = 0;
    for (const auto& v : entry.second) {
      const Eigen::Vector3d p = T * v;
      if (p.z() <= lowest + 1e-5) { contact += p; ++count; }
    }
    contact /= static_cast<double>(count);
    contact.z() = lowest;
    out.contact_candidates.emplace(entry.first, contact);
    points.push_back(contact.head<2>());
    minimum = std::min(minimum, lowest);
    maximum = std::max(maximum, lowest);
  }
  out.plane_z = minimum;
  if (maximum - minimum > tolerance) {
    out.error = "URDF support extrema are not coplanar; no horizontal support polygon";
    return out;
  }
  out.polygon = hull(points);
  if (out.polygon.empty()) {
    out.error = "degenerate support polygon";
    return out;
  }
  out.A.resize(out.polygon.size(), 2);
  out.b.resize(out.polygon.size());
  for (std::size_t i = 0; i < out.polygon.size(); ++i) {
    const Eigen::Vector2d edge = out.polygon[(i+1) % out.polygon.size()] - out.polygon[i];
    const Eigen::Vector2d normal = Eigen::Vector2d(edge.y(), -edge.x()).normalized();
    out.A.row(i) = normal.transpose();
    out.b[i] = normal.dot(out.polygon[i]) - margin;
  }
  // Check that inward erosion did not eliminate the feasible region.
  bool feasible = false;
  for (Eigen::Index i = 0; i < out.A.rows() && !feasible; ++i) {
    for (Eigen::Index j = i + 1; j < out.A.rows(); ++j) {
      Eigen::Matrix2d pair;
      pair.row(0) = out.A.row(i); pair.row(1) = out.A.row(j);
      if (std::abs(pair.determinant()) < 1e-12) continue;
      const Eigen::Vector2d vertex = pair.inverse() * Eigen::Vector2d(out.b[i], out.b[j]);
      if ((out.A * vertex - out.b).maxCoeff() <= 1e-9) { feasible = true; break; }
    }
  }
  if (!feasible) {
    out.error = "safety margin eliminates support region";
    return out;
  }
  out.geometry_valid = true;
  out.signed_margin = (out.b - out.A * model.com.head<2>()).minCoeff();
  return out;
}
}  // namespace motion_control_pkg::wbc
