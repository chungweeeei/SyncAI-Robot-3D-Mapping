#include "syncai_mapping/pgos/map_writer.h"

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>
#include <system_error>

void writeMergedMapPcd(
  const std::filesystem::path & path, const std::vector<KeyPoseWithCloud> & key_poses)
{
  size_t total = 0;
  for (const KeyPoseWithCloud & kp : key_poses) total += kp.body_cloud->size();
  if (total > static_cast<size_t>(std::numeric_limits<int>::max()))
    throw std::runtime_error("map too large for one PCD: " + std::to_string(total) + " points");

  // PCL's own header for an unorganised cloud of `total` points, exactly what savePCDFileBinary
  // writes for the merged cloud (WIDTH from the cloud, VIEWPOINT from its default sensor pose).
  CloudType shape;
  shape.width = static_cast<uint32_t>(total);
  shape.height = 1;
  const std::string header =
    pcl::PCDWriter::generateHeader<PointType>(shape, static_cast<int>(total)) + "DATA binary\n";

  const std::filesystem::path tmp = path.string() + ".save.tmp";
  auto fail = [&tmp](const std::string & what) {
    std::error_code ec;
    std::filesystem::remove(tmp, ec);
    throw std::runtime_error(what);
  };

  {
    std::ofstream out(tmp, std::ios::binary | std::ios::trunc);
    if (!out) fail("cannot open " + tmp.string());
    out.write(header.data(), static_cast<std::streamsize>(header.size()));

    // Binary PCD data is the fields packed, no padding: x y z intensity, 16 B a point, where
    // pcl::PointXYZI is 32 B in memory.
    CloudType world;
    std::vector<float> packed;
    for (const KeyPoseWithCloud & kp : key_poses) {
      // The same call the in-memory merge made, so the floats are identical.
      pcl::transformPointCloud(*kp.body_cloud, world, kp.t_global, Eigen::Quaterniond(kp.r_global));
      packed.resize(world.size() * 4);
      for (size_t i = 0; i < world.size(); i++) {
        const PointType & p = world.points[i];
        packed[4 * i + 0] = p.x;
        packed[4 * i + 1] = p.y;
        packed[4 * i + 2] = p.z;
        packed[4 * i + 3] = p.intensity;
      }
      out.write(
        reinterpret_cast<const char *>(packed.data()),
        static_cast<std::streamsize>(packed.size() * sizeof(float)));
      if (!out) fail("write failed: " + tmp.string());
    }
    out.close();
    if (!out) fail("write failed: " + tmp.string());
  }

  std::error_code ec;
  std::filesystem::rename(tmp, path, ec);
  if (ec) fail("cannot rename " + tmp.string() + ": " + ec.message());
}

void writeKeyframes(
  const std::filesystem::path & dir, const std::vector<KeyPoseWithCloud> & key_poses,
  const std::vector<size_t> & session_starts)
{
  const std::filesystem::path patches_dir = dir / "patches";
  std::filesystem::create_directories(patches_dir);

  {
    // Opening it truncates an existing poses.txt.
    std::ofstream txt_file(dir / "poses.txt");
    for (size_t i = 0; i < key_poses.size(); i++) {
      std::string patch_name = std::to_string(i) + ".pcd";
      std::filesystem::path patch_path = patches_dir / patch_name;
      pcl::io::savePCDFileBinary(patch_path.string(), *key_poses[i].body_cloud);
      Eigen::Quaterniond q(key_poses[i].r_global);
      V3D t = key_poses[i].t_global;
      txt_file << patch_name << " " << t.x() << " " << t.y() << " " << t.z() << " " << q.w() << " "
               << q.x() << " " << q.y() << " " << q.z() << std::endl;
    }
    txt_file.close();
    // The patches throw by themselves (pcl); a full disk under poses.txt would otherwise leave a
    // short list beside a complete patches/, which a resume reads as a smaller map.
    if (!txt_file) throw std::runtime_error("write failed: " + (dir / "poses.txt").string());
  }

  writeSessions(dir, key_poses.size(), session_starts);
}
