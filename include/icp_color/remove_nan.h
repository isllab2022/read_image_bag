#ifndef MY_REMOVE_NAN_H_
#define MY_REMOVE_NAN_H_

 #include <pcl/pcl_macros.h>  // for pcl_isfinite -- PCL v1.8
// //#include <pcl/common/point_tests.h> // for pcl::isFinite -- PCL v1.11
#include <pcl/filters/filter.h>


namespace abc
{
  void removeNaNFPFHFromPointCloud (const pcl::PointCloud<pcl::FPFHSignature33> &cloud_in, 
                                    pcl::PointCloud<pcl::FPFHSignature33> &cloud_out, 
                                    std::vector<int> &index);  // PCL v1.8

  void removeNaNSHOTFromPointCloud (const pcl::PointCloud<pcl::SHOT352> &cloud_in, 
                                    pcl::PointCloud<pcl::SHOT352> &cloud_out, 
                                    std::vector<int> &index);  // PCL v1.8

  void removeNaNSHOTColorFromPointCloud (const pcl::PointCloud<pcl::SHOT1344> &cloud_in, 
                                         pcl::PointCloud<pcl::SHOT1344> &cloud_out, 
                                         std::vector<int> &index);  // PCL v1.8

  template <typename PointT1>
  void removePointsFromPointCloud (const pcl::PointCloud<PointT1> &cloud_in, 
                                pcl::PointCloud<PointT1> &cloud_out,
                                std::vector<int> &index);

  template <typename PointT1>
  void removeNaNFromPointCloudBruteForce (const pcl::PointCloud<PointT1> &cloud_in,
                                     pcl::PointCloud<PointT1> &cloud_out,
                                     std::vector<int> &index);

  template <typename PointT1>
  void removeNaNRGBFromPointCloud (const pcl::PointCloud<PointT1> &cloud_in,
                              pcl::PointCloud<PointT1> &cloud_out,
                              std::vector<int> &index);
}


void abc::removeNaNFPFHFromPointCloud (const pcl::PointCloud<pcl::FPFHSignature33> &cloud_in, 
                                       pcl::PointCloud<pcl::FPFHSignature33> &cloud_out,
                                       std::vector<int> &index)
{
  // If the clouds are not the same, prepare the output
  if (&cloud_in != &cloud_out)
  {
    cloud_out.header = cloud_in.header;
    cloud_out.points.resize (cloud_in.points.size ());
  }
  // Reserve enough space for the indices
  index.resize (cloud_in.points.size ());
  size_t j = 0;

  // If the data is dense, we don't need to check for NaN
  if (cloud_in.is_dense)
  {
    // Simply copy the data
    cloud_out = cloud_in;
    for (j = 0; j < cloud_out.points.size (); ++j)
      index[j] = static_cast<int>(j);
  }
  else
  {
    for (size_t i = 0; i < cloud_in.points.size (); ++i)
    {
      if (!pcl_isfinite (cloud_in.points[i].histogram[0]))  // PCL v1.8
        continue;
      cloud_out.points[j] = cloud_in.points[i];
      index[j] = static_cast<int>(i);
      j++;
    }
    if (j != cloud_in.points.size ())
    {
      // Resize to the correct size
      cloud_out.points.resize (j);
      index.resize (j);
    }

    cloud_out.height = 1;
    cloud_out.width  = static_cast<std::uint32_t>(j);

    // Removing bad points => dense (note: 'dense' doesn't mean 'organized')
    cloud_out.is_dense = true;
  }
}


void abc::removeNaNSHOTFromPointCloud (const pcl::PointCloud<pcl::SHOT352> &cloud_in, 
                                       pcl::PointCloud<pcl::SHOT352> &cloud_out,
                                       std::vector<int> &index)
{
  // If the clouds are not the same, prepare the output
  if (&cloud_in != &cloud_out)
  {
    cloud_out.header = cloud_in.header;
    cloud_out.points.resize (cloud_in.points.size ());
  }
  // Reserve enough space for the indices
  index.resize (cloud_in.points.size ());
  size_t j = 0;

  // If the data is dense, we don't need to check for NaN
  if (cloud_in.is_dense)
  {
    // Simply copy the data
    cloud_out = cloud_in;
    for (j = 0; j < cloud_out.points.size (); ++j)
      index[j] = static_cast<int>(j);
  }
  else
  {
    for (size_t i = 0; i < cloud_in.points.size (); ++i)
    {
      if (!std::isfinite (cloud_in.points[i].descriptor[0]) ||    // PCL v1.8
          !std::isfinite (cloud_in.points[i].rf[0]))              // PCL v1.8
        continue;
      cloud_out.points[j] = cloud_in.points[i];
      index[j] = static_cast<int>(i);
      j++;
    }
    if (j != cloud_in.points.size ())
    {
      // Resize to the correct size
      cloud_out.points.resize (j);
      index.resize (j);
    }

    cloud_out.height = 1;
    cloud_out.width  = static_cast<std::uint32_t>(j);

    // Removing bad points => dense (note: 'dense' doesn't mean 'organized')
    cloud_out.is_dense = true;
  }
}


void abc::removeNaNSHOTColorFromPointCloud (const pcl::PointCloud<pcl::SHOT1344> &cloud_in, 
                                            pcl::PointCloud<pcl::SHOT1344> &cloud_out,
                                            std::vector<int> &index)
{
  // If the clouds are not the same, prepare the output
  if (&cloud_in != &cloud_out)
  {
    cloud_out.header = cloud_in.header;
    cloud_out.points.resize (cloud_in.points.size ());
  }
  // Reserve enough space for the indices
  index.resize (cloud_in.points.size ());
  size_t j = 0;

  // If the data is dense, we don't need to check for NaN
  if (cloud_in.is_dense)
  {
    // Simply copy the data
    cloud_out = cloud_in;
    for (j = 0; j < cloud_out.points.size (); ++j)
      index[j] = static_cast<int>(j);
  }
  else
  {
    for (size_t i = 0; i < cloud_in.points.size (); ++i)
    {
      if (!std::isfinite (cloud_in.points[i].descriptor[0]) ||    // PCL v1.8
          !std::isfinite (cloud_in.points[i].rf[0]))              // PCL v1.8
        continue;
      cloud_out.points[j] = cloud_in.points[i];
      index[j] = static_cast<int>(i);
      j++;
    }
    if (j != cloud_in.points.size ())
    {
      // Resize to the correct size
      cloud_out.points.resize (j);
      index.resize (j);
    }

    cloud_out.height = 1;
    cloud_out.width  = static_cast<std::uint32_t>(j);

    // Removing bad points => dense (note: 'dense' doesn't mean 'organized')
    cloud_out.is_dense = true;
  }
}


template <typename PointT1>
void abc::removePointsFromPointCloud (const pcl::PointCloud<PointT1> &cloud_in, 
                                      pcl::PointCloud<PointT1> &cloud_out,
                                      std::vector<int> &index)
{
  // Copy data
  size_t j = 0;
  for (size_t i : index)
  {
    cloud_out.points[j] = cloud_in.points[i];
    j++;
  }
  if (j != cloud_in.points.size ())
  {
    // Resize to the correct size
    cloud_out.points.resize (j);
  }

  cloud_out.height = 1;
  cloud_out.width  = static_cast<std::uint32_t>(j);

  // Removing bad points => dense (note: 'dense' doesn't mean 'organized')
  cloud_out.is_dense = true;
}


template <typename PointT1>
void abc::removeNaNFromPointCloudBruteForce (const pcl::PointCloud<PointT1> &cloud_in,
                                             pcl::PointCloud<PointT1> &cloud_out,
                                             std::vector<int> &index)
{
  // If the clouds are not the same, prepare the output
  if (&cloud_in != &cloud_out)
  {
    cloud_out.header = cloud_in.header;
    cloud_out.points.resize (cloud_in.points.size ());
  }
  // Reserve enough space for the indices
  index.resize (cloud_in.points.size ());
  size_t j = 0;

  // Check NaN anyway, regardless of the density of the data
  for (size_t i = 0; i < cloud_in.points.size (); ++i)
  {
    if (!std::isfinite (cloud_in.points[i].x) ||
        !std::isfinite (cloud_in.points[i].y) ||
        !std::isfinite(cloud_in.points[i].z))
      continue;
    cloud_out.points[j] = cloud_in.points[i];
    index[j] = static_cast<int>(i);
    j++;
  }
  if (j != cloud_in.points.size ())
  {
    // Resize to the correct size
    cloud_out.points.resize (j);
    index.resize (j);
  }

  cloud_out.height = 1;
  cloud_out.width  = static_cast<std::uint32_t>(j);

  // Removing bad points => dense (note: 'dense' doesn't mean 'organized')
  cloud_out.is_dense = true;
}


template <typename PointT1>
void abc::removeNaNRGBFromPointCloud (const pcl::PointCloud<PointT1> &cloud_in,
                                      pcl::PointCloud<PointT1> &cloud_out,
                                      std::vector<int> &index)
{
  // If the clouds are not the same, prepare the output
  if (&cloud_in != &cloud_out)
  {
    cloud_out.header = cloud_in.header;
    cloud_out.points.resize (cloud_in.points.size ());
  }
  // Reserve enough space for the indices
  index.resize (cloud_in.points.size ());
  size_t j = 0;

  // Check NaN anyway, regardless of the density of the data
  for (size_t i = 0; i < cloud_in.points.size (); ++i)
  {
    if (cloud_in.points[i].rgba == 0)
      continue;
    cloud_out.points[j] = cloud_in.points[i];
    index[j] = static_cast<int>(i);
    j++;
  }
  if (j != cloud_in.points.size ())
  {
    // Resize to the correct size
    cloud_out.points.resize (j);
    index.resize (j);
  }

  cloud_out.height = 1;
  cloud_out.width  = static_cast<std::uint32_t>(j);

  // Removing bad points => dense (note: 'dense' doesn't mean 'organized')
  cloud_out.is_dense = true;
}


#endif  //#ifndef MY_REMOVE_NAN_H_
