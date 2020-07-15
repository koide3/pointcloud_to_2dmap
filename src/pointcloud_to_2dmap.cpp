#include <iostream>
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>

#include <opencv2/opencv.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>


class MapGenerater {
public:
  MapGenerater(boost::program_options::variables_map& vm)
  : resolution(vm["resolution"].as<double>()),
    m2pix(1.0 / resolution),
    map_width(vm["map_width"].as<int>()),
    map_height(vm["map_height"].as<int>()),
    min_points_in_pix(vm["min_points_in_pix"].as<int>()),
    max_points_in_pix(vm["max_points_in_pix"].as<int>()),
    min_height(vm["min_height"].as<double>()),
    max_height(vm["max_height"].as<double>())
  {}

  cv::Mat generate(const pcl::PointCloud<pcl::PointXYZ>& cloud) const {
    cv::Mat map(map_height, map_width, CV_32SC1, cv::Scalar::all(0));

    for(const auto& point: cloud) {
      if(point.z < min_height || point.z > max_height) {
        continue;
      }

      int x = point.x * m2pix + map_width / 2;
      int y = -point.y * m2pix + map_width / 2;

      if(x < 0 || x >= map_width || y < 0 || y >= map_height) {
        continue;
      }

      map.at<int>(y, x) ++;
    }

    map -= min_points_in_pix;
    map.convertTo(map, CV_8UC1, - 255.0 / (max_points_in_pix - min_points_in_pix),  255);

    return map;
  }

public:
  const double resolution;    // meters per pixel
  const double m2pix;         // inverse resolution (pix/m)
  const int map_width;
  const int map_height;

  const int min_points_in_pix;
  const int max_points_in_pix;
  const double min_height;
  const double max_height;
};


int main(int argc, char** argv) {
  namespace po = boost::program_options;
  po::options_description description("pointcloud_to_2dmap");
  description.add_options()
    ("help", "Produce help message")
    ("resolution,r", po::value<double>()->default_value(0.1), "Pixel resolution (meters / pix)")
    ("map_width,w", po::value<int>()->default_value(1024), "Map width [pix]")
    ("map_height,h", po::value<int>()->default_value(1024), "Map height [pix]")
    ("min_points_in_pix", po::value<int>()->default_value(2), "Min points in a occupied pix")
    ("max_points_in_pix", po::value<int>()->default_value(5), "Max points in a pix for saturation")
    ("min_height", po::value<double>()->default_value(0.5), "Min height of clipping range")
    ("max_height", po::value<double>()->default_value(1.0), "Max height of clipping range")
    ("input_pcd", po::value<std::string>(), "Input PCD file")
    ("dest_directory", po::value<std::string>(), "Destination directory")
  ;

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv)
    .options(description)
    .positional(
      po::positional_options_description()
        .add("input_pcd", 1)
        .add("dest_directory", 1)
    ).run(), vm
  );
  po::notify(vm);

  if (vm.count("help")) {
      std::cout << description << std::endl;
      return 1;
  }

  std::cout << "input_pcd     :" << vm["input_pcd"].as<std::string>() << std::endl;
  std::cout << "dest_directory:" << vm["dest_directory"].as<std::string>() << std::endl;
  std::cout << "resolution    :" << vm["resolution"].as<double>() << std::endl;
  auto cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  if(pcl::io::loadPCDFile(vm["input_pcd"].as<std::string>(), *cloud)) {
    std::cerr << "failed to open the input cloud" << std::endl;
    return 1;
  }

  MapGenerater generater(vm);
  cv::Mat map = generater.generate(*cloud);

  std::string destination = vm["dest_directory"].as<std::string>();
  if(!boost::filesystem::exists(destination)) {
    boost::filesystem::create_directories(destination);
  }

  cv::imwrite(destination + "/map.png", map);
  
  std::ofstream ofs(destination + "/map.yaml");
  ofs << "image: map.png" << std::endl;
  ofs << "resolution: " << generater.resolution << std::endl;
  ofs << "origin: [" << -generater.resolution * generater.map_width / 2 << ", " << -generater.resolution * generater.map_height / 2 << ", 0.0]" << std::endl;
  ofs << "occupied_thresh: 0.5" << std::endl;
  ofs << "free_thresh: 0.2" << std::endl;
  ofs << "negate: 0" << std::endl;

  return 0;
}