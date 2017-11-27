/*
  Given an output file and then a  list of point pair feature histograms,
  outputs a unified histogram and variance statistic the input histograms.
 */

#include <unistd.h>
#include <fstream>
#include <iostream>
#include <random>
#include <stdexcept>
#include <string>
#include <typeinfo>

#include "common/common.hpp"
#include "yaml-cpp/yaml.h"

using namespace std;
using namespace Eigen;

int main(int argc, char** argv) {
  srand(0);

  if (argc == 1) {
    printf(
        "Use: collect_point_pair_feature_histograms <input_file_list_file> "
        "<output_file>\n");
    printf(
        "\t input_file_list_file should be a file with each line containing a "
        "path to a ppfh\n");
    exit(-1);
  }

  ifstream input_file_list_file(argv[1]);
  string output_file = string(argv[2]);

  // Load up first input file to get appropriate array sizing
  string first_input_file;
  getline(input_file_list_file, first_input_file);

  YAML::Node first_ppf = YAML::LoadFile(first_input_file);

  double max_distance = first_ppf["max_distance"].as<double>();
  int n_features = first_ppf["n_features"].as<int>();
  int n_bins_distance = first_ppf["n_bins"]["distance"].as<int>();
  int n_bins_n1_n2 = first_ppf["n_bins"]["n1_n2"].as<int>();
  int n_bins_d_n1 = first_ppf["n_bins"]["d_n1"].as<int>();
  int n_bins_d_n2 = first_ppf["n_bins"]["d_n2"].as<int>();

  int n_bins_total = n_bins_distance * n_bins_n1_n2 * n_bins_d_n1 * n_bins_d_n2;
  printf("Running with %d bins total...\n", n_bins_total);

  // Perform online variance estimation following Welford's online algorithm,
  // as described at
  // https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance#Online_algorithm
  VectorXd means(n_bins_total);
  means.setZero();
  VectorXd M2(n_bins_total);
  M2.setZero();
  VectorXi counts(n_bins_total);
  counts.setZero();

  // And declare these for repeated use.
  VectorXd delta(n_bins_total);
  VectorXd delta2(n_bins_total);

  // Figure out  how many input files we have
  vector<string> input_files;
  string this_input_file;
  while (getline(input_file_list_file, this_input_file)){
    input_files.push_back(this_input_file);
  }
  int total_num_input_files = input_files.size();

  printf("File 0/0");
  fflush(stdout);

  // Factor of 1E6 to avoid possible numeric issues...
  const double kBig = 1E5;
  for (int k = 0; k < total_num_input_files; k++) {
    printf("\rFile %d/%d", k + 1, total_num_input_files);
    fflush(stdout);
    YAML::Node this_ppf = YAML::LoadFile(input_files[k]);

    vector<double> this_ppf_raw_data =
        this_ppf["histogram"].as<vector<double>>();
    if (this_ppf_raw_data.size() != n_bins_total) {
      printf("\nFailure: # bins mismatch between file %s and file %s\n",
             first_input_file.c_str(), input_files[k].c_str());
      printf("\t (%d != %lu)\n", n_bins_total, this_ppf_raw_data.size());
      exit(0);
    }

    auto this_histogram_data_map =
        Map<VectorXd>(this_ppf_raw_data.data(), this_ppf_raw_data.size());

    counts += this_histogram_data_map.cast<int>();

    this_histogram_data_map *= kBig / this_histogram_data_map.sum();
    delta = this_histogram_data_map - means;
    means += delta / (double)(k + 1);
    delta2 = this_histogram_data_map - means;
    M2 += (delta.array() * delta2.array()).matrix();
  }
  VectorXd variances = M2 / (total_num_input_files - 1);

  means /= kBig;
  variances /= kBig;

  // Save out means and variances
  YAML::Emitter out;

  out << YAML::BeginMap;
  {
    out << YAML::Key << "n_features";
    out << YAML::Value << n_features;

    out << YAML::Key << "max_distance";
    out << YAML::Value << max_distance;

    out << YAML::Key << "n_bins";
    out << YAML::BeginMap;
    {
      out << YAML::Key << "distance";
      out << YAML::Value << n_bins_distance;
      out << YAML::Key << "n1_n2";
      out << YAML::Value << n_bins_n1_n2;
      out << YAML::Key << "d_n1";
      out << YAML::Value << n_bins_d_n1;
      out << YAML::Key << "d_n2";
      out << YAML::Value << n_bins_d_n2;
    }
    out << YAML::EndMap;

    out << YAML::Key << "histogram_counts";
    out << YAML::Value << YAML::Flow
        << vector<int>(counts.data(),
                       counts.data() + counts.rows() * counts.cols());

    out << YAML::Key << "histogram_means";
    out << YAML::Value << YAML::Flow
        << vector<double>(means.data(),
                          means.data() + means.rows() * means.cols());

    out << YAML::Key << "histogram_variances";
    out << YAML::Value << YAML::Flow
        << vector<double>(
               variances.data(),
               variances.data() + variances.rows() * variances.cols());
  }
  out << YAML::EndMap;

  ofstream fout(output_file);
  fout << out.c_str();
  fout.close();

  printf("\nDone!\n");
  return 0;
}
