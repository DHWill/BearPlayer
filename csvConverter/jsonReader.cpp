#include <nlohmann/json.hpp>
#include <iostream>
#include <vector>
#include <tuple>
#include <nlohmann/json.hpp>
using json = nlohmann::json;

int main() {
  json jsonData = R"([
    {"name": "Object1", "startTime": 10, "endTime": 20},
    {"name": "Object2", "startTime": 15, "endTime": 25},
    {"name": "Object3", "startTime": 5, "endTime": 15}
  ])"_json;

  std::vector<std::tuple<std::string, int, int>> extractedValues;

  for (const auto& obj : jsonData) {
    std::string name = obj["name"];
    int startTime = obj["startTime"];
    int endTime = obj["endTime"];

    extractedValues.push_back(std::make_tuple(name, startTime, endTime));
  }

  // Print the extracted values
  for (const auto& value : extractedValues) {
    std::cout << "Name: " << std::get<0>(value) << ", Start Time: " << std::get<1>(value)
              << ", End Time: " << std::get<2>(value) << std::endl;
  }

  return 0;
}

using json = nlohmann::json;

json jsonData = R"([
  {"name": "Object1", "startTime": 10, "endTime": 20},
  {"name": "Object2", "startTime": 15, "endTime": 25},
  {"name": "Object3", "startTime": 5, "endTime": 15}
])"_json;

std::vector<std::tuple<std::string, int, int>> extractedValues;
for (const auto& obj : jsonData) {
  std::string name = obj["name"];
  int startTime = obj["startTime"];
  int endTime = obj["endTime"];
  
  extractedValues.push_back(std::make_tuple(name, startTime, endTime));
}
