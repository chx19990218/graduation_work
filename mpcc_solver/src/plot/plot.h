// Copyright @2022 HITCSC. All rights reserved.
// Authors: Hongxu Cao (chx19990218@qq.com)

#pragma once

#include <matplotlibcpp.h>

#include <string>
#include <unordered_map>

#include "map.h"
#include "mpcc.h"
#include "resample.h"
#include "search.h"
#include "smooth.h"
#include "obstacle.h"

namespace plt = matplotlibcpp;

struct Option {
  std::string color_;
  std::string line_;
  Option(std::string color, std::string line) : color_(color), line_(line){};
};

class Plot {
 public:
  enum Color {
    Blue = 1,     // 蓝色
    Green = 2,    // 绿色
    Red = 3,      // 红色
    Cyan = 4,     // 青色
    Magenta = 5,  // 品红
    Yellow = 6,   // 黄色
    Black = 7,    // 黑色
    White         // 白色
  };
  enum Line {
    One = 1,    // 实线
    Two = 2,    // 虚线
    Three = 3,  // 虚点线
    Four = 4,   // 点线
    Five = 5,   // 点
    Six = 6,    // 像素点
    Seven = 7,  // 圆点
    Eight = 8,  // 加号点
    Nine = 9,   // 星形点
    Ten         // 五角点
  };
  std::unordered_map<Line, std::string> line_map{
      {One, "-"}, {Two, "--"},  {Three, "-."}, {Four, ":"}, {Five, "."},
      {Six, ","}, {Seven, "o"}, {Eight, "+"},  {Nine, "*"}, {Ten, "p"}};
  std::unordered_map<Color, std::string> color_map{
      {Blue, "b"},    {Green, "g"},  {Red, "r"},   {Cyan, "c"},
      {Magenta, "m"}, {Yellow, "y"}, {Black, "k"}, {White, "w"}};
  void plot(const Map& map, const Search& search, const Smooth& smooth,
            const Resample& resample, const Mpcc& mpcc, const Obstacle& obstacle);
};