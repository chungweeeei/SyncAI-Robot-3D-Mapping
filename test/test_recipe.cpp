// map_clean.recipe.json's "progress": the one number a console draws a
// running clean's bar from. The backend reads "fraction" and nothing else of
// it, so what is pinned here is what that number may do, not the shares.
#include <gtest/gtest.h>

#include <string>
#include <utility>
#include <vector>

#include "nlohmann/json.hpp"
#include "syncai_mapping/dynamic_removal/recipe.hpp"

namespace
{
namespace dr = syncai_mapping::dynamic_removal;
namespace recipe = syncai_mapping::dynamic_removal::recipe;

// Every Progress call of a clean of `n` keyframes, in the order clean() makes
// them (remover.cpp's cleanImpl).
std::vector<double> fractionsOfAClean(size_t n, bool octomap)
{
  std::vector<double> ret;
  for (size_t i = 1; i <= n; ++i) ret.push_back(recipe::overallFraction("hits", i, n));
  for (size_t i = 1; i <= n; ++i) ret.push_back(recipe::overallFraction("misses", i, n));
  if (octomap) ret.push_back(recipe::overallFraction("tree", 0, 0));
  for (size_t i = 1; i <= n; ++i) ret.push_back(recipe::overallFraction("map", i, n));
  ret.push_back(recipe::overallFraction("write", 0, 0));
  return ret;
}

TEST(RecipeProgress, NeverGoesBackOverAClean)
{
  for (const bool octomap : {true, false}) {
    double last = 0.0;
    for (const double f : fractionsOfAClean(250, octomap)) {
      EXPECT_GE(f, last);
      last = f;
    }
  }
}

// "ok" is what says a clean is done: a bar at 100 % over a clean still
// writing map.pcd would be a lie for as long as the write takes.
TEST(RecipeProgress, StaysUnderOneUntilTheCleanIsOver)
{
  for (const double f : fractionsOfAClean(250, true)) {
    EXPECT_GE(f, 0.0);
    EXPECT_LT(f, 1.0);
  }
}

TEST(RecipeProgress, StartsAtZero)
{
  EXPECT_DOUBLE_EQ(recipe::overallFraction("hits", 0, 250), 0.0);
}

// A skipped patch counts in "hits" (keyframes + skipped), so done can only
// reach total; past it must not carry the bar into the next stage's share.
TEST(RecipeProgress, AStageNeverRunsPastItsShare)
{
  EXPECT_DOUBLE_EQ(
    recipe::overallFraction("hits", 300, 250), recipe::overallFraction("hits", 250, 250));
  EXPECT_LE(recipe::overallFraction("hits", 300, 250), recipe::overallFraction("misses", 0, 250));
}

// The caller writes no mark for a stage it gets no fraction for.
TEST(RecipeProgress, AnUnknownStageHasNoFraction)
{
  EXPECT_LT(recipe::overallFraction("polish", 1, 2), 0.0);
}

TEST(RecipeDocument, ConvertingCarriesProgressOnlyWhenGiven)
{
  const dr::Params params;
  const std::string started_at = "2026-11-19T03:12:45Z";

  const auto bare = nlohmann::json::parse(recipe::converting(params, started_at));
  EXPECT_EQ(bare.at("status"), "converting");
  EXPECT_EQ(bare.at("started_at"), started_at);
  EXPECT_FALSE(bare.contains("progress"));

  const auto marked =
    nlohmann::json::parse(recipe::converting(params, started_at, "misses", 0.4237));
  EXPECT_EQ(marked.at("status"), "converting");
  EXPECT_EQ(marked.at("started_at"), started_at);
  EXPECT_EQ(marked.at("progress").at("stage"), "misses");
  EXPECT_DOUBLE_EQ(marked.at("progress").at("fraction").get<double>(), 0.424);
  // What a reader ages a hard-killed clean out by, and what it tells an
  // octomap clean by: both must outlive a mark.
  EXPECT_EQ(marked.at("params"), bare.at("params"));
}

TEST(RecipeDocument, AFractionOutOfRangeIsWrittenInRange)
{
  const dr::Params params;
  for (const auto & [given, written] :
       std::vector<std::pair<double, double>>{{-0.2, 0.0}, {1.7, 1.0}}) {
    const auto j = nlohmann::json::parse(recipe::converting(params, "t", "map", given));
    EXPECT_DOUBLE_EQ(j.at("progress").at("fraction").get<double>(), written);
  }
}

}  // namespace
