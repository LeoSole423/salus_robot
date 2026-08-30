#include <gtest/gtest.h>
#include "salus_navigation_costmap/vector_keepout_geometry.hpp"
using namespace salus_navigation_costmap;

static Polygon square(double x, double y, double size) { Polygon p; p.id = "zone"; p.outer = {{x,y},{x+size,y},{x+size,y+size},{x,y+size}}; p.bounds = bounds_of(p.outer); return p; }
static RasterSpec spec(double x = 0, double y = 0, unsigned w = 10, unsigned h = 10) { return {{x,y,x+w,y+h}, 1.0, w, h}; }
TEST(VectorKeepoutGeometry, RasterizesCoreAndClipsToWindow) { auto costs = rasterize({square(2,2,3)}, spec(), {}); EXPECT_EQ(costs[2 * 10 + 2], 254); EXPECT_EQ(costs[0], 0); EXPECT_EQ(costs.size(), 100u); }
TEST(VectorKeepoutGeometry, HolesAreNotKeepoutCore) { auto p = square(1,1,8); p.holes = {{{3,3},{7,3},{7,7},{3,7}}}; auto costs = rasterize({p}, spec(), {}); EXPECT_LT(costs[4 * 10 + 4], 254); EXPECT_EQ(costs[1 * 10 + 1], 254); }
TEST(VectorKeepoutGeometry, HaloUsesLegacyExponentialProfile) { CostProfile profile; profile.halo_radius_m = 3.0; profile.halo_edge_cost = 60; profile.halo_min_cost = 10; auto costs = rasterize({square(4,4,2)}, spec(), profile); EXPECT_EQ(costs[4 * 10 + 4], 254); EXPECT_GT(costs[4 * 10 + 3], costs[4 * 10 + 1]); EXPECT_GE(costs[4 * 10 + 1], 10); }
TEST(VectorKeepoutGeometry, FarPolygonsDoNotContribute) { auto costs = rasterize({square(1000,1000,5)}, spec(), {}); for (auto value : costs) EXPECT_EQ(value, 0); }
TEST(VectorKeepoutGeometry, MultiplePolygonsAreMergedByMaximum) { CostProfile profile; profile.halo_radius_m = 2.0; auto costs = rasterize({square(2,2,2), square(6,2,2)}, spec(), profile); EXPECT_EQ(costs[2 * 10 + 2], 254); EXPECT_EQ(costs[2 * 10 + 6], 254); EXPECT_GT(costs[2 * 10 + 4], 0); }
