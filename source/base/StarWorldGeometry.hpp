#pragma once

#include "StarPoly.hpp"

namespace Star {

STAR_CLASS(WorldGeometry);

// Utility class for dealing with the non-euclidean nature of the World.
// Handles the surprisingly complex job of deciding intersections and splitting
// geometry across the world wrap boundary.
class WorldGeometry {
public:
  // A null WorldGeometry will have diff / wrap methods etc be the normal
  // euclidean variety.
  WorldGeometry();
  WorldGeometry(unsigned width, unsigned height);
  //WorldGeometry(Vec2U const& size);
  WorldGeometry(Vec2U const& size, bool const& xWrap, bool const& yWrap);

  bool isNull();

  bool operator==(WorldGeometry const& other) const;
  bool operator!=(WorldGeometry const& other) const;

  unsigned width() const;
  unsigned height() const;
  Vec2U size() const;
  
  int xwrap(int x) const;
  float xwrap(float x) const;
  
  Vec2F xwrap(Vec2F const& pos) const;
  Vec2I xwrap(Vec2I const& pos) const;
  
  int xlimit(int x) const;
  float xlimit(float x) const;
  // Wraps and clamps only x component.
  Vec2F xlimit(Vec2F const& pos) const;
  Vec2I xlimit(Vec2I const& pos) const;
  
  int ywrap(int y) const;
  float ywrap(float y) const;
  
  Vec2F ywrap(Vec2F const& pos) const;
  Vec2I ywrap(Vec2I const& pos) const;
  
  int ylimit(int y) const;
  float ylimit(float y) const;
  // Wraps and clamps only y component.
  Vec2F ylimit(Vec2F const& pos) const;
  Vec2I ylimit(Vec2I const& pos) const;

  // Wraps and clamps position
  Vec2F limit(Vec2F const& pos) const;
  // Only wraps position
  Vec2F wrap(Vec2F const& pos) const;
  Vec2I wrap(Vec2I const& pos) const;

  bool crossesWrapX(float xMin, float xMax) const;
  bool crossesWrapY(float yMin, float yMax) const;
  bool crossesWrap(Vec2F min, Vec2F max) const;

  // Do these two indexes point to the same location
  bool equal(Vec2I const& p1, Vec2I const& p2) const;

  // Same as wrap, returns unsigned type.
  unsigned xindex(int x) const;
  unsigned yindex(int x) const;
  Vec2U index(Vec2I const& i) const;

  // returns right only distance from x2 to x1 (or x1 - x2).  Always positive.
  int xpdiff(int x1, int x2) const;

  // Shortest difference between two given points.  Always returns diff on the
  // "side" that x1 is on.
  float xdiff(float x1, float x2) const;
  int xdiff(int x1, int x2) const;

  // returns right only distance from y2 to y1 (or y1 - y2).  Always positive.
  int ypdiff(int y1, int y2) const;

  // Shortest difference between two given points.  Always returns diff on the
  // "side" that y1 is on.
  float ydiff(float y1, float y2) const;
  int ydiff(int y1, int y2) const;

  // Same but for 2d vectors
  Vec2F diff(Vec2F const& p1, Vec2F const& p2) const;
  Vec2I diff(Vec2I const& p1, Vec2I const& p2) const;

  // Midpoint of the shortest line connecting two points.
  Vec2F midpoint(Vec2F const& p1, Vec2F const& p2) const;

  function<float(float, float)> xDiffFunction() const;
  function<float(float, float)> yDiffFunction() const;
  function<Vec2F(Vec2F, Vec2F)> diffFunction() const;
  function<float(float, float, float)> xLerpFunction(Maybe<float> discontinuityThreshold = {}) const;
  function<float(float, float, float)> yLerpFunction(Maybe<float> discontinuityThreshold = {}) const;
  function<Vec2F(float, Vec2F, Vec2F)> lerpFunction(Maybe<float> discontinuityThreshold = {}) const;

  // Wrapping functions are not guaranteed to work for objects larger than
  // worldWidth / 2.  Bad things can happen.

  // Split the given Rect across world boundaries.
  StaticList<RectF, 2> splitRectX(RectF const& bbox) const;
  StaticList<RectF, 2> splitRectY(RectF const& bbox) const;
  StaticList<RectF, 4> splitRect(RectF const& bbox) const;
  // Split the given Rect after translating it by position.
  StaticList<RectF, 4> splitRect(RectF bbox, Vec2F const& position) const;

  StaticList<RectI, 2> splitRectX(RectI bbox) const;
  StaticList<RectI, 2> splitRectY(RectI bbox) const;
  StaticList<RectI, 4> splitRect(RectI bbox) const;

  // Same but for Line
  StaticList<Line2F, 2> splitLineX(Line2F line, bool preserveDirection = false) const;
  StaticList<Line2F, 2> splitLineY(Line2F line, bool preserveDirection = false) const;
  StaticList<Line2F, 4> splitLine(Line2F line, bool preserveDirection = false) const;
  StaticList<Line2F, 4> splitLine(Line2F line, Vec2F const& position, bool preserveDirection = false) const;

  // Same but for Poly
  StaticList<PolyF, 2> splitPolyX(PolyF const& poly) const;
  StaticList<PolyF, 2> splitPolyY(PolyF const& poly) const;
  StaticList<PolyF, 4> splitPoly(PolyF const& poly) const;
  StaticList<PolyF, 4> splitPoly(PolyF poly, Vec2F const& position) const;

  // Split a horizontal region of the world across the world wrap point.
  // (Only used by weather)
  StaticList<Vec2I, 2> splitXRegion(Vec2I const& region) const;
  StaticList<Vec2F, 2> splitXRegion(Vec2F const& region) const;

  bool rectContains(RectF const& rect1, Vec2F const& pos) const;
  bool rectIntersectsRect(RectF const& rect1, RectF const& rect2) const;
  RectF rectOverlap(RectF const& rect1, RectF const& rect2) const;
  bool polyContains(PolyF const& poly, Vec2F const& pos) const;
  float polyOverlapArea(PolyF const& poly1, PolyF const& poly2) const;

  bool lineIntersectsRect(Line2F const& line, RectF const& rect) const;
  bool lineIntersectsPoly(Line2F const& line, PolyF const& poly) const;
  bool polyIntersectsPoly(PolyF const& poly1, PolyF const& poly2) const;

  bool rectIntersectsCircle(RectF const& rect, Vec2F const& center, float radius) const;
  bool lineIntersectsCircle(Line2F const& line, Vec2F const& center, float radius) const;

  Maybe<Vec2F> lineIntersectsPolyAt(Line2F const& line, PolyF const& poly) const;

  // Returns the distance from a point to any part of the given poly
  float polyDistance(PolyF const& poly, Vec2F const& point) const;

  // Produces a point that is on the same "side" of the world as the source point.
  int nearestToX(int source, int target) const;
  float nearestToX(float source, float target) const;
  int nearestToY(int source, int target) const;
  float nearestToY(float source, float target) const;
  Vec2I nearestTo(Vec2I const& source, Vec2I const& target) const;
  Vec2F nearestTo(Vec2F const& source, Vec2F const& target) const;

  Vec2F nearestCoordInBox(RectF const& box, Vec2F const& pos) const;
  Vec2F diffToNearestCoordInBox(RectF const& box, Vec2F const& pos) const;
  
  bool wrapsX() const;
  bool wrapsY() const;

private:
  Vec2U m_size;
  bool m_xWrap;
  bool m_yWrap;
};

inline WorldGeometry::WorldGeometry()
  : m_size(Vec2U()), m_xWrap(false), m_yWrap(false) {}

inline WorldGeometry::WorldGeometry(unsigned width, unsigned height)
  : m_size(width, height), m_xWrap(width != 0), m_yWrap(false) {}

/*inline WorldGeometry::WorldGeometry(Vec2U const& size)
  : m_size(size), m_xWrap(size[1] != 0), m_yWrap(false) {}*/
  
inline WorldGeometry::WorldGeometry(Vec2U const& size, bool const& xWrap, bool const& yWrap)
  : m_size(size) {
    m_xWrap = xWrap;
    m_yWrap = yWrap;
  }

inline bool WorldGeometry::isNull() {
  return m_size == Vec2U();
}

inline bool WorldGeometry::operator==(WorldGeometry const& other) const {
  return m_size == other.m_size && m_xWrap == other.m_xWrap && m_yWrap == other.m_yWrap;
}

inline bool WorldGeometry::operator!=(WorldGeometry const& other) const {
  return m_size != other.m_size || m_xWrap != other.m_xWrap || m_yWrap != other.m_yWrap;
}

inline unsigned WorldGeometry::width() const {
  return m_size[0];
}

inline unsigned WorldGeometry::height() const {
  return m_size[1];
}

inline bool WorldGeometry::wrapsX() const {
  return m_xWrap;
}

inline bool WorldGeometry::wrapsY() const {
  return m_yWrap;
}

inline Vec2U WorldGeometry::size() const {
  return m_size;
}

inline int WorldGeometry::xlimit(int x) const {
  if (m_size[0] == 0)
    return x;
  else if (!m_xWrap)
    return clamp<int>(x, 0, m_size[0]);
  else
    return pmod<int>(x, m_size[0]);
}

inline float WorldGeometry::xlimit(float x) const {
  if (m_size[0] == 0)
    return x;
  else if (!m_xWrap)
    return clamp<float>(x, 0, std::nextafter(m_size[0], 0.0f));
  else
    return pfmod<float>(x, m_size[0]);
}

inline Vec2I WorldGeometry::xlimit(Vec2I const& pos) const {
  return {xlimit(pos[0]), pos[1]};
}

inline Vec2F WorldGeometry::xlimit(Vec2F const& pos) const {
  return {xlimit(pos[0]), pos[1]};
}

inline int WorldGeometry::ylimit(int y) const {
  if (m_size[1] == 0)
    return y;
  else if (!m_yWrap)
    return clamp<int>(y, 0, m_size[1]);
  else
    return pmod<int>(y, m_size[1]);
}

inline float WorldGeometry::ylimit(float y) const {
  if (m_size[1] == 0)
    return y;
  else if (!m_yWrap)
    return clamp<float>(y, 0, std::nextafter(m_size[1], 0.0f));
  else
    return pfmod<float>(y, m_size[1]);
}

inline Vec2I WorldGeometry::ylimit(Vec2I const& pos) const {
  return {pos[0], ylimit(pos[1])};
}

inline Vec2F WorldGeometry::ylimit(Vec2F const& pos) const {
  return {pos[0], ylimit(pos[1])};
}

inline int WorldGeometry::xwrap(int x) const {
  if (!m_xWrap)
    return x;
  else
    return pmod<int>(x, m_size[0]);
}

inline float WorldGeometry::xwrap(float x) const {
  if (!m_xWrap)
    return x;
  else
    return pfmod<float>(x, m_size[0]);
}

inline Vec2I WorldGeometry::xwrap(Vec2I const& pos) const {
  return {xwrap(pos[0]), pos[1]};
}

inline Vec2F WorldGeometry::xwrap(Vec2F const& pos) const {
  return {xwrap(pos[0]), pos[1]};
}

inline int WorldGeometry::ywrap(int y) const {
  if (!m_yWrap)
    return y;
  else
    return pmod<int>(y, m_size[1]);
}

inline float WorldGeometry::ywrap(float y) const {
  if (!m_yWrap)
    return y;
  else
    return pfmod<float>(y, m_size[1]);
}

inline Vec2I WorldGeometry::ywrap(Vec2I const& pos) const {
  return {pos[0], ywrap(pos[1])};
}

inline Vec2F WorldGeometry::ywrap(Vec2F const& pos) const {
  return {pos[0], ywrap(pos[1])};
}

inline Vec2F WorldGeometry::limit(Vec2F const& pos) const {
  return {xlimit(pos[0]), ylimit(pos[1])};
}

inline Vec2F WorldGeometry::wrap(Vec2F const& pos) const {
  return {xwrap(pos[0]), ywrap(pos[1])};
}

inline Vec2I WorldGeometry::wrap(Vec2I const& pos) const {
  return {xwrap(pos[0]), ywrap(pos[1])};
}

inline bool WorldGeometry::crossesWrapX(float xMin, float xMax) const {
  return m_xWrap && xwrap(xMax) < xwrap(xMin);
}

inline bool WorldGeometry::crossesWrapY(float yMin, float yMax) const {
  return m_yWrap && ywrap(yMax) < ywrap(yMin);
}

inline bool WorldGeometry::crossesWrap(Vec2F min, Vec2F max) const {
  return crossesWrapX(min[0],max[0]) || crossesWrapY(min[1],max[1]);
}

inline bool WorldGeometry::equal(Vec2I const& p1, Vec2I const& p2) const {
  return index(p1) == index(p2);
}

inline unsigned WorldGeometry::xindex(int x) const {
  return (unsigned)xwrap(x);
}

inline unsigned WorldGeometry::yindex(int y) const {
  return (unsigned)ywrap(y);
}

inline Vec2U WorldGeometry::index(Vec2I const& i) const {
  return Vec2U(xwrap(i[0]), ywrap(i[1]));
}

inline int WorldGeometry::xpdiff(int x1, int x2) const {
  if (!m_xWrap)
    return x1 - x2;
  else
    return pmod<int>(x1 - x2, m_size[0]);
}

inline float WorldGeometry::xdiff(float x1, float x2) const {
  if (!m_xWrap)
    return x1 - x2;
  else
    return wrapDiffF<float>(x1, x2, m_size[0]);
}

inline int WorldGeometry::xdiff(int x1, int x2) const {
  if (!m_xWrap)
    return x1 - x2;
  else
    return wrapDiff<int>(x1, x2, m_size[0]);
}

inline int WorldGeometry::ypdiff(int y1, int y2) const {
  if (!m_yWrap)
    return y1 - y2;
  else
    return pmod<int>(y1 - y2, m_size[1]);
}

inline float WorldGeometry::ydiff(float y1, float y2) const {
  if (!m_yWrap)
    return y1 - y2;
  else
    return wrapDiffF<float>(y1, y2, m_size[1]);
}

inline int WorldGeometry::ydiff(int y1, int y2) const {
  if (!m_yWrap)
    return y1 - y2;
  else
    return wrapDiff<int>(y1, y2, m_size[1]);
}

inline Vec2F WorldGeometry::diff(Vec2F const& p1, Vec2F const& p2) const {
  return {xdiff(p1[0], p2[0]), ydiff(p1[1], p2[1])};
}

inline Vec2I WorldGeometry::diff(Vec2I const& p1, Vec2I const& p2) const {
  return {xdiff(p1[0], p2[0]), ydiff(p1[1], p2[1])};
}

inline Vec2F WorldGeometry::midpoint(Vec2F const& p1, Vec2F const& p2) const {
  return limit(diff(p1, p2) / 2 + p2);
}

inline int WorldGeometry::nearestToX(int source, int target) const {
  if (!m_xWrap || abs(target - source) < (int)(m_size[0] / 2))
    return target;
  else
    return xdiff(target, source) + source;
}

inline float WorldGeometry::nearestToX(float source, float target) const {
  if (!m_xWrap || abs(target - source) < (float)(m_size[0] / 2))
    return target;
  else
    return xdiff(target, source) + source;
}

inline int WorldGeometry::nearestToY(int source, int target) const {
  if (!m_yWrap || abs(target - source) < (int)(m_size[1] / 2))
    return target;
  else
    return ydiff(target, source) + source;
}

inline float WorldGeometry::nearestToY(float source, float target) const {
  if (!m_yWrap || abs(target - source) < (float)(m_size[1] / 2))
    return target;
  else
    return ydiff(target, source) + source;
}

inline Vec2I WorldGeometry::nearestTo(Vec2I const& source, Vec2I const& target) const {
  return Vec2I(nearestToX(source[0], target[0]), nearestToY(source[1], target[1]));
}

inline Vec2F WorldGeometry::nearestTo(Vec2F const& source, Vec2F const& target) const {
  return Vec2F(nearestToX(source[0], target[0]), nearestToY(source[1], target[1]));
}

}
