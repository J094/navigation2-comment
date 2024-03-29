// Copyright (c) 2008, Willow Garage, Inc.
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of the Willow Garage nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

//
// Navigation function computation
// Uses Dijkstra's method
// Modified for Euclidean-distance computation
//
// Path calculation uses no interpolation when pot field is at max in
//   nearby cells
//
// Path calc has sanity check that it succeeded
//

#include "nav2_navfn_planner/navfn.hpp"

#include <algorithm>
#include "rclcpp/rclcpp.hpp"

namespace nav2_navfn_planner
{

//
// function to perform nav fn calculation
// keeps track of internal buffers, will be more efficient
//   if the size of the environment does not change
//

// Example usage:
/*
int
create_nav_plan_astar(
  COSTTYPE * costmap, int nx, int ny,
  int * goal, int * start,
  float * plan, int nplan)
{
  static NavFn * nav = NULL;

  if (nav == NULL) {
    nav = new NavFn(nx, ny);
  }

  if (nav->nx != nx || nav->ny != ny) {  // check for compatibility with previous call
    delete nav;
    nav = new NavFn(nx, ny);
  }

  nav->setGoal(goal);
  nav->setStart(start);

  nav->costarr = costmap;
  nav->setupNavFn(true);

  // calculate the nav fn and path
  nav->priInc = 2 * COST_NEUTRAL;
  nav->propNavFnAstar(std::max(nx * ny / 20, nx + ny));

  // path
  int len = nav->calcPath(nplan);

  if (len > 0) {  // found plan
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "[NavFn] Path found, %d steps\n", len);
  } else {
    RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "[NavFn] No path found\n");
  }

  if (len > 0) {
    for (int i = 0; i < len; i++) {
      plan[i * 2] = nav->pathx[i];
      plan[i * 2 + 1] = nav->pathy[i];
    }
  }

  return len;
}
*/

//
// create nav fn buffers
//

NavFn::NavFn(int xs, int ys)
{
  // xs 和 ys 为地图的 x y 尺寸
  // create cell arrays
  // 初始化各种数组
  costarr = NULL;
  potarr = NULL;
  pending = NULL;
  gradx = grady = NULL;
  setNavArr(xs, ys);

  // 优先级 buffers
  // priority buffers
  pb1 = new int[PRIORITYBUFSIZE];
  pb2 = new int[PRIORITYBUFSIZE];
  pb3 = new int[PRIORITYBUFSIZE];

  // 不同算法不同设置
  // for Dijkstra (breadth-first), set to COST_NEUTRAL
  // for A* (best-first), set to COST_NEUTRAL
  priInc = 2 * COST_NEUTRAL;

  // 终点和起点初始化
  // goal and start
  goal[0] = goal[1] = 0;
  start[0] = start[1] = 0;

  // display function
  // displayFn = NULL;
  // displayInt = 0;

  // 路径的 buffers
  // path buffers
  npathbuf = npath = 0;
  pathx = pathy = NULL;
  pathStep = 0.5;
}


NavFn::~NavFn()
{
  if (costarr) {
    delete[] costarr;
  }
  if (potarr) {
    delete[] potarr;
  }
  if (pending) {
    delete[] pending;
  }
  if (gradx) {
    delete[] gradx;
  }
  if (grady) {
    delete[] grady;
  }
  if (pathx) {
    delete[] pathx;
  }
  if (pathy) {
    delete[] pathy;
  }
  if (pb1) {
    delete[] pb1;
  }
  if (pb2) {
    delete[] pb2;
  }
  if (pb3) {
    delete[] pb3;
  }
}


//
// set goal, start positions for the nav fn
//

void
NavFn::setGoal(int * g)
{
  goal[0] = g[0];
  goal[1] = g[1];
  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "[NavFn] Setting goal to %d,%d\n", goal[0], goal[1]);
}

void
NavFn::setStart(int * g)
{
  start[0] = g[0];
  start[1] = g[1];
  RCLCPP_DEBUG(
    rclcpp::get_logger("rclcpp"), "[NavFn] Setting start to %d,%d\n", start[0],
    start[1]);
}

//
// Set/Reset map size
//

void
NavFn::setNavArr(int xs, int ys)
{
  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "[NavFn] Array is %d x %d\n", xs, ys);

  nx = xs;
  ny = ys;
  ns = nx * ny;

  // 重置这些数组
  if (costarr) {
    delete[] costarr;
  }
  if (potarr) {
    delete[] potarr;
  }
  if (pending) {
    delete[] pending;
  }

  if (gradx) {
    delete[] gradx;
  }
  if (grady) {
    delete[] grady;
  }

  costarr = new COSTTYPE[ns];  // cost array, 2d config space
  memset(costarr, 0, ns * sizeof(COSTTYPE));
  potarr = new float[ns];  // navigation potential array
  pending = new bool[ns];
  memset(pending, 0, ns * sizeof(bool));
  gradx = new float[ns];
  grady = new float[ns];
}


//
// set up cost array, usually from ROS
//

void
NavFn::setCostmap(const COSTTYPE * cmap, bool isROS, bool allow_unknown)
{
  // 对于 ros 的和非 ros 的地图不同操作
  COSTTYPE * cm = costarr;
  if (isROS) {  // ROS-type cost array
    for (int i = 0; i < ny; i++) {
      int k = i * nx;
      for (int j = 0; j < nx; j++, k++, cmap++, cm++) {
        // This transforms the incoming cost values:
        // COST_OBS                 -> COST_OBS (incoming "lethal obstacle")
        // COST_OBS_ROS             -> COST_OBS (incoming "inscribed inflated obstacle")
        // values in range 0 to 252 -> values from COST_NEUTRAL to COST_OBS_ROS.
        // 开始给个致命障碍, 后续根据情况更新
        *cm = COST_OBS;
        int v = *cmap;
        if (v < COST_OBS_ROS) {
          v = COST_NEUTRAL + COST_FACTOR * v;
          if (v >= COST_OBS) {
            // 其实就是 COST_OBS_ROS
            v = COST_OBS - 1;
          }
          *cm = v;
        } else if (v == COST_UNKNOWN_ROS && allow_unknown) {
          // 如果 allow_unknown 表示 unknown 区域的 cost 小一点
          v = COST_OBS - 1;
          *cm = v;
        }
      }
    }
  } else {  // not a ROS map, just a PGM
    for (int i = 0; i < ny; i++) {
      int k = i * nx;
      for (int j = 0; j < nx; j++, k++, cmap++, cm++) {
        *cm = COST_OBS;
        // TODO: 意义不明?
        if (i < 7 || i > ny - 8 || j < 7 || j > nx - 8) {
          continue;  // don't do borders
        }
        int v = *cmap;
        if (v < COST_OBS_ROS) {
          v = COST_NEUTRAL + COST_FACTOR * v;
          if (v >= COST_OBS) {
            v = COST_OBS - 1;
          }
          *cm = v;
        } else if (v == COST_UNKNOWN_ROS) {
          v = COST_OBS - 1;
          *cm = v;
        }
      }
    }
  }
}

bool
NavFn::calcNavFnDijkstra(bool atStart)
{
  // 配置
  setupNavFn(true);

  // 计算路径
  // calculate the nav fn and path
  return propNavFnDijkstra(std::max(nx * ny / 20, nx + ny), atStart);
}


//
// calculate navigation function, given a costmap, goal, and start
//

bool
NavFn::calcNavFnAstar()
{
  // 配置
  setupNavFn(true);

  // 计算路径
  // calculate the nav fn and path
  return propNavFnAstar(std::max(nx * ny / 20, nx + ny));
}

//
// returning values
//

float * NavFn::getPathX() {return pathx;}
float * NavFn::getPathY() {return pathy;}
int NavFn::getPathLen() {return npath;}

// inserting onto the priority blocks
// n 必须在地图中, 然后 pending[n] 表示 n 没有访问过, 然后 n 处不是障碍物, 然后 curPe 不超过缓存尺寸
// 就把 n 放入 curP 的 curPe 的位置, 然后 curPe++, 注意这里 curPe 初始为 0, 会不断累加
#define push_cur(n)  {if (n >= 0 && n < ns && !pending[n] && \
      costarr[n] < COST_OBS && curPe < PRIORITYBUFSIZE) \
    {curP[curPe++] = n; pending[n] = true;}}
// 同样的判断条件, 只不过会放入到 nextP 中, 但是同样会影响 pending[n]
#define push_next(n) {if (n >= 0 && n < ns && !pending[n] && \
      costarr[n] < COST_OBS && nextPe < PRIORITYBUFSIZE) \
    {nextP[nextPe++] = n; pending[n] = true;}}
// 同样, 放入 overP 中
#define push_over(n) {if (n >= 0 && n < ns && !pending[n] && \
      costarr[n] < COST_OBS && overPe < PRIORITYBUFSIZE) \
    {overP[overPe++] = n; pending[n] = true;}}


// Set up navigation potential arrays for new propagation

void
NavFn::setupNavFn(bool keepit)
{
  // 先重置数组
  // reset values in propagation arrays
  for (int i = 0; i < ns; i++) {
    potarr[i] = POT_HIGH;
    if (!keepit) {
      // 如果不保留, 重置 costarr
      costarr[i] = COST_NEUTRAL;
    }
    gradx[i] = grady[i] = 0.0;
  }

  // 这里是把地图边界一圈设置为 COST_OBS 禁止靠近
  // outer bounds of cost array
  COSTTYPE * pc;
  pc = costarr;
  for (int i = 0; i < nx; i++) {
    *pc++ = COST_OBS;
  }
  pc = costarr + (ny - 1) * nx;
  for (int i = 0; i < nx; i++) {
    *pc++ = COST_OBS;
  }
  pc = costarr;
  for (int i = 0; i < ny; i++, pc += nx) {
    *pc = COST_OBS;
  }
  pc = costarr + nx - 1;
  for (int i = 0; i < ny; i++, pc += nx) {
    *pc = COST_OBS;
  }

  // 优先级 buffers
  // priority buffers
  // 当前阈值设为禁止区域 COST_OBS
  curT = COST_OBS;
  // 当前为 pb1
  // 用于当前传播过程的索引数组
  curP = pb1;
  // 数量
  curPe = 0;
  // 下一个为 pb2
  // 用于下一个传播过程的索引数组
  nextP = pb2;
  // 数量
  nextPe = 0;
  // overflow 的 block 为 pb3
  // 传播阈值之外的索引数组
  overP = pb3;
  // 数量
  overPe = 0;
  // 先给 pending 全设置为 0
  memset(pending, 0, ns * sizeof(bool));

  // 设置终点的 index = x + y * size_x
  // set goal
  int k = goal[0] + goal[1] * nx;
  // 初始化终点的 potential 为 0
  initCost(k, 0);

  // 寻找障碍物 cells, 计算数量
  // find # of obstacle cells
  pc = costarr;
  int ntot = 0;
  for (int i = 0; i < ns; i++, pc++) {
    if (*pc >= COST_OBS) {
      ntot++;  // number of cells that are obstacles
    }
  }
  nobs = ntot;
}


// initialize a goal-type cost for starting propagation

void
NavFn::initCost(int k, float v)
{
  // 这里设置 potential
  // 然后把当前点的前后左右放进 curP 中
  // 终点的话就是设置 potential 为 0
  potarr[k] = v;
  push_cur(k + 1);
  push_cur(k - 1);
  push_cur(k - nx);
  push_cur(k + nx);
}


//
// Critical function: calculate updated potential value of a cell,
//   given its neighbors' values
// Planar-wave update calculation from two lowest neighbors in a 4-grid
// Quadratic approximation to the interpolated value
// No checking of bounds here, this function should be fast
//

#define INVSQRT2 0.707106781

inline void
NavFn::updateCell(int n)
{
  // get neighbors
  float u, d, l, r;
  l = potarr[n - 1];
  r = potarr[n + 1];
  u = potarr[n - nx];
  d = potarr[n + nx];
  // ROS_INFO("[Update] c: %0.1f  l: %0.1f  r: %0.1f  u: %0.1f  d: %0.1f\n",
  //  potarr[n], l, r, u, d);
  // ROS_INFO("[Update] cost: %d\n", costarr[n]);

  // find lowest, and its lowest neighbor
  float ta, tc;
  if (l < r) {tc = l;} else {tc = r;}
  if (u < d) {ta = u;} else {ta = d;}

  // do planar wave update
  if (costarr[n] < COST_OBS) {  // don't propagate into obstacles
    float hf = static_cast<float>(costarr[n]);  // traversability factor
    float dc = tc - ta;  // relative cost between ta,tc
    if (dc < 0) {  // ta is lowest
      dc = -dc;
      ta = tc;
    }

    // calculate new potential
    float pot;
    if (dc >= hf) {  // if too large, use ta-only update
      pot = ta + hf;
    } else {  // two-neighbor interpolation update
      // use quadratic approximation
      // might speed this up through table lookup, but still have to
      //   do the divide
      float d = dc / hf;
      float v = -0.2301 * d * d + 0.5307 * d + 0.7040;
      pot = ta + hf * v;
    }

    //      ROS_INFO("[Update] new pot: %d\n", costarr[n]);

    // now add affected neighbors to priority blocks
    if (pot < potarr[n]) {
      float le = INVSQRT2 * static_cast<float>(costarr[n - 1]);
      float re = INVSQRT2 * static_cast<float>(costarr[n + 1]);
      float ue = INVSQRT2 * static_cast<float>(costarr[n - nx]);
      float de = INVSQRT2 * static_cast<float>(costarr[n + nx]);
      potarr[n] = pot;
      if (pot < curT) {  // low-cost buffer block
        if (l > pot + le) {push_next(n - 1);}
        if (r > pot + re) {push_next(n + 1);}
        if (u > pot + ue) {push_next(n - nx);}
        if (d > pot + de) {push_next(n + nx);}
      } else {  // overflow block
        if (l > pot + le) {push_over(n - 1);}
        if (r > pot + re) {push_over(n + 1);}
        if (u > pot + ue) {push_over(n - nx);}
        if (d > pot + de) {push_over(n + nx);}
      }
    }
  }
}

//
// Use A* method for setting priorities
// Critical function: calculate updated potential value of a cell,
//   given its neighbors' values
// Planar-wave update calculation from two lowest neighbors in a 4-grid
// Quadratic approximation to the interpolated value
// No checking of bounds here, this function should be fast
//

// 这是根号二分之一
#define INVSQRT2 0.707106781

inline void
NavFn::updateCellAstar(int n)
{
  // 先拿到该 cell 的四周 potential, 就是该点到终点的花费
  // get neighbors
  float u, d, l, r;
  l = potarr[n - 1];
  r = potarr[n + 1];
  u = potarr[n - nx];
  d = potarr[n + nx];
  // ROS_INFO("[Update] c: %0.1f  l: %0.1f  r: %0.1f  u: %0.1f  d: %0.1f\n",
  // potarr[n], l, r, u, d);
  // ROS_INFO("[Update] cost of %d: %d\n", n, costarr[n]);

  // find lowest, and its lowest neighbor
  float ta, tc;
  // tc 为左右中 potential 更小的那一个 cell
  if (l < r) {tc = l;} else {tc = r;}
  // ta 为上下中 potential 更小的那一个 cell
  if (u < d) {ta = u;} else {ta = d;}

  // planar-wave update 是从上下左右中最小的两个邻居计算
  // do planar wave update
  if (costarr[n] < COST_OBS) {  // don't propagate into obstacles
    // 对于障碍物不考虑
    // hf 拿到了当前 cell 的 cost
    float hf = static_cast<float>(costarr[n]);  // traversability factor
    float dc = tc - ta;  // relative cost between ta,tc
    if (dc < 0) {  // ta is lowest
      // ta 应该是 ta tc 最小的那一个, dc 应该 > 0
      dc = -dc;
      ta = tc;
    }

    // 这里计算 potential, 当前的 potential 应该是周围最小的 potential 加上当前的 cost
    // calculate new potential
    float pot;
    if (dc >= hf) {  // if too large, use ta-only update
      // 这里如果 dc >= hf 表示最小的两个邻居 potential 差大于当前位置的 cost, 差值足够大
      // 直接采用 ta 的 potential 计算当前 cell 的 potential
      pot = ta + hf;
    } else {  // two-neighbor interpolation update
      // use quadratic approximation
      // might speed this up through table lookup, but still have to
      //   do the divide
      // 如果最小的和第二小的两个邻居差距没有足够大
      // 二阶近似插值, 四个点, 两个最小 potential 的点差值是 dc, 还存在一个点距离当前点的差值为 hf
      // 通过二阶插值公式获得当前点的 potential, 存在的那个点应该在 ta 一下的某个位置
      // 提供更加连续平滑的 potential
      float d = dc / hf;
      float v = -0.2301 * d * d + 0.5307 * d + 0.7040;
      pot = ta + hf * v;
    }

    // ROS_INFO("[Update] new pot: %d\n", costarr[n]);

    // 当前位置的 potential 改变了, 又会影响四周的 potential
    // 添加受影响的邻居到优先级 blocks
    // now add affected neighbors to priority blocks
    if (pot < potarr[n]) {
      // 如果当前新计算的 potential 小于之前的
      // 获取上下左右四个邻居的 cost
      // TODO: 为什么这里除以根号二?
      // 这里默认斜对角才是正常 cost 花费, 所以上下左右都是除以根号二来归一化
      float le = INVSQRT2 * static_cast<float>(costarr[n - 1]);
      float re = INVSQRT2 * static_cast<float>(costarr[n + 1]);
      float ue = INVSQRT2 * static_cast<float>(costarr[n - nx]);
      float de = INVSQRT2 * static_cast<float>(costarr[n + nx]);

      // 计算当前位置的 x y, 然后计算地图距离
      // calculate distance
      int x = n % nx;
      int y = n / nx;
      float dist = hypot(x - start[0], y - start[1]) * static_cast<float>(COST_NEUTRAL);

      // 当前位置的 potential 为 pot
      potarr[n] = pot;
      // 加上距离表示起点到当前位置的预估花费, pot 为实际花费
      pot += dist;
      if (pot < curT) {  // low-cost buffer block
        // 如果 pot 小于当前阈值, 放入 next 优先级 block 中, 下一个循环会检查这些
        // 加入的前提是当前计算的 pot + 当前 cost 要小于之前的 potential
        if (l > pot + le) {push_next(n - 1);}
        if (r > pot + re) {push_next(n + 1);}
        if (u > pot + ue) {push_next(n - nx);}
        if (d > pot + de) {push_next(n + nx);}
      } else {
        // 否则放入 overflow 的优先级 block 中
        if (l > pot + le) {push_over(n - 1);}
        if (r > pot + re) {push_over(n + 1);}
        if (u > pot + ue) {push_over(n - nx);}
        if (d > pot + de) {push_over(n + nx);}
      }
    }
  }
}


//
// main propagation function
// Dijkstra method, breadth-first
// runs for a specified number of cycles,
//   or until it runs out of cells to update,
//   or until the Start cell is found (atStart = true)
//

bool
NavFn::propNavFnDijkstra(int cycles, bool atStart)
{
  int nwv = 0;  // max priority block size
  int nc = 0;  // number of cells put into priority blocks
  int cycle = 0;  // which cycle we're on

  // set up start cell
  int startCell = start[1] * nx + start[0];

  for (; cycle < cycles; cycle++) {  // go for this many cycles, unless interrupted
    if (curPe == 0 && nextPe == 0) {  // priority blocks empty
      break;
    }

    // stats
    nc += curPe;
    if (curPe > nwv) {
      nwv = curPe;
    }

    // reset pending flags on current priority buffer
    int * pb = curP;
    int i = curPe;
    while (i-- > 0) {
      pending[*(pb++)] = false;
    }

    // process current priority buffer
    pb = curP;
    i = curPe;
    while (i-- > 0) {
      updateCell(*pb++);
    }

    // if (displayInt > 0 && (cycle % displayInt) == 0) {
    //   displayFn(this);
    // }

    // swap priority blocks curP <=> nextP
    curPe = nextPe;
    nextPe = 0;
    pb = curP;  // swap buffers
    curP = nextP;
    nextP = pb;

    // see if we're done with this priority level
    if (curPe == 0) {
      curT += priInc;  // increment priority threshold
      curPe = overPe;  // set current to overflow block
      overPe = 0;
      pb = curP;  // swap buffers
      curP = overP;
      overP = pb;
    }

    // check if we've hit the Start cell
    if (atStart) {
      if (potarr[startCell] < POT_HIGH) {
        break;
      }
    }
  }

  RCLCPP_DEBUG(
    rclcpp::get_logger("rclcpp"),
    "[NavFn] Used %d cycles, %d cells visited (%d%%), priority buf max %d\n",
    cycle, nc, (int)((nc * 100.0) / (ns - nobs)), nwv);

  return (cycle < cycles) ? true : false;
}

//
// main propagation function
// A* method, best-first
// uses Euclidean distance heuristic
// runs for a specified number of cycles,
//   or until it runs out of cells to update,
//   or until the Start cell is found (atStart = true)
//

bool
NavFn::propNavFnAstar(int cycles)
{
  int nwv = 0;  // max priority block size
  // 放入优先级 buffer 的数量
  int nc = 0;  // number of cells put into priority blocks
  int cycle = 0;  // which cycle we're on

  // 基于距离设置初始的阈值
  // set initial threshold, based on distance
  float dist = hypot(goal[0] - start[0], goal[1] - start[1]) * static_cast<float>(COST_NEUTRAL);
  // 这里阈值被放大, 应该是从终点开始, 然后加上预估起点到终点的花费
  curT = dist + curT;

  // 设置开始 cell 的 index = x + y * size_x
  // set up start cell
  int startCell = start[1] * nx + start[0];

  // 主要循环
  // do main cycle
  for (; cycle < cycles; cycle++) {  // go for this many cycles, unless interrupted
    if (curPe == 0 && nextPe == 0) {  // priority blocks empty
      // 这里如果所有指正都是 0, 表示优先级 arr 中没有 cell
      break;
    }

    // 统计
    // stats
    // 这一轮累计 curPe 个 cells
    // 如果是第一轮, 也就是终点附近的四个邻居
    nc += curPe;
    if (curPe > nwv) {
      // 如果找到了最大的优先级 buffer 的 size, 记录下来
      nwv = curPe;
    }

    // 在当前的优先级 buffer 中重置 pending
    // reset pending flags on current priority buffer
    int * pb = curP;
    int i = curPe;
    while (i-- > 0) {
      // 这里 *pb 就是获取 index
      pending[*(pb++)] = false;
    }

    // 处理当前的优先级 buffer
    // process current priority buffer
    pb = curP;
    i = curPe;
    while (i-- > 0) {
      // 当前优先级 buffer 中的所有 cell
      // 第一轮就是更新终点附近的四个邻居的 cell
      updateCellAstar(*pb++);
    }
    // 更新完所有 cells 之后, nextP 和 overP 中会有一些新添加的 cell

    // if (displayInt > 0 && (cycle % displayInt) == 0) {
    //   displayFn(this);
    // }

    // curP 处理完了, 和 nextP 互换, 下一轮继续
    // swap priority blocks curP <=> nextP
    curPe = nextPe;
    nextPe = 0;
    pb = curP;  // swap buffers
    curP = nextP;
    nextP = pb;

    // see if we're done with this priority level
    if (curPe == 0) {
      curT += priInc;  // increment priority threshold
      curPe = overPe;  // set current to overflow block
      overPe = 0;
      pb = curP;  // swap buffers
      curP = overP;
      overP = pb;
    }

    // check if we've hit the Start cell
    if (potarr[startCell] < POT_HIGH) {
      // 检查如果已经找到了开始的 cell, 退出
      // 这里是起点的 potential 被更新了, 就代表它被找到了
      break;
    }
  }

  // 最终获得路径的花费
  last_path_cost_ = potarr[startCell];

  RCLCPP_DEBUG(
    rclcpp::get_logger("rclcpp"),
    "[NavFn] Used %d cycles, %d cells visited (%d%%), priority buf max %d\n",
    cycle, nc, (int)((nc * 100.0) / (ns - nobs)), nwv);

  if (potarr[startCell] < POT_HIGH) {
    return true;  // finished up here}
  } else {
    // 如果经过了这么多轮, 还是没有找到开始点, 那么路径规划失败
    return false;
  }
}


float NavFn::getLastPathCost()
{
  return last_path_cost_;
}


//
// Path construction
// Find gradient at array points, interpolate path
// Use step size of pathStep, usually 0.5 pixel
//
// Some sanity checks:
//  1. Stuck at same index position
//  2. Doesn't get near goal
//  3. Surrounded by high potentials
//

int
NavFn::calcPath(int n, int * st)
{
  // test write
  // savemap("test");

  // 首先重置路径
  // check path arrays
  if (npathbuf < n) {
    if (pathx) {delete[] pathx;}
    if (pathy) {delete[] pathy;}
    pathx = new float[n];
    pathy = new float[n];
    npathbuf = n;
  }

  // 设置起点
  // set up start position at cell
  // st is always upper left corner for 4-point bilinear interpolation
  if (st == NULL) {st = start;}
  int stc = st[1] * nx + st[0];

  // set up offset
  float dx = 0;
  float dy = 0;
  npath = 0;

  // go for <n> cycles at most
  for (int i = 0; i < n; i++) {
    // check if near goal
    // 这里是从 stc 走到 dx dy 开外的点位置就是 nearest_point
    int nearest_point = std::max(
      0,
      std::min(
        nx * ny - 1, stc + static_cast<int>(round(dx)) +
        static_cast<int>(nx * round(dy))));
    // 如果这个点的 cost 小于 cost_neutral, 那么代表到达终点了
    // 终点的 potential 为 0
    if (potarr[nearest_point] < COST_NEUTRAL) {
      pathx[npath] = static_cast<float>(goal[0]);
      pathy[npath] = static_cast<float>(goal[1]);
      return ++npath;  // done!
    }

    // 检查是否出界 stc 不应该大于 ns - nx
    // stc 也不应该小于 nx, 因为边界都设置为了障碍物区域, 不应该是起点位置
    if (stc < nx || stc > ns - nx) {  // would be out of bounds
      RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "[PathCalc] Out of bounds");
      return 0;
    }

    // 开始增加路径
    // add to path
    pathx[npath] = stc % nx + dx;
    pathy[npath] = stc / nx + dy;
    npath++;

    // 检查是否震荡, 也就是回到了之前到过的某一点
    bool oscillation_detected = false;
    if (npath > 2 &&
      pathx[npath - 1] == pathx[npath - 3] &&
      pathy[npath - 1] == pathy[npath - 3])
    {
      RCLCPP_DEBUG(
        rclcpp::get_logger("rclcpp"),
        "[PathCalc] oscillation detected, attempting fix.");
      oscillation_detected = true;
    }

    // 上下两点
    int stcnx = stc + nx;
    int stcpx = stc - nx;

    // 检查周围 8 个点的 potential
    // check for potentials at eight positions near cell
    if (potarr[stc] >= POT_HIGH ||
      potarr[stc + 1] >= POT_HIGH ||
      potarr[stc - 1] >= POT_HIGH ||
      potarr[stcnx] >= POT_HIGH ||
      potarr[stcnx + 1] >= POT_HIGH ||
      potarr[stcnx - 1] >= POT_HIGH ||
      potarr[stcpx] >= POT_HIGH ||
      potarr[stcpx + 1] >= POT_HIGH ||
      potarr[stcpx - 1] >= POT_HIGH ||
      oscillation_detected)
    {
      RCLCPP_DEBUG(
        rclcpp::get_logger("rclcpp"),
        "[Path] Pot fn boundary, following grid (%0.1f/%d)", potarr[stc], npath);

      // 这里找到周围最小的 potential, 并把起点设置为最小的点
      // check eight neighbors to find the lowest
      int minc = stc;
      int minp = potarr[stc];
      int st = stcpx - 1;
      if (potarr[st] < minp) {minp = potarr[st]; minc = st;}
      st++;
      if (potarr[st] < minp) {minp = potarr[st]; minc = st;}
      st++;
      if (potarr[st] < minp) {minp = potarr[st]; minc = st;}
      st = stc - 1;
      if (potarr[st] < minp) {minp = potarr[st]; minc = st;}
      st = stc + 1;
      if (potarr[st] < minp) {minp = potarr[st]; minc = st;}
      st = stcnx - 1;
      if (potarr[st] < minp) {minp = potarr[st]; minc = st;}
      st++;
      if (potarr[st] < minp) {minp = potarr[st]; minc = st;}
      st++;
      if (potarr[st] < minp) {minp = potarr[st]; minc = st;}
      stc = minc;
      dx = 0;
      dy = 0;

      RCLCPP_DEBUG(
        rclcpp::get_logger("rclcpp"), "[Path] Pot: %0.1f  pos: %0.1f,%0.1f",
        potarr[stc], pathx[npath - 1], pathy[npath - 1]);

      // 检查这一点是否合理, 如果这一点还是没有到达的点, 那么找不到路径了
      if (potarr[stc] >= POT_HIGH) {
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "[PathCalc] No path found, high potential");
        // savemap("navfn_highpot");
        return 0;
      }
    } else {  // have a good gradient here
      // 这种情况是周围的梯度都没问题
      // get grad at four positions near cell
      // 计算周围四个位置的梯度, 因为周围 8 个点, 只能得到中间四个交点的梯度
      gradCell(stc);
      gradCell(stc + 1);
      gradCell(stcnx);
      gradCell(stcnx + 1);


      // get interpolated gradient
      // 得到带 dx dy 偏移的梯度
      float x1 = (1.0 - dx) * gradx[stc] + dx * gradx[stc + 1];
      float x2 = (1.0 - dx) * gradx[stcnx] + dx * gradx[stcnx + 1];
      float x = (1.0 - dy) * x1 + dy * x2;  // interpolated x
      float y1 = (1.0 - dx) * grady[stc] + dx * grady[stc + 1];
      float y2 = (1.0 - dx) * grady[stcnx] + dx * grady[stcnx + 1];
      float y = (1.0 - dy) * y1 + dy * y2;  // interpolated y

#if 0
      // show gradients
      RCLCPP_DEBUG(
        rclcpp::get_logger("rclcpp"),
        "[Path] %0.2f,%0.2f  %0.2f,%0.2f  %0.2f,%0.2f  %0.2f,%0.2f; final x=%.3f, y=%.3f\n",
        gradx[stc], grady[stc], gradx[stc + 1], grady[stc + 1],
        gradx[stcnx], grady[stcnx], gradx[stcnx + 1], grady[stcnx + 1],
        x, y);
#endif

      // check for zero gradient, failed
      if (x == 0.0 && y == 0.0) {
        RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "[PathCalc] Zero gradient");
        return 0;
      }

      // move in the right direction
      // 沿着梯度方向移动
      float ss = pathStep / hypot(x, y);
      dx += x * ss;
      dy += y * ss;

      // check for overflow
      // 检查 dx 和 dy, 适当移动 stc 来保证 dx 和 dy 在 -1 ~ 1 之间
      if (dx > 1.0) {stc++; dx -= 1.0;}
      if (dx < -1.0) {stc--; dx += 1.0;}
      if (dy > 1.0) {stc += nx; dy -= 1.0;}
      if (dy < -1.0) {stc -= nx; dy += 1.0;}
    }

    //      ROS_INFO("[Path] Pot: %0.1f  grad: %0.1f,%0.1f  pos: %0.1f,%0.1f\n",
    //      potarr[stc], x, y, pathx[npath-1], pathy[npath-1]);
  }

  //  return npath;  // out of cycles, return failure
  RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "[PathCalc] No path found, path too long");
  // savemap("navfn_pathlong");
  return 0;  // out of cycles, return failure
}


//
// gradient calculations
//

// calculate gradient at a cell
// positive value are to the right and down
float
NavFn::gradCell(int n)
{
  // 检查这个 cell 是否计算过了
  if (gradx[n] + grady[n] > 0.0) {  // check this cell
    return 1.0;
  }

  if (n < nx || n > ns - nx) {  // would be out of bounds
    return 0.0;
  }

  float cv = potarr[n];
  float dx = 0.0;
  float dy = 0.0;

  // check for in an obstacle
  // 这里检测障碍物, 如果有障碍, 那么梯度加速远离障碍物
  if (cv >= POT_HIGH) {
    if (potarr[n - 1] < POT_HIGH) {
      dx = -COST_OBS;
    } else if (potarr[n + 1] < POT_HIGH) {
      dx = COST_OBS;
    }
    if (potarr[n - nx] < POT_HIGH) {
      dy = -COST_OBS;
    } else if (potarr[n + nx] < POT_HIGH) {
      dy = COST_OBS;
    }
  } else {  // not in an obstacle
    // 如果不是障碍物计算 potential 的梯度
    // 这里计算的是两边的梯度, 同样只有不是障碍物才参与计算
    // dx calc, average to sides
    if (potarr[n - 1] < POT_HIGH) {
      dx += potarr[n - 1] - cv;
    }
    if (potarr[n + 1] < POT_HIGH) {
      dx += cv - potarr[n + 1];
    }

    // dy calc, average to sides
    if (potarr[n - nx] < POT_HIGH) {
      dy += potarr[n - nx] - cv;
    }
    if (potarr[n + nx] < POT_HIGH) {
      dy += cv - potarr[n + nx];
    }
  }

  // normalize
  // 归一化梯度
  float norm = hypot(dx, dy);
  if (norm > 0) {
    norm = 1.0 / norm;
    gradx[n] = norm * dx;
    grady[n] = norm * dy;
  }
  return norm;
}


//
// display function setup
// <n> is the number of cycles to wait before displaying,
//     use 0 to turn it off

// void
// NavFn::display(void fn(NavFn * nav), int n)
// {
//   displayFn = fn;
//   displayInt = n;
// }


//
// debug writes
// saves costmap and start/goal
//

// void
// NavFn::savemap(const char * fname)
// {
//   char fn[4096];

//   RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "[NavFn] Saving costmap and start/goal points");
//   // write start and goal points
//   snprintf(fn, sizeof(fn), "%s.txt", fname);
//   FILE * fp = fopen(fn, "w");
//   if (!fp) {
//     RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Can't open file %s", fn);
//     return;
//   }
//   fprintf(fp, "Goal: %d %d\nStart: %d %d\n", goal[0], goal[1], start[0], start[1]);
//   fclose(fp);

//   // write cost array
//   if (!costarr) {
//     return;
//   }
//   snprintf(fn, sizeof(fn), "%s.pgm", fname);
//   fp = fopen(fn, "wb");
//   if (!fp) {
//     RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Can't open file %s", fn);
//     return;
//   }
//   fprintf(fp, "P5\n%d\n%d\n%d\n", nx, ny, 0xff);
//   fwrite(costarr, 1, nx * ny, fp);
//   fclose(fp);
// }

}  // namespace nav2_navfn_planner
