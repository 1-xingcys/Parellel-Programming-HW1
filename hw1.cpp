#include "tbb/concurrent_priority_queue.h"
#include "tbb/concurrent_unordered_map.h"
#include "tbb/concurrent_unordered_set.h"
#include <algorithm>
#include <boost/functional/hash.hpp>
#include <condition_variable>
#include <fstream>
#include <iostream>
#include <map>
#include <mutex>
#include <omp.h>
#include <optional>
#include <queue>
#include <set>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

// #define DEBUG 1
#define AXIS_VERTICAL 0
#define AXIS_HORIZONTAL 1

// --- 座標結構 ---
struct Point {
  int r, c;

  // 為了能放入 std::set 和 std::map
  bool operator<(const Point &other) const {
    if (r != other.r)
      return r < other.r;
    return c < other.c;
  }
  bool operator==(const Point &other) const {
    return r == other.r && c == other.c;
  }
  Point operator+(const Point &other) const {
    return {.r = r + other.r, .c = c + other.c};
  }
};

// 為 Point 提供 hasher
struct PointHasher {
  std::size_t operator()(const Point &p) const {
    std::size_t seed = 0;
    boost::hash_combine(seed, p.r);
    boost::hash_combine(seed, p.c);
    return seed;
  }
};

struct PointPairHasher {
  std::size_t operator()(const std::pair<Point, Point> &pp) const {
    std::size_t seed = 0;
    boost::hash_combine(seed, pp.first.r);
    boost::hash_combine(seed, pp.first.c);
    boost::hash_combine(seed, pp.second.r);
    boost::hash_combine(seed, pp.second.c);
    return seed;
  }
};

// --- 遊戲狀態結構 ---
struct State {
  Point playerPos;
  std::vector<Point> boxPositions; // 保持排序以作為唯一標識

  // 為了能放入 std::set 和 std::map
  bool operator<(const State &other) const {
    if (playerPos < other.playerPos)
      return true;
    if (other.playerPos < playerPos)
      return false;
    return boxPositions < other.boxPositions;
  }
  bool operator==(const State &other) const {
    return playerPos == other.playerPos && boxPositions == other.boxPositions;
  }
};

// 為 State 結構提供一個雜湊函式 (Hasher)
struct StateHasher {
  std::size_t operator()(const State &s) const {
    std::size_t seed = 0;
    boost::hash_combine(seed, s.playerPos.r);
    boost::hash_combine(seed, s.playerPos.c);

    // 手動計算 vector<Point> 的雜湊值
    PointHasher point_hasher;
    for (const auto &box : s.boxPositions) {
      boost::hash_combine(seed, point_hasher(box));
    }
    return seed;
  }
};

// 用一個 wrapper 結構來儲存狀態和它的 f-cost
struct Node {
  State state;
  int f_cost;

  // priority_queue 預設是 max-heap，我們需要 min-heap，所以反轉比較符
  bool operator>(const Node &other) const { return f_cost > other.f_cost; }

  // TBB concurrent_priority_queue 需要 < 運算符
  bool operator<(const Node &other) const { return f_cost < other.f_cost; }
};

// For outside BFS
struct StateInfo {
  int g_cost;
  State parent_state;
  std::string move_to_get_here;
  std::unordered_set<Point, PointHasher> frozen_boxes;
};

// --- 全域變數和輔助函數 ---
std::vector<std::string> static_map;
std::set<Point> goals;
std::unordered_map<std::pair<Point, Point>, int, PointPairHasher> distances;
tbb::concurrent_unordered_map<State, StateInfo, StateHasher> state_info_map;
std::unordered_set<Point, PointHasher> deadlock_points;

// 方向向量 (Up, Left, Down, Right)
const std::vector<Point> DIRS = {{-1, 0}, {0, -1}, {1, 0}, {0, 1}};
const std::string MOVE_CHARS = "WASD"; // 注意順序對應 DIRS


int calculate_h_cost(const State &s) {
  int total_distance = 0;
  for (const auto &box : s.boxPositions) {
    int min_dist_for_this_box = 1e9;
    for (const auto &goal : goals) {
      int dist = distances[{goal, box}];
      if (dist < min_dist_for_this_box) {
        min_dist_for_this_box = dist;
      }
    }
    total_distance += min_dist_for_this_box;
  }
  return total_distance;
}

// 檢查某座標是否可走 (對玩家而言)
bool is_walkable(const Point &p, const std::vector<Point> &boxes) {
  if (p.r < 0 || p.r >= static_cast<int>(static_map.size()) || p.c < 0 ||
      p.c >= static_cast<int>(static_map[0].size())) {
    return false;
  }
  char tile = static_map[p.r][p.c];
  if (tile == '#') {
    return false;
  }
  // 檢查是否撞到箱子
  for (const auto &box : boxes) {
    if (p == box) {
      return false;
    }
  }
  return true;
}

bool IsBlockedOnAxis(
    const Point &box,
    const std::unordered_set<Point, PointHasher> &candidateBoxes,
    std::unordered_set<Point, PointHasher>
        &checkedBoxes, // boxes already checked in recursive steps
    std::unordered_map<Point, bool, PointHasher> &stuck_status_horizontal,
    std::unordered_map<Point, bool, PointHasher> &stuck_status_vertical,
    int axis) {
  Point pos1 = box + DIRS[axis], pos2 = box + DIRS[axis + 2];
  checkedBoxes.insert(box);

  // Check stuck by wall
  if (static_map[pos1.r][pos1.c] == '#' || static_map[pos2.r][pos2.c] == '#') {
    checkedBoxes.erase(box);
    return true;
  }

  // Check stuck by deadlock point
  if (deadlock_points.find(pos1) != deadlock_points.end() &&
      deadlock_points.find(pos2) != deadlock_points.end()) {
    checkedBoxes.erase(box);
    return true;
  }

  // Check stuck by other boxes

  // cycle check
  if (checkedBoxes.find(pos1) != checkedBoxes.end() ||
      checkedBoxes.find(pos2) != checkedBoxes.end()) {
    checkedBoxes.erase(box);
    return true;
  }

  bool pos_stucks[2] = {false, false};
  Point poses[2] = {pos1, pos2};

  for (int i = 0; i < 2; ++i) {
    Point pos = poses[i];
    if (candidateBoxes.find(pos) != candidateBoxes.end()) {
      if (axis == AXIS_VERTICAL) {
        if (stuck_status_horizontal.find(pos) !=
            stuck_status_horizontal.end()) {
          pos_stucks[i] = stuck_status_horizontal[pos];
        } else {
          pos_stucks[i] = IsBlockedOnAxis(
              pos, candidateBoxes, checkedBoxes, stuck_status_horizontal,
              stuck_status_vertical, AXIS_HORIZONTAL);
          stuck_status_horizontal[pos] = pos_stucks[i];
        }
      }
      if (axis == AXIS_HORIZONTAL) {
        if (stuck_status_vertical.find(pos) != stuck_status_vertical.end()) {
          pos_stucks[i] = stuck_status_vertical[pos];
        } else {
          pos_stucks[i] = IsBlockedOnAxis(pos, candidateBoxes, checkedBoxes,
                                          stuck_status_horizontal,
                                          stuck_status_vertical, AXIS_VERTICAL);
          stuck_status_vertical[pos] = pos_stucks[i];
        }
      }
      if (pos_stucks[i]) {
        checkedBoxes.erase(box);
        return true;
      }
    }
  }
  checkedBoxes.erase(box);
  return false;
}

bool is_dead_goal(const std::unordered_set<Point, PointHasher> &candidateBoxes,
                  const std::unordered_set<Point, PointHasher> &frozenBoxes) {
  // check every unmatched goals if it is a dead goal
  for (const auto &goal : goals) {
    // regard the frozen boxes as walls, do BFS to check if any candidate box is
    // reachable from the goal
    if (frozenBoxes.find(goal) != frozenBoxes.end())
      continue;
    bool is_dead = true;
    std::queue<Point> q;
    q.push(goal);
    std::set<Point> visited;
    visited.insert(goal);
    while (!q.empty()) {
      Point curr = q.front();
      q.pop();
      if (candidateBoxes.find(curr) != candidateBoxes.end() &&
          frozenBoxes.find(curr) == frozenBoxes.end()) {
        is_dead = false;
        break;
      }
      for (int i = 0; i < 4; ++i) {
        Point next = curr + DIRS[i], next2 = next + DIRS[i];
        if (visited.find(next) == visited.end() &&
            static_map[next.r][next.c] != '#' &&
            frozenBoxes.find(next) == frozenBoxes.end() &&
            static_map[next2.r][next2.c] != '#' &&
            frozenBoxes.find(next2) == frozenBoxes.end()) {
          visited.insert(next);
          q.push(next);
        }
      }
    }
    if (is_dead)
      return true;
  }
  return false;
}

bool is_dead_box(const std::unordered_set<Point, PointHasher> &candidateBoxes,
                 const std::unordered_set<Point, PointHasher> &frozenBoxes) {
  for (const auto &box : candidateBoxes) {
    if (frozenBoxes.find(box) != frozenBoxes.end())
      continue;
    // regard the frozen boxes as walls, do BFS to check if any goal is
    // reachable from the box
    bool is_dead = true;
    std::queue<Point> q;
    q.push(box);
    std::set<Point> visited;
    visited.insert(box);
    while (!q.empty()) {
      Point curr = q.front();
      q.pop();
      if (goals.find(curr) != goals.end()) {
        is_dead = false;
        break;
      }
      for (int i = 0; i < 4; ++i) {
        Point next = curr + DIRS[i], prev = curr + DIRS[(i + 2) % 4];
        if (visited.find(next) == visited.end() &&
            static_map[next.r][next.c] != '#' &&
            frozenBoxes.find(next) == frozenBoxes.end() &&
            static_map[prev.r][prev.c] != '#' &&
            frozenBoxes.find(prev) == frozenBoxes.end()) {
          visited.insert(next);
          q.push(next);
        }
      }
    }
    if (is_dead)
      return true;
  }
  return false;
}

bool is_deadlock(const State &newState, const Point &movedBox,
                 std::unordered_set<Point, PointHasher> &frozenBoxes) {
  std::unordered_set<Point, PointHasher> candidateBoxes;
  std::unordered_set<Point, PointHasher> all_boxes(
      newState.boxPositions.begin(), newState.boxPositions.end());

  // BFS from movedBox to find connected boxes
  std::queue<Point> q;
  q.push(movedBox);
  std::unordered_set<Point, PointHasher> visited;
  visited.insert(movedBox);
  while (!q.empty()) {
    Point curr = q.front();
    q.pop();
    candidateBoxes.insert(curr);
    for (int i = 0; i < 4; ++i) {
      Point next = curr + DIRS[i];
      if (all_boxes.find(next) != all_boxes.end() &&
          visited.find(next) == visited.end()) {
        visited.insert(next);
        q.push(next);
      }
    }
  }

  // For freeze deadlock
  std::unordered_map<Point, bool, PointHasher> stuck_status_horizontal;
  std::unordered_map<Point, bool, PointHasher> stuck_status_vertical;
  std::unordered_set<Point, PointHasher> checkedBoxes;

  for (const auto &box : candidateBoxes) {
    if (frozenBoxes.find(box) != frozenBoxes.end())
      continue;
    bool horizontally_stuck = false, vertically_stuck = false;
    // Check if the box is horizontally stuck
    if (stuck_status_horizontal.find(box) != stuck_status_horizontal.end()) {
      horizontally_stuck = stuck_status_horizontal[box];
    } else {
      checkedBoxes.clear();
      horizontally_stuck = IsBlockedOnAxis(
          box, candidateBoxes, checkedBoxes, stuck_status_horizontal,
          stuck_status_vertical, AXIS_HORIZONTAL);
      stuck_status_horizontal[box] = horizontally_stuck;
    }

    // Check if the box is vertically stuck
    if (stuck_status_vertical.find(box) != stuck_status_vertical.end()) {
      vertically_stuck = stuck_status_vertical[box];
    } else {
      checkedBoxes.clear();
      vertically_stuck = IsBlockedOnAxis(box, candidateBoxes, checkedBoxes,
                                         stuck_status_horizontal,
                                         stuck_status_vertical, AXIS_VERTICAL);
      stuck_status_vertical[box] = vertically_stuck;
    }

    if (horizontally_stuck && vertically_stuck) {
      frozenBoxes.insert(box);
    }
  }

  // If there is any frozen box not in goal, it is a freeze deadlock
  if (frozenBoxes.empty())
    return false;
  for (const auto &box : frozenBoxes) {
    if (goals.find(box) == goals.end()) {
      return true;
    }
  }
  // If there is any dead goal, it is a deadlock
  if (is_dead_goal(all_boxes, frozenBoxes)) {
    return true;
  }
  if (is_dead_box(all_boxes, frozenBoxes)) {
    return true;
  }
  return false;
}

// 內層 BFS: 尋找玩家從 start 到 end 的路徑
std::string find_player_path(const Point &start, const Point &end,
                             const std::vector<Point> &boxes) {
  std::queue<Point> q;
  q.push(start);
  std::map<Point, std::pair<Point, char>> parent_map;
  std::set<Point> visited;
  visited.insert(start);

  while (!q.empty()) {
    Point curr = q.front();
    q.pop();

    if (curr == end) {
      std::string path = "";
      Point at = end;
      while (!(at == start)) {
        auto &p = parent_map[at];
        path += p.second;
        at = p.first;
      }
      std::reverse(path.begin(), path.end());
      return path;
    }

    for (int i = 0; i < 4; ++i) {
      Point next = curr + DIRS[i];
      if (is_walkable(next, boxes) && visited.find(next) == visited.end()) {
        visited.insert(next);
        q.push(next);
        parent_map[next] = {curr, MOVE_CHARS[i]};
      }
    }
  }
  return "none"; // 找不到路徑
}

std::string normalize_player_position(State &state) {
  // 使用 BFS 找到玩家能到達的所有位置
  std::queue<Point> q;
  std::set<Point> visited;
  Point start = state.playerPos;

  q.push(start);
  visited.insert(start);

  Point topLeft = start; // 初始化為起始位置

  while (!q.empty()) {
    Point curr = q.front();
    q.pop();

    // 檢查是否是更 top-left 的位置
    if (curr.r < topLeft.r || (curr.r == topLeft.r && curr.c < topLeft.c)) {
      topLeft = curr;
    }

    // 探索四個方向
    for (int i = 0; i < 4; ++i) {
      Point next = curr + DIRS[i];
      if (is_walkable(next, state.boxPositions) &&
          visited.find(next) == visited.end()) {
        visited.insert(next);
        q.push(next);
      }
    }
  }
  state.playerPos = topLeft;

  // 如果最 top-left 的位置就是當前位置，回傳空字串
  if (topLeft == start) {
    return "";
  }

  // 使用現有的 find_player_path 函式找到路徑
  return find_player_path(start, topLeft, state.boxPositions);
}

void init_distances() {
  for (const auto &goal : goals) {
    std::queue<Point> q;
    q.push(goal);
    std::set<Point> visited;
    visited.insert(goal);
    while (!q.empty()) {
      Point curr = q.front();
      q.pop();
      for (int i = 0; i < 4; ++i) {
        Point next = curr + DIRS[i];
        if (visited.find(next) == visited.end() &&
            (static_map[next.r][next.c] == ' ' ||
             static_map[next.r][next.c] == '.')) {
          visited.insert(next);
          q.push(next);
          distances[{goal, next}] = distances[{goal, curr}] + 1;
        }
      }
    }
  }
}

void init_deadlock_points() {
  std::unordered_set<Point, PointHasher> safe;
  for (const Point &goal : goals) {
    std::queue<Point> q;
    q.push(goal);
    std::set<Point> visited;
    visited.insert(goal);
    while (!q.empty()) {
      Point curr = q.front();
      q.pop();
      safe.insert(curr);
      for (int i = 0; i < 4; ++i) {
        Point next = curr + DIRS[i], next2 = next + DIRS[i];
        if (next.r < 0 || next.r >= static_cast<int>(static_map.size()) ||
            next.c < 0 || next.c >= static_cast<int>(static_map[0].size()))
          continue;
        if (next2.r < 0 || next2.r >= static_cast<int>(static_map.size()) ||
            next2.c < 0 || next2.c >= static_cast<int>(static_map[0].size()))
          continue;
        if (visited.find(next) != visited.end())
          continue;
        if ((static_map[next.r][next.c] == ' ' ||
             static_map[next.r][next.c] == '.') &&
            (static_map[next2.r][next2.c] == ' ' ||
             static_map[next2.r][next2.c] == '.' ||
             static_map[next2.r][next2.c] == '@')) {
          visited.insert(next);
          q.push(next);
        }
      }
    }
  }
  for (int r = 0; r < static_cast<int>(static_map.size()); ++r) {
    for (int c = 0; c < static_cast<int>(static_map[0].size()); ++c) {
      if (safe.find({r, c}) == safe.end() && static_map[r][c] == ' ') {
        deadlock_points.insert({r, c});
      }
    }
  }
  return;
}
// --- 主函式 ---
int main(int argc, char *argv[]) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <map_file>" << std::endl;
    return 1;
  }

  // 1. 讀取並解析地圖
  std::ifstream file(argv[1]);
  if (!file) {
    std::cerr << "Error: Cannot open file " << argv[1] << std::endl;
    return 1;
  }

  State initial_state;
  std::string line;
  int r = 0;
  while (std::getline(file, line)) {
    static_map.push_back(line);
    for (int c = 0; c < static_cast<int>(line.length()); ++c) {
      char tile = line[c];
      if (tile == 'o' || tile == 'O' || tile == '!') {
        initial_state.playerPos = {r, c};
      }
      if (tile == 'x' || tile == 'X') {
        initial_state.boxPositions.push_back({r, c});
      }
      if (tile == '.' || tile == 'O' || tile == 'X') {
        goals.insert({r, c});
      }
      // 將地圖簡化為靜態部分
      if (tile == 'o' || tile == 'x' || tile == ' ')
        static_map[r][c] = ' ';
      else if (tile == 'O' || tile == 'X' || tile == '.')
        static_map[r][c] = '.';
      else if (tile == '!')
        static_map[r][c] = '@';
    }
    r++;
  }
  file.close();

  // 初始化距離
  init_distances();
  init_deadlock_points();

  // 排序初始箱子位置
  std::sort(initial_state.boxPositions.begin(),
            initial_state.boxPositions.end());

#ifdef DEBUG
  // 輸出初始狀態資訊
  std::cout << "初始狀態資訊:" << std::endl;
  std::cout << "玩家位置: (" << initial_state.playerPos.r << ", "
            << initial_state.playerPos.c << ")" << std::endl;
  std::cout << "箱子數量: " << initial_state.boxPositions.size() << std::endl;
  std::cout << "箱子位置: ";
  for (const auto &box : initial_state.boxPositions) {
    std::cout << "(" << box.r << ", " << box.c << ") ";
  }
  std::cout << std::endl;
  std::cout << "目標數量: " << goals.size() << std::endl;
  std::cout << "目標位置: ";
  for (const auto &goal : goals) {
    std::cout << "(" << goal.r << ", " << goal.c << ") ";
  }
  std::cout << std::endl;

  // 印出 static_map
  std::cout << "static_map:" << std::endl;
  for (const auto &row : static_map) {
    std::cout << row << std::endl;
  }
  std::cout << "distances:" << std::endl;
  for (const auto &[key, value] : distances) {
    std::cout << "(" << key.first.r << ", " << key.first.c << ") -> ("
              << key.second.r << ", " << key.second.c << "): " << value
              << std::endl;
  }
  std::cout << "deadlock_points:" << std::endl;
  for (const auto &point : deadlock_points) {
    std::cout << "(" << point.r << ", " << point.c << ") ";
  }
  std::cout << std::endl;
#endif

  // 2. BFS 初始化

  // BFS 算法所需變數
  int initial_g = 0;
  int initial_h = calculate_h_cost(initial_state);
  tbb::concurrent_priority_queue<Node> pq;
  pq.push({initial_state, initial_g + initial_h});
  state_info_map.insert({initial_state,
                         {initial_g, initial_state, "",
                          std::unordered_set<Point, PointHasher>()}});

  // 平行處理所需變數
  std::atomic<bool> solution_found = false;
  std::optional<State> solution_state;
  std::mutex solution_mutex;

  // 紀錄正在工作的 threads 數量
  std::atomic<int> active_threads = 0;
  std::condition_variable cv;
  std::mutex cv_mutex;

#ifdef DEBUG
  std::cout << "開始並行 BFS 搜尋..." << std::endl;
  std::atomic<int> step_count = 0;
  std::mutex debug_mutex; // 用於同步 debug 輸出
#endif

#pragma omp parallel num_threads(6)
  {

#ifdef DEBUG
    int thread_id = omp_get_thread_num();
    int num_threads = omp_get_num_threads();
    {
      std::lock_guard<std::mutex> lock(debug_mutex);
      std::cout << "Thread " << thread_id << " 開始執行 (總共 " << num_threads
                << " 個 threads)" << std::endl;
    }
#endif

    active_threads++;
    Node current_node;
    // 3. 外層 BFS (狀態搜尋) - 多執行緒並行處理
    while (!solution_found) {
      bool got_work = pq.try_pop(current_node);

      if (!got_work) {
        // 如果沒有工作，檢查是否所有 threads 都在等待
        active_threads--;
        if (active_threads == 0) {
          // 所有 threads 都沒有工作，搜尋結束
          cv.notify_all();
          break;
        }

        // 等待一小段時間，看是否有新工作
        std::unique_lock<std::mutex> lock(cv_mutex);
        cv.wait_for(lock, std::chrono::milliseconds(1));
        active_threads++;
        continue;
      }

      // 有工作要做
      State current_state = current_node.state;
      std::cout << std::flush;

#ifdef DEBUG
      int current_step = step_count.fetch_add(1) + 1;
      {
        std::lock_guard<std::mutex> lock(debug_mutex);
        std::cout << "[Thread " << thread_id << "] 步驟 " << current_step
                  << ": 檢查狀態 - 玩家位置: (" << current_state.playerPos.r
                  << ", " << current_state.playerPos.c << ")" << std::endl;
        std::cout << "[Thread " << thread_id << "] 箱子位置: ";
        for (const auto &box : current_state.boxPositions) {
          std::cout << "(" << box.r << ", " << box.c << ") ";
        }
        std::cout << std::endl;
      }
#endif

      // 檢查是否達成目標
      bool solved = true;
      for (const auto &box : current_state.boxPositions) {
        if (goals.find(box) == goals.end()) {
          solved = false;
          break;
        }
      }

      // 找到解答，atomic 設置解答狀態
      if (solved) {

#ifdef DEBUG
        {
          std::lock_guard<std::mutex> lock(debug_mutex);
          std::cout << "[Thread " << thread_id
                    << "] 找到解答！將解答狀態設置為 true..." << std::endl;
        }
#endif
        std::lock_guard<std::mutex> lock(solution_mutex);
        if (!solution_found) {
          solution_found = true;
          solution_state = current_state;
#ifdef DEBUG
          {
            std::lock_guard<std::mutex> debug_lock(debug_mutex);
            std::cout << "[Thread " << thread_id << "] 成功設置解答狀態！"
                      << std::endl;
          }
#endif
        }
        break;
      }

#ifdef DEBUG
      {
        std::lock_guard<std::mutex> lock(debug_mutex);
        std::cout << "[Thread " << thread_id << "] 嘗試推動箱子..."
                  << std::endl;
      }
#endif

      // 產生所有可能的後繼狀態 (嘗試推動每一個箱子)
      for (int i = 0; i < static_cast<int>(current_state.boxPositions.size());
           ++i) {
        if (solution_found)
          break;
        const auto &box = current_state.boxPositions[i];

#ifdef DEBUG
        {
          std::lock_guard<std::mutex> lock(debug_mutex);
          std::cout << "[Thread " << thread_id << "] 檢查箱子 " << i
                    << " 在位置 (" << box.r << ", " << box.c << ")"
                    << std::endl;
        }
#endif

        // 嘗試四個方向
        for (int j = 0; j < 4; ++j) {
          if (solution_found)
            break;
          Point push_dir = DIRS[j];
          char push_char = MOVE_CHARS[j];

          Point box_dest = box + push_dir;
          Point player_start_pos =
              box + DIRS[(j + 2) % 4]; // 玩家要站的位置 (箱子反方向)

#ifdef DEBUG
          {
            std::lock_guard<std::mutex> lock(debug_mutex);
            std::cout << "[Thread " << thread_id << "] 嘗試方向 " << push_char
                      << " - 箱子目標: (" << box_dest.r << ", " << box_dest.c
                      << "), 玩家需站在: (" << player_start_pos.r << ", "
                      << player_start_pos.c << ")" << std::endl;
          }
#endif
          char dest_tile = static_map[box_dest.r][box_dest.c];
          if (dest_tile == '#' || dest_tile == '@') {

#ifdef DEBUG
            {
              std::lock_guard<std::mutex> lock(debug_mutex);
              std::cout << "[Thread " << thread_id
                        << "] 箱子目標位置是牆壁或脆弱地板，跳過" << std::endl;
            }
#endif

            continue; // 箱子不能上牆或脆弱地板
          }

          if (deadlock_points.find(box_dest) != deadlock_points.end()) {
#ifdef DEBUG
            {
              std::lock_guard<std::mutex> lock(debug_mutex);
              std::cout << "[Thread " << thread_id
                        << "] 箱子目標位置是死胡同，跳過" << std::endl;
            }
#endif
            continue;
          }

          bool occupied = false;
          for (const auto &other_box : current_state.boxPositions) {
            if (box_dest == other_box) {
              occupied = true;
              break;
            }
          }
          if (occupied) {
#ifdef DEBUG
            {
              std::lock_guard<std::mutex> lock(debug_mutex);
              std::cout << "[Thread " << thread_id
                        << "] 箱子目標位置被其他箱子佔據，跳過" << std::endl;
            }
#endif
            continue;
          }

          // 內層 BFS: 檢查玩家是否能走到推箱子的位置

#ifdef DEBUG
          {
            std::lock_guard<std::mutex> lock(debug_mutex);
            std::cout << "[Thread " << thread_id
                      << "] 檢查玩家是否能到達推箱位置..." << std::endl;
          }
#endif

          std::string player_moves =
              find_player_path(current_state.playerPos, player_start_pos,
                               current_state.boxPositions);
          if (player_moves != "none") {

#ifdef DEBUG
            {
              std::lock_guard<std::mutex> lock(debug_mutex);
              std::cout << "[Thread " << thread_id
                        << "] 玩家可以到達！移動序列: "
                        << (player_moves != "" ? player_moves : "直接推箱")
                        << std::endl;
            }
#endif

            // 產生新狀態
            State next_state;
            next_state.playerPos = box; // 推完後玩家在箱子原來的位置
            next_state.boxPositions = current_state.boxPositions;
            next_state.boxPositions[i] = box_dest;
            std::sort(next_state.boxPositions.begin(),
                      next_state.boxPositions.end());
            // 1. 安全地讀取 current_state 的 g_cost
            auto it_current = state_info_map.find(current_state);
            std::unordered_set<Point, PointHasher> new_frozen_boxes =
                it_current->second.frozen_boxes;

            if (is_deadlock(next_state, box_dest, new_frozen_boxes)) {
#ifdef DEBUG
              {
                std::lock_guard<std::mutex> lock(debug_mutex);
                std::cout << "[Thread " << thread_id
                          << "] 新狀態是 freeze deadlock，跳過" << std::endl;
              }
#endif
              continue;
            }

#ifdef DEBUG
            {
              std::lock_guard<std::mutex> lock(debug_mutex);
              std::cout << "[Thread " << thread_id
                        << "] 產生新狀態 - 玩家位置: ("
                        << next_state.playerPos.r << ", "
                        << next_state.playerPos.c << ")" << std::endl;
              std::cout << "[Thread " << thread_id << "] 新狀態箱子位置: ";
              for (const auto &new_box : next_state.boxPositions) {
                std::cout << "(" << new_box.r << ", " << new_box.c << ") ";
              }
              std::cout << std::endl;
            }
#endif

            int new_g = it_current->second.g_cost + 1;

#ifdef DEBUG
            {
              std::lock_guard<std::mutex> lock(debug_mutex);
              std::cout << "[Thread " << thread_id
                        << "] 計算新狀態 g_cost: " << new_g << std::endl;
            }
#endif

            // 2. 創建新狀態的資訊
            std::string normalized_move = normalize_player_position(next_state);
            StateInfo new_info = {new_g, current_state,
                                  player_moves + push_char + normalized_move,
                                  new_frozen_boxes};

#ifdef DEBUG
            {
              std::lock_guard<std::mutex> lock(debug_mutex);
              std::cout << "[Thread " << thread_id
                        << "] 創建新狀態資訊 - 移動序列: "
                        << player_moves + push_char << std::endl;
            }
#endif

            // 3. 嘗試原子性插入
            auto result_pair = state_info_map.insert({next_state, new_info});
            bool inserted_successfully = result_pair.second;

            if (inserted_successfully) {
              // 我們是第一個，成功佔位，直接加入工作佇列
#ifdef DEBUG
              {
                std::lock_guard<std::mutex> lock(debug_mutex);
                std::cout << "[Thread " << thread_id
                          << "] 成功插入新狀態到 state_info_map" << std::endl;
              }
#endif
              int new_h = calculate_h_cost(next_state);
              int new_f = new_g + new_h;
#ifdef DEBUG
              {
                std::lock_guard<std::mutex> lock(debug_mutex);
                std::cout << "[Thread " << thread_id
                          << "] 計算 h_cost: " << new_h << ", f_cost: " << new_f
                          << std::endl;
              }
#endif
              pq.push({next_state, new_f});
              cv.notify_one();
#ifdef DEBUG
              {
                std::lock_guard<std::mutex> lock(debug_mutex);
                std::cout << "[Thread " << thread_id << "] 新狀態已加入優先佇列"
                          << std::endl;
              }
#endif
            }

#ifdef DEBUG
            {
              std::lock_guard<std::mutex> lock(debug_mutex);
              std::cout << "[Thread " << thread_id
                        << "] 新狀態插入 g_cost_map: " << new_g << std::endl;
              std::cout << result_pair.second << std::endl;
            }
#endif
          }
        }
#ifdef DEBUG
        {
          std::lock_guard<std::mutex> lock(debug_mutex);
          std::cout << "[Thread " << thread_id
                    << "] 當前佇列大小: " << pq.size()
                    << ", 已訪問狀態數: " << state_info_map.size() << std::endl;
        }
#endif
      }
    }

    active_threads--;

#ifdef DEBUG
    {
      std::lock_guard<std::mutex> lock(debug_mutex);
      std::cout << "Thread " << thread_id << " 結束執行" << std::endl;
    }
#endif
  }

#ifdef DEBUG
  std::cout << "所有 threads 執行完畢，總共處理了 " << step_count.load()
            << " 個狀態" << std::endl;
#endif

  if (solution_found && solution_state) {
    std::string solution = "";
    State at = *solution_state;
    while (!(at.playerPos == initial_state.playerPos &&
             at.boxPositions == initial_state.boxPositions)) {

      auto it = state_info_map.find(at);
      if (it == state_info_map.end()) {
        std::cerr << "No solution found." << std::endl;
        return 1;
      }
      const StateInfo &p = it->second;
      solution = p.move_to_get_here + solution;
      at = p.parent_state;
    }
    std::cout << solution << std::endl;
  } else {
    std::cerr << "No solution found." << std::endl;
    return 1;
  }

  return 0;
}