#include "tbb/concurrent_priority_queue.h"
#include "tbb/concurrent_unordered_map.h"
#include "tbb/concurrent_unordered_set.h"
#include <algorithm>
#include <bitset>
#include <boost/functional/hash.hpp>
#include <condition_variable>
#include <fstream>
#include <iostream>
#include <map>
#include <mutex>
#include <omp.h>
#include <optional>
#include <queue>
#include <unordered_set>
#include <string>
#include <utility>

// #define DEBUG 1
#define AXIS_VERTICAL 0
#define AXIS_HORIZONTAL 1
#define MAX_MAP_SIZE 256
#define DIRS(pos, dir_code)                                                    \
  ((dir_code) == 0   ? ((pos) < width ? -1 : (pos)-width)                      \
   : (dir_code) == 1 ? ((pos) % width == 0 ? -1 : (pos)-1)                     \
   : (dir_code) == 2 ? ((pos) >= width * (height - 1) ? -1 : (pos) + width)    \
   : (dir_code) == 3 ? ((pos) % width == width - 1 ? -1 : (pos) + 1)           \
                     : -1)

struct UnsignedCharPairHasher {
  std::size_t
  operator()(const std::pair<unsigned char, unsigned char> &pp) const {
    std::size_t seed = 0;
    boost::hash_combine(seed, pp.first);
    boost::hash_combine(seed, pp.second);
    return seed;
  }
};

// --- 遊戲狀態結構 ---
struct State {
  unsigned char playerPos;
  std::bitset<MAX_MAP_SIZE> boxPositions; // 保持排序以作為唯一標識

  // 為了能放入 std::set 和 std::map
  bool operator<(const State &other) const {
    if (playerPos < other.playerPos)
      return true;
    if (other.playerPos < playerPos)
      return false;
    // 逐位比較 bitset，避免 to_ulong() overflow
    for (int i = 0; i < MAX_MAP_SIZE; ++i) {
      if (boxPositions[i] != other.boxPositions[i]) {
        return !boxPositions[i] && other.boxPositions[i];
      }
    }
    return false; // 完全相等
  }
  bool operator==(const State &other) const {
    return playerPos == other.playerPos && boxPositions == other.boxPositions;
  }
};

// 為 State 結構提供一個雜湊函式 (Hasher)
struct StateHasher {
  std::size_t operator()(const State &s) const {
    std::size_t seed = 0;
    char boxPositions[MAX_MAP_SIZE];
    for (int i = 0; i < MAX_MAP_SIZE; ++i) {
      boxPositions[i] = s.boxPositions[i];
    }
    boost::hash_combine(seed, s.playerPos);
    boost::hash_combine(seed, boxPositions);
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
  std::bitset<MAX_MAP_SIZE> frozen_boxes;
};

// --- 全域變數和輔助函數 ---
// std::vector<std::string> static_map;
std::string static_map;
std::bitset<MAX_MAP_SIZE> goals_bitset;
std::unordered_set<unsigned char> goals_set;
std::unordered_map<std::pair<unsigned char, unsigned char>, int,
                   UnsignedCharPairHasher>
    distances;
tbb::concurrent_unordered_map<State, StateInfo, StateHasher> state_info_map;
std::bitset<MAX_MAP_SIZE> deadlock_points;

// 方向向量 (Up, Left, Down, Right)
unsigned char width, height, map_size;
const std::string MOVE_CHARS = "WASD"; // 注意順序對應 DIRS

int calculate_h_cost(const State &s) {
  int total_distance = 0;
  for (unsigned char box = 0; box < map_size; ++box) {
    if (!s.boxPositions.test(box))
      continue;
    int min_dist_for_this_box = 1e9;
    for (unsigned char goal : goals_set) {
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
bool is_walkable(const unsigned char p,
                 const std::bitset<MAX_MAP_SIZE> &boxes) {
  char tile = static_map[p];
  if (tile == '#') {
    return false;
  }
  // 檢查是否撞到箱子
  return !boxes.test(p);
}

bool IsBlockedOnAxis(
    const unsigned char box, const std::bitset<MAX_MAP_SIZE> &candidateBoxes,
    std::bitset<MAX_MAP_SIZE>
        &checkedBoxes, // boxes already checked in recursive steps
    std::unordered_map<unsigned char, bool> &stuck_status_horizontal,
    std::unordered_map<unsigned char, bool> &stuck_status_vertical, int axis) {
  unsigned char pos1 = DIRS(box, axis);
  unsigned char pos2 = DIRS(box, axis + 2);
  checkedBoxes.set(box);

  // Check stuck by wall
  if (static_map[pos1] == '#' || static_map[pos2] == '#') {
    checkedBoxes.reset(box);
    return true;
  }

  // Check stuck by deadlock point
  if (deadlock_points.test(pos1) && deadlock_points.test(pos2)) {
    checkedBoxes.reset(box);
    return true;
  }

  // Check stuck by other boxes

  // cycle check
  if (checkedBoxes.test(pos1) || checkedBoxes.test(pos2)) {
    checkedBoxes.reset(box);
    return true;
  }

  bool pos_stucks[2] = {false, false};
  unsigned char poses[2] = {pos1, pos2};

  for (unsigned char i = 0; i < 2; ++i) {
    unsigned char pos = poses[i];
    if (candidateBoxes.test(pos)) {
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
        checkedBoxes.reset(box);
        return true;
      }
    }
  }
  checkedBoxes.reset(box);
  return false;
}

bool is_dead_goal(const std::bitset<MAX_MAP_SIZE> &candidateBoxes,
                  const std::bitset<MAX_MAP_SIZE> &frozenBoxes) {
  // check every unmatched goals if it is a dead goal
  for (unsigned char goal : goals_set) {
    // regard the frozen boxes as walls, do BFS to check if any candidate box is
    // reachable from the goal
    if (frozenBoxes.test(goal))
      continue;
    bool is_dead = true;
    std::queue<unsigned char> q;
    q.push(goal);
    std::bitset<MAX_MAP_SIZE> visited;
    visited.set(goal);
    while (!q.empty()) {
      unsigned char curr = q.front();
      q.pop();
      if (candidateBoxes.test(curr) && !frozenBoxes.test(curr)) {
        is_dead = false;
        break;
      }
      for (int i = 0; i < 4; ++i) {
        unsigned char next = DIRS(curr, i);
        int try_next2 = DIRS(next, i);
        if (try_next2 == -1)
          continue;
        unsigned char next2 = try_next2;
        if (!visited.test(next) && static_map[next] != '#' &&
            !frozenBoxes.test(next) && static_map[next2] != '#' &&
            !frozenBoxes.test(next2)) {
          visited.set(next);
          q.push(next);
        }
      }
    }
    if (is_dead)
      return true;
  }
  return false;
}

bool is_dead_box(const std::bitset<MAX_MAP_SIZE> &candidateBoxes,
                 const std::bitset<MAX_MAP_SIZE> &frozenBoxes) {
  std::bitset<MAX_MAP_SIZE> unFrozenBoxes = candidateBoxes & ~frozenBoxes;
  for (unsigned char box = 0; box < map_size; ++box) {
    if (!unFrozenBoxes.test(box))
      continue;
    // regard the frozen boxes as walls, do BFS to check if any goal is
    // reachable from the box
    bool is_dead = true;
    std::queue<unsigned char> q;
    q.push(box);
    std::bitset<MAX_MAP_SIZE> visited;
    visited.set(box);
    while (!q.empty()) {
      unsigned char curr = q.front();
      q.pop();
      if (goals_bitset.test(curr)) {
        is_dead = false;
        break;
      }
      for (int i = 0; i < 4; ++i) {
        unsigned char next = DIRS(curr, i);
        unsigned char prev = DIRS(curr, (i + 2) % 4);
        if (!visited.test(next) && static_map[next] != '#' &&
            !frozenBoxes.test(next) && static_map[prev] != '#' &&
            !frozenBoxes.test(prev)) {
          visited.set(next);
          q.push(next);
        }
      }
    }
    if (is_dead)
      return true;
  }
  return false;
}

bool is_deadlock(const State &newState, const unsigned char movedBox,
                 std::bitset<MAX_MAP_SIZE> &frozenBoxes) {
  std::bitset<MAX_MAP_SIZE> candidateBoxes;
  std::bitset<MAX_MAP_SIZE> all_boxes(newState.boxPositions);
  // BFS from movedBox to find connected boxes
  std::queue<unsigned char> q;
  q.push(movedBox);
  std::bitset<MAX_MAP_SIZE> visited;
  visited.set(movedBox);
  while (!q.empty()) {
    unsigned char curr = q.front();
    q.pop();
    candidateBoxes.set(curr);
    for (int i = 0; i < 4; ++i) {
      unsigned char next = DIRS(curr, i);
      if (all_boxes.test(next) && !visited.test(next)) {
        visited.set(next);
        q.push(next);
      }
    }
  }

  // For freeze deadlock
  std::unordered_map<unsigned char, bool> stuck_status_horizontal;
  std::unordered_map<unsigned char, bool> stuck_status_vertical;
  std::bitset<MAX_MAP_SIZE> checkedBoxes;
  std::bitset<MAX_MAP_SIZE> unFrozenBoxes = candidateBoxes & ~frozenBoxes;

  for (unsigned char box = 0; box < map_size; ++box) {
    if (!unFrozenBoxes.test(box))
      continue;
    bool horizontally_stuck = false, vertically_stuck = false;
    // Check if the box is horizontally stuck
    if (stuck_status_horizontal.find(box) != stuck_status_horizontal.end()) {
      horizontally_stuck = stuck_status_horizontal[box];
    } else {
      checkedBoxes.reset();
      horizontally_stuck = IsBlockedOnAxis(
          box, candidateBoxes, checkedBoxes, stuck_status_horizontal,
          stuck_status_vertical, AXIS_HORIZONTAL);
      stuck_status_horizontal[box] = horizontally_stuck;
    }

    // Check if the box is vertically stuck
    if (stuck_status_vertical.find(box) != stuck_status_vertical.end()) {
      vertically_stuck = stuck_status_vertical[box];
    } else {
      checkedBoxes.reset();
      vertically_stuck = IsBlockedOnAxis(box, candidateBoxes, checkedBoxes,
                                         stuck_status_horizontal,
                                         stuck_status_vertical, AXIS_VERTICAL);
      stuck_status_vertical[box] = vertically_stuck;
    }

    if (horizontally_stuck && vertically_stuck) {
      frozenBoxes.set(box);
    }
  }

  // If there is any frozen box not in goal, it is a freeze deadlock
  if (frozenBoxes.none())
    return false;
  if ((frozenBoxes & ~goals_bitset).any())
    return true;
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
std::string find_player_path(const unsigned char start, const unsigned char end,
                             const std::bitset<MAX_MAP_SIZE> &boxes) {
  std::queue<unsigned char> q;
  q.push(start);
  std::map<unsigned char, std::pair<unsigned char, char>> parent_map;
  std::bitset<MAX_MAP_SIZE> visited;
  visited.set(start);

  while (!q.empty()) {
    unsigned char curr = q.front();
    q.pop();

    if (curr == end) {
      std::string path = "";
      unsigned char at = end;
      while (!(at == start)) {
        auto &p = parent_map[at];
        path += p.second;
        at = p.first;
      }
      std::reverse(path.begin(), path.end());
      return path;
    }

    for (int i = 0; i < 4; ++i) {
      unsigned char next = DIRS(curr, i);
      if (is_walkable(next, boxes) && !visited.test(next)) {
        visited.set(next);
        q.push(next);
        parent_map[next] = {curr, MOVE_CHARS[i]};
      }
    }
  }
  return "none"; // 找不到路徑
}

std::string normalize_player_position(State &state) {
  // 使用 BFS 找到玩家能到達的所有位置
  std::queue<unsigned char> q;
  std::bitset<MAX_MAP_SIZE> visited;
  unsigned char start = state.playerPos;

  q.push(start);
  visited.set(start);

  unsigned char topLeft = start; // 初始化為起始位置

  while (!q.empty()) {
    unsigned char curr = q.front();
    q.pop();

    // 檢查是否是更 top-left 的位置
    if (curr < topLeft) {
      topLeft = curr;
    }

    // 探索四個方向
    for (int i = 0; i < 4; ++i) {
      unsigned char next = DIRS(curr, i);
      if (is_walkable(next, state.boxPositions) && !visited.test(next)) {
        visited.set(next);
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
  for (unsigned char goal : goals_set) {
    std::queue<unsigned char> q;
    q.push(goal);
    std::bitset<MAX_MAP_SIZE> visited;
    visited.set(goal);
    while (!q.empty()) {
      unsigned char curr = q.front();
      q.pop();
      for (int i = 0; i < 4; ++i) {
        int try_next = DIRS(curr, i);
        if (try_next == -1)
          continue;
        unsigned char next = try_next;
        if (!visited.test(next) &&
            (static_map[next] == ' ' || static_map[next] == '.')) {
          visited.set(next);
          q.push(next);
          distances[{goal, next}] = distances[{goal, curr}] + 1;
        }
      }
    }
  }
}

void init_deadlock_points() {
  std::bitset<MAX_MAP_SIZE> safe;
  for (unsigned char goal : goals_set) {
    std::queue<unsigned char> q;
    q.push(goal);
    std::bitset<MAX_MAP_SIZE> visited;
    visited.set(goal);
    while (!q.empty()) {
      unsigned char curr = q.front();
      q.pop();
      safe.set(curr);
      for (int i = 0; i < 4; ++i) {
        int try_next = DIRS(curr, i);
        if (try_next == -1)
          continue;
        unsigned char next = try_next;
        int try_next2 = DIRS(next, i);
        if (try_next2 == -1)
          continue;
        unsigned char next2 = try_next2;

        if (visited.test(next))
          continue;
        if ((static_map[next] == ' ' || static_map[next] == '.') &&
            (static_map[next2] == ' ' || static_map[next2] == '.' ||
             static_map[next2] == '@')) {
          visited.set(next);
          q.push(next);
        }
      }
    }
  }
  for (unsigned char i = 0; i < map_size; ++i) {
    if (!safe.test(i) && static_map[i] == ' ') {
      deadlock_points.set(i);
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
  unsigned char r = 0;
  while (std::getline(file, line)) {
    width = line.length();
    static_map += line;
    for (unsigned char c = 0; c < static_cast<int>(line.length()); ++c) {
      char tile = line[c];
      if (tile == 'o' || tile == 'O' || tile == '!') {
        initial_state.playerPos = r * width + c;
      }
      if (tile == 'x' || tile == 'X') {
        initial_state.boxPositions.set(r * width + c);
      }
      if (tile == '.' || tile == 'O' || tile == 'X') {
        goals_bitset.set(r * width + c);
        goals_set.insert(r * width + c);
      }
      // 將地圖簡化為靜態部分
      if (tile == 'o' || tile == 'x' || tile == ' ')
        static_map[r * width + c] = ' ';
      else if (tile == 'O' || tile == 'X' || tile == '.')
        static_map[r * width + c] = '.';
      else if (tile == '!')
        static_map[r * width + c] = '@';
    }
    r++;
  }
  height = r;
  map_size = width * height;
#ifdef DEBUG
  std::cout << "width: " << static_cast<int>(width)
            << ", height: " << static_cast<int>(height)
            << ", map_size: " << static_cast<int>(map_size) << std::endl;
#endif
  file.close();

  // 初始化距離
  init_distances();
  init_deadlock_points();

#ifdef DEBUG
  // 輸出初始狀態資訊
  std::cout << "初始狀態資訊:" << std::endl;
  std::cout << "玩家位置: (" << initial_state.playerPos % width << ", "
            << initial_state.playerPos / width << ")" << std::endl;
  std::cout << "箱子數量: " << initial_state.boxPositions.size() << std::endl;
  std::cout << "箱子位置: ";
  for (unsigned char box = 0; box < map_size; ++box) {
    if (initial_state.boxPositions.test(box)) {
      std::cout << "(" << box % width << ", " << box / width << ") ";
    }
  }
  std::cout << std::endl;
  std::cout << "目標數量: " << goals_bitset.size() << std::endl;
  std::cout << "目標位置: ";
  for (unsigned char goal = 0; goal < map_size; ++goal) {
    if (goals_bitset.test(goal)) {
      std::cout << "(" << goal % width << ", " << goal / width << ") ";
    }
  }
  std::cout << std::endl;

  // 印出 static_map
  std::cout << "static_map:" << std::endl;
  for (unsigned char pos = 0; pos < map_size; ++pos) {
    std::cout << static_map[pos];
    if (pos % width == width - 1) {
      std::cout << std::endl;
    }
  }
  std::cout << "distances:" << std::endl;
  for (const auto &[key, value] : distances) {
    std::cout << "(" << key.first % width << ", " << key.first / width
              << ") -> (" << key.second % width << ", " << key.second / width
              << "): " << value << std::endl;
  }
  std::cout << "deadlock_points:" << std::endl;
  for (unsigned char point = 0; point < map_size; ++point) {
    if (deadlock_points.test(point)) {
      std::cout << "(" << point % width << ", " << point / width << ") ";
    }
  }
  std::cout << std::endl;
#endif

  // 2. BFS 初始化

  // BFS 算法所需變數
  int initial_g = 0;
  int initial_h = calculate_h_cost(initial_state);
  tbb::concurrent_priority_queue<Node> pq;
  pq.push({initial_state, initial_g + initial_h});
  std::bitset<MAX_MAP_SIZE> initial_frozen_boxes(0);
  initial_frozen_boxes.reset();
  state_info_map.insert(
      {initial_state, {initial_g, initial_state, "", initial_frozen_boxes}});

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
                  << ": 檢查狀態 - 玩家位置: ("
                  << current_state.playerPos / width << ", "
                  << current_state.playerPos % width << ")" << std::endl;
        std::cout << "[Thread " << thread_id << "] 箱子位置: ";
        for (unsigned char box = 0; box < map_size; ++box) {
          if (current_state.boxPositions.test(box)) {
            std::cout << "(" << box / width << ", " << box % width << ") ";
          }
        }
        std::cout << std::endl;
      }
#endif

      // 檢查是否達成目標
      bool solved = true;
      if ((goals_bitset & ~current_state.boxPositions).any()) {
        solved = false;
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
      for (unsigned char i = 0; i < map_size; ++i) {
        if (solution_found)
          break;
        if (!current_state.boxPositions.test(i))
          continue;
        unsigned char box = i;

#ifdef DEBUG
        {
          std::lock_guard<std::mutex> lock(debug_mutex);
          std::cout << "[Thread " << thread_id << "] 檢查箱子 "
                    << static_cast<int>(i) << " 在位置 (" << box / width << ", "
                    << box % width << ")" << std::endl;
        }
#endif

        // 嘗試四個方向
        for (int j = 0; j < 4; ++j) {
          if (solution_found)
            break;
          unsigned char box_dest = DIRS(box, j);
          char push_char = MOVE_CHARS[j];

          unsigned char player_start_pos =
              DIRS(box, (j + 2) % 4); // 玩家要站的位置 (箱子反方向)

#ifdef DEBUG
          {
            std::lock_guard<std::mutex> lock(debug_mutex);
            std::cout << "[Thread " << thread_id << "] 嘗試方向 " << push_char
                      << " - 箱子目標: (" << box_dest / width << ", "
                      << box_dest % width << "), 玩家需站在: ("
                      << player_start_pos / width << ", "
                      << player_start_pos % width << ")" << std::endl;
          }
#endif
          char dest_tile = static_map[box_dest];
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

          if (deadlock_points.test(box_dest)) {
#ifdef DEBUG
            {
              std::lock_guard<std::mutex> lock(debug_mutex);
              std::cout << "[Thread " << thread_id
                        << "] 箱子目標位置是死胡同，跳過" << std::endl;
            }
#endif
            continue;
          }

          if (current_state.boxPositions.test(box_dest)) {
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
            next_state.boxPositions.reset(box);
            next_state.boxPositions.set(box_dest);
            // 1. 安全地讀取 current_state 的 g_cost
            auto it_current = state_info_map.find(current_state);
            std::bitset<MAX_MAP_SIZE> new_frozen_boxes =
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
                        << next_state.playerPos / width << ", "
                        << next_state.playerPos % width << ")" << std::endl;
              std::cout << "[Thread " << thread_id << "] 新狀態箱子位置: ";
              for (unsigned char new_box = 0; new_box < map_size; ++new_box) {
                if (next_state.boxPositions.test(new_box)) {
                  std::cout << "(" << new_box / width << ", " << new_box % width
                            << ") ";
                }
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