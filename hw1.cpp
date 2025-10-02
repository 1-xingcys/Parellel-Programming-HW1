#include "tbb/concurrent_priority_queue.h"
#include "tbb/concurrent_unordered_map.h"
#include "tbb/concurrent_unordered_set.h"
#include <bitset>
#include <boost/functional/hash.hpp>
#include <condition_variable>
#include <fstream>
#include <iostream>
#include <mutex>
#include <omp.h>
#include <optional>
#include <queue>
#include <string>
#include <sys/resource.h>
#include <unordered_set>
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

// game state
struct State {
  unsigned char playerPos;
  std::bitset<MAX_MAP_SIZE> boxPositions;

  // for std::set and std::map
  bool operator<(const State &other) const {
    if (playerPos < other.playerPos)
      return true;
    if (other.playerPos < playerPos)
      return false;
    for (int i = 0; i < MAX_MAP_SIZE; ++i) {
      if (boxPositions[i] != other.boxPositions[i]) {
        return !boxPositions[i] && other.boxPositions[i];
      }
    }
    return false;
  }
  bool operator==(const State &other) const {
    return playerPos == other.playerPos && boxPositions == other.boxPositions;
  }
};

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

// for priority_queue
struct Node {
  State state;
  int f_cost;

  // TBB concurrent_priority_queue needs < operator
  bool operator<(const Node &other) const { return f_cost > other.f_cost; }
};

// For outside BFS
struct StateInfo {
  int g_cost;
  State parent_state;
  std::string move_to_get_here;
};

// global variables and helper functions
tbb::concurrent_unordered_map<State, StateInfo, StateHasher> state_info_map;

std::string static_map_backward;
std::bitset<MAX_MAP_SIZE> goals_bitset_backward;
std::unordered_set<unsigned char> goals_set_backward;
int distances[MAX_MAP_SIZE][MAX_MAP_SIZE] = {{0}};

unsigned char width, height, map_size;
const std::string MOVE_CHARS_BACKWARD = "SDWA";

int calculate_h_cost(const State &s) {
  int total_distance = 0;
  for (unsigned char box = 0; box < map_size; ++box) {
    if (!s.boxPositions.test(box))
      continue;
    int min_dist_for_this_box = 1e9;
    for (unsigned char goal : goals_set_backward) {
      int dist = distances[goal][box];
      if (dist < min_dist_for_this_box) {
        min_dist_for_this_box = dist;
      }
    }
    total_distance += min_dist_for_this_box;
  }
  return total_distance;
}

// check if the position is walkable for the player
bool is_walkable(const unsigned char p,
                 const std::bitset<MAX_MAP_SIZE> &boxes) {
  char tile = static_map_backward[p];
  if (tile == '#') return false;
  return !boxes.test(p);
}

// find the path from start to end for the player
std::string find_player_path(const unsigned char start, const unsigned char end,
                             const std::bitset<MAX_MAP_SIZE> &boxes) {
#ifdef DEBUG
  {
    std::cout << "[find_player_path] start: (" << start / width << ", "
              << start % width << ")" << std::endl;
    std::cout << "[find_player_path] end: (" << end / width << ", "
              << end % width << ")" << std::endl;
    std::cout << "[find_player_path] boxes: ";
    for (unsigned char pos = 0; pos < map_size; ++pos) {
      if (boxes.test(pos)) {
        std::cout << "(" << pos / width << ", " << pos % width << ") ";
      }
    }
    std::cout << std::endl;
  }
#endif
  std::queue<unsigned char> q;
  q.push(start);
  unsigned char parent_pos[MAX_MAP_SIZE];
  char parent_move[MAX_MAP_SIZE];
  std::bitset<MAX_MAP_SIZE> visited;
  visited.set(start);

  while (!q.empty()) {
    unsigned char curr = q.front();
    q.pop();

    if (curr == end) {
      std::string path = "";
      unsigned char at = end;
      while (!(at == start)) {
        path += parent_move[at];
        at = parent_pos[at];
      }
      return path;
    }

    for (int i = 0; i < 4; ++i) {
      unsigned char next = DIRS(curr, i);
      if (is_walkable(next, boxes) && !visited.test(next)) {
        visited.set(next);
        q.push(next);
        parent_pos[next] = curr;
        parent_move[next] = MOVE_CHARS_BACKWARD[i];
      }
    }
  }
  return "none";
}

std::string normalize_player_position(State &state) {
  // use BFS to find all reachable positions for the player
  std::queue<unsigned char> q;
  std::bitset<MAX_MAP_SIZE> visited;
  unsigned char start = state.playerPos;

  q.push(start);
  visited.set(start);

  unsigned char topLeft = start; // initialize to the start position

  while (!q.empty()) {
    unsigned char curr = q.front();
    q.pop();

    // check if it is the top-left position
    if (curr < topLeft) {
      topLeft = curr;
    }

    // explore four directions
    for (int i = 0; i < 4; ++i) {
      unsigned char next = DIRS(curr, i);
      if (is_walkable(next, state.boxPositions) && !visited.test(next)) {
        visited.set(next);
        q.push(next);
      }
    }
  }
  state.playerPos = topLeft;

  // if the top-left position is the current position, return empty string
  if (topLeft == start) {
    return "";
  }

  return find_player_path(start, topLeft, state.boxPositions);
}

void init_distances() {
  for (unsigned char goal = 0; goal < map_size; ++goal) {
    if (!goals_bitset_backward.test(goal))
      continue;
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
            (static_map_backward[next] == ' ' || static_map_backward[next] == '.')) {
          visited.set(next);
          q.push(next);
          distances[goal][next] = distances[goal][curr] + 1;
        }
      }
    }
  }
}

void init_backword_initial_state(const State &state,
                                 tbb::concurrent_priority_queue<Node> &pq) {
  std::bitset<MAX_MAP_SIZE> visited;
  for (unsigned char pos = 0; pos < map_size; ++pos) {
    if (static_map_backward[pos] != '#' && !state.boxPositions.test(pos) &&
        !visited.test(pos)) {
      visited.set(pos);
      std::queue<unsigned char> q;
      q.push(pos);

      // create new state
      State new_state = state;
      new_state.playerPos = pos;
      int h = calculate_h_cost(new_state);
      pq.push({new_state, h});
      state_info_map.insert(
          {new_state, {0, new_state, ""}});

      // bfs to eliminate all reachable positions
      while (!q.empty()) {
        unsigned char curr = q.front();
        q.pop();
        for (int i = 0; i < 4; ++i) {
          unsigned char next = DIRS(curr, i);
          if (static_map_backward[next] != '#' &&
              !state.boxPositions.test(next) && !visited.test(next)) {
            visited.set(next);
            q.push(next);
          }
        }
      }
    }
  }
}


int main(int argc, char *argv[]) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <map_file>" << std::endl;
    return 1;
  }

  // 1. read and parse the map
  std::ifstream file(argv[1]);
  if (!file) {
    std::cerr << "Error: Cannot open file " << argv[1] << std::endl;
    return 1;
  }

  State initial_backward_state;
  std::string line;
  unsigned char r = 0;
  unsigned char backword_end_player_pos = 0;
  while (std::getline(file, line)) {
    width = line.length();
    static_map_backward += line;
    for (unsigned char c = 0; c < static_cast<int>(line.length()); ++c) {
      char tile = line[c];
      if (tile == 'o' || tile == 'O' || tile == '!') {
        backword_end_player_pos = r * width + c;
      }
      if (tile == 'x' || tile == 'X') {
        goals_bitset_backward.set(r * width + c);
        goals_set_backward.insert(r * width + c);
      }
      if (tile == '.' || tile == 'O' || tile == 'X') {
        initial_backward_state.boxPositions.set(r * width + c);
      }

      if (tile == 'o' || tile == '.' || tile == 'O')
        static_map_backward[r * width + c] = ' ';
      else if (tile == 'x' || tile == 'X')
        static_map_backward[r * width + c] = '.';
      else if (tile == '!')
        static_map_backward[r * width + c] = '@';
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

  init_distances();

  // 2. BFS initialization

  // BFS variables
  tbb::concurrent_priority_queue<Node> pq;

  init_backword_initial_state(initial_backward_state, pq);

  // parallel processing variables
  std::atomic<bool> solution_found = false;
  std::optional<State> solution_state;
  std::mutex solution_mutex;

  // record the number of threads working
  std::atomic<int> active_threads = 0;
  std::condition_variable cv;
  std::mutex cv_mutex;

#ifdef DEBUG
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
    // 3. state search - parallel processing
    while (!solution_found) {
      bool got_work = pq.try_pop(current_node);

      if (!got_work) {
        // if there is no work, check if all threads are waiting
        active_threads--;
        if (active_threads == 0) {
          // if all threads are waiting, the search is over
          cv.notify_all();
          break;
        }

        // wait for a short time, check if there is new work
        std::unique_lock<std::mutex> lock(cv_mutex);
        cv.wait_for(lock, std::chrono::milliseconds(1));
        active_threads++;
        continue;
      }

      // there is work to do
      State current_state = current_node.state;

#ifdef DEBUG
      int current_step = step_count.fetch_add(1) + 1;
#endif

       // backward states search
        for (unsigned char i = 0; i < map_size; ++i) {
          if (solution_found)
            break;
          if (!current_state.boxPositions.test(i))
            continue;

          unsigned char box = i;
#ifdef DEBUG
          {
            std::lock_guard<std::mutex> lock(debug_mutex);
            std::cout << "[Thread " << thread_id << "] 處理箱子位置: ("
                      << box / width << ", " << box % width << ")" << std::endl;
          }
#endif
          for (int j = 0; j < 4; ++j) {
            if (solution_found)
              break;
            unsigned char box_dest = DIRS(box, j);
            char push_char = MOVE_CHARS_BACKWARD[j];
            unsigned char player_start_pos = box_dest;

            // if the player start position is a box or a wall, skip
            if (current_state.boxPositions.test(player_start_pos) ||
                static_map_backward[player_start_pos] == '#' || static_map_backward[player_start_pos] == '@')
              continue;

            int try_player_end_pos = DIRS(player_start_pos, j);
            if (try_player_end_pos == -1)
              continue;
            unsigned char player_end_pos = try_player_end_pos;

            // if the player end position is a box or a wall, skip
            if (current_state.boxPositions.test(player_end_pos) ||
                static_map_backward[player_end_pos] == '#')
              continue;

#ifdef DEBUG
            {
              std::lock_guard<std::mutex> lock(debug_mutex);
              std::cout << "[Thread " << thread_id << "] 嘗試方向 " << j
                        << " (推動字符: " << push_char << ")" << std::endl;
              std::cout << "[Thread " << thread_id << "] 箱子目標位置: ("
                        << box_dest / width << ", " << box_dest % width << ")"
                        << std::endl;
              std::cout << "[Thread " << thread_id << "] 玩家結束位置: ("
                        << player_end_pos / width << ", "
                        << player_end_pos % width << ")" << std::endl;
              std::cout << "[Thread " << thread_id << "] 玩家目前位置: ("
                        << current_state.playerPos / width << ", "
                        << current_state.playerPos % width << ")" << std::endl;
              std::cout << "[Thread " << thread_id
                        << "] current_state 的 boxPositions: ";
              for (unsigned char pos = 0; pos < map_size; ++pos) {
                if (current_state.boxPositions.test(pos)) {
                  std::cout << "(" << pos / width << ", " << pos % width
                            << ") ";
                }
              }
              std::cout << std::endl;
            }
#endif

            std::string player_moves =
                find_player_path(current_state.playerPos, player_start_pos,
                                 current_state.boxPositions);

            // if the player can moves to the start position, explore!
            if (player_moves != "none") {
              State next_state;
              next_state.playerPos = player_end_pos;
              next_state.boxPositions = current_state.boxPositions;
              next_state.boxPositions.reset(box);
              next_state.boxPositions.set(box_dest);

              auto it_current = state_info_map.find(current_state);
              int new_g = it_current->second.g_cost + 1;

              // normalize the player position, to reduce the search space
              std::string normalized_move =
                  normalize_player_position(next_state);
              bool solved = false;

              // handle the case that all boxes are at the goals, we need to check if the player can moves to the initial position
              if ((goals_bitset_backward & ~next_state.boxPositions).none()) {
                std::string player_moves_to_initial_pos = find_player_path(
                    next_state.playerPos, backword_end_player_pos,
                    next_state.boxPositions);
                if (player_moves_to_initial_pos != "none") {
                  normalized_move = player_moves_to_initial_pos + normalized_move;
                  solved = true;
                }
              }
#ifdef DEBUG
              {
                std::lock_guard<std::mutex> lock(debug_mutex);
                std::cout << "[Thread " << thread_id
                          << "] 創建新狀態資訊 - 移動序列: " << player_moves
                          << " + " << push_char << " + " << normalized_move
                          << std::endl;
              }
#endif
              StateInfo new_info = {new_g, current_state,
                                    normalized_move + push_char + player_moves};
                                    
              // try to insert the new state atomically
              auto result_pair = state_info_map.insert({next_state, new_info});
              bool inserted_successfully = result_pair.second;

              if (inserted_successfully) {
                if (solved) {
                  std::lock_guard<std::mutex> lock(solution_mutex);
                  if (!solution_found) {
                    solution_found = true;
                    solution_state = next_state;
#ifdef DEBUG
                    {
                      std::lock_guard<std::mutex> debug_lock(debug_mutex);
                      std::cout << "[Thread " << thread_id
                                << "] 成功設置解答狀態！" << std::endl;
                    }
#endif
                  }
                  break;
                }
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
                pq.push({next_state, new_f});
                cv.notify_one();
#ifdef DEBUG
                {
                  std::lock_guard<std::mutex> lock(debug_mutex);
                  std::cout << "[Thread " << thread_id
                            << "] 新狀態已加入優先佇列" << std::endl;
                }
#endif
              } else {
#ifdef DEBUG
                {
                  std::lock_guard<std::mutex> lock(debug_mutex);
                  std::cout << "[Thread " << thread_id
                            << "] 狀態已存在於 state_info_map，跳過"
                            << std::endl;
                }
#endif
              }
            } else {
#ifdef DEBUG
              {
                std::lock_guard<std::mutex> lock(debug_mutex);
                std::cout << "[Thread " << thread_id
                          << "] 找不到玩家路徑，跳過此移動" << std::endl;
              }
#endif
            }
        }
      }
    }

    active_threads--;

#ifdef DEBUG
    {
      std::lock_guard<std::mutex> lock(debug_mutex);
      std::cout << "Thread " << thread_id << " 結束執行" << std::endl;
    }
#endif
  } // end of parallel region

#ifdef DEBUG
  std::cout << "所有 threads 執行完畢，總共處理了 " << step_count.load()
            << " 個狀態" << std::endl;
#endif

  if (solution_found && solution_state) {
    std::string solution = "";
    State at = *solution_state;
      while (state_info_map[at].g_cost != 0) {
        auto it = state_info_map.find(at);
        if (it == state_info_map.end()) {
          std::cerr << "No solution found." << std::endl;
          return 1;
        }
        const StateInfo &p = it->second;
        solution += p.move_to_get_here;
        at = p.parent_state;
    }
    std::cout << solution << std::endl;
  } else {
    std::cerr << "No solution found." << std::endl;
    return 1;
  }

  return 0;
}