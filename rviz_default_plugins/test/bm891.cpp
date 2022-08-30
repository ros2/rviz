#include <benchmark/benchmark.h>

#include <limits>
#include <mutex>
#include <set>
#include <unordered_set>
#include <utility>
#include <vector>


#include <visualization_msgs/msg/marker_array.hpp>


void method_no_check(const visualization_msgs::msg::MarkerArray * array)
{
  std::vector<visualization_msgs::msg::Marker::SharedPtr> message_queue_;
  std::mutex queue_mutex_;
  // ==========
  std::unique_lock<std::mutex> lock(queue_mutex_);
  for (auto const & marker : array->markers) {
    message_queue_.push_back(std::make_shared<visualization_msgs::msg::Marker>(marker));
  }
}


void method_vector_of_pairs(const visualization_msgs::msg::MarkerArray * array)
{
  std::vector<visualization_msgs::msg::Marker::SharedPtr> message_queue_;
  std::mutex queue_mutex_;
  // ==========
  using ns_type = decltype(visualization_msgs::msg::Marker::ns);
  using id_type = decltype(visualization_msgs::msg::Marker::id);
  using pair_type = std::pair<const ns_type &, id_type>;

  // Keep track of unique markers
  std::vector<pair_type> unique_markers;
  unique_markers.reserve(array->markers.size());
  id_type biggest_id = std::numeric_limits<id_type>::lowest();
  bool found_duplicate = false;

  std::unique_lock<std::mutex> lock(queue_mutex_);
  for (auto const & marker : array->markers) {
    bool is_duplicate = false;
    // Comparing against the biggest id will avoid searching the unique_marker list when
    // someone creates a lot of markers by incrementing the id.
    if (marker.id > biggest_id) {
      biggest_id = marker.id;
    } else if (!found_duplicate) {
      // Look for a duplicate marker
      for (const auto & ns_id : unique_markers) {
        // Compare id first because it's a numeric type.
        if (ns_id.second == marker.id && ns_id.first == marker.ns) {
          found_duplicate = true;
          is_duplicate = true;
          break;
        }
      }
    }
    if (!is_duplicate) {
      unique_markers.push_back(pair_type(marker.ns, marker.id));
    }
    message_queue_.push_back(std::make_shared<visualization_msgs::msg::Marker>(marker));
  }
}


void method_set_insertion(const visualization_msgs::msg::MarkerArray * array)
{
  std::vector<visualization_msgs::msg::Marker::SharedPtr> message_queue_;
  std::mutex queue_mutex_;
  // ==========
  using ns_type = decltype(visualization_msgs::msg::Marker::ns);
  using id_type = decltype(visualization_msgs::msg::Marker::id);
  using pair_type = std::pair<const ns_type &, id_type>;

  // Keep track of unique markers
  std::set<pair_type> unique_markers;
  // std::set doesn't have a way to reserve memory
  // unique_markers.reserve(array->markers.size());
  bool found_duplicate = false;

  std::unique_lock<std::mutex> lock(queue_mutex_);
  for (auto const & marker : array->markers) {
    if (!found_duplicate) {
      pair_type pair(marker.ns, marker.id);
      found_duplicate = !unique_markers.insert(pair).second;
    }
    message_queue_.push_back(std::make_shared<visualization_msgs::msg::Marker>(marker));
  }
}

void method_set_insertion_id_first(const visualization_msgs::msg::MarkerArray * array)
{
  std::vector<visualization_msgs::msg::Marker::SharedPtr> message_queue_;
  std::mutex queue_mutex_;
  // ==========
  using ns_type = decltype(visualization_msgs::msg::Marker::ns);
  using id_type = decltype(visualization_msgs::msg::Marker::id);
  using pair_type = std::pair<id_type, const ns_type &>;

  // Keep track of unique markers
  std::set<pair_type> unique_markers;
  // std::set doesn't have a way to reserve memory
  // unique_markers.reserve(array->markers.size());
  bool found_duplicate = false;

  std::unique_lock<std::mutex> lock(queue_mutex_);
  for (auto const & marker : array->markers) {
    if (!found_duplicate) {
      pair_type pair(marker.id, marker.ns);
      found_duplicate = !unique_markers.insert(pair).second;
    }
    message_queue_.push_back(std::make_shared<visualization_msgs::msg::Marker>(marker));
  }
}

void method_set_insertion_id_first_always_lock(const visualization_msgs::msg::MarkerArray * array)
{
  std::vector<visualization_msgs::msg::Marker::SharedPtr> message_queue_;
  std::mutex queue_mutex_;
  // ==========
  using ns_type = decltype(visualization_msgs::msg::Marker::ns);
  using id_type = decltype(visualization_msgs::msg::Marker::id);
  using pair_type = std::pair<id_type, const ns_type &>;

  // Keep track of unique markers
  std::set<pair_type> unique_markers;
  // std::set doesn't have a way to reserve memory
  // unique_markers.reserve(array->markers.size());
  bool found_duplicate = false;

  for (auto const & marker : array->markers) {
    if (!found_duplicate) {
      pair_type pair(marker.id, marker.ns);
      found_duplicate = !unique_markers.insert(pair).second;
    }
    std::unique_lock<std::mutex> lock(queue_mutex_);
    message_queue_.push_back(std::make_shared<visualization_msgs::msg::Marker>(marker));
  }
}

void method_set_insertion_id_first_always_lock_emplace(const visualization_msgs::msg::MarkerArray * array)
{
  std::vector<visualization_msgs::msg::Marker::SharedPtr> message_queue_;
  std::mutex queue_mutex_;
  // ==========
  using ns_type = decltype(visualization_msgs::msg::Marker::ns);
  using id_type = decltype(visualization_msgs::msg::Marker::id);
  using pair_type = std::pair<id_type, const ns_type &>;

  // Keep track of unique markers
  std::set<pair_type> unique_markers;
  // std::set doesn't have a way to reserve memory
  // unique_markers.reserve(array->markers.size());
  bool found_duplicate = false;

  for (auto const & marker : array->markers) {
    if (!found_duplicate) {
      found_duplicate = !unique_markers.emplace(marker.id, marker.ns).second;
    }
    std::unique_lock<std::mutex> lock(queue_mutex_);
    message_queue_.push_back(std::make_shared<visualization_msgs::msg::Marker>(marker));
  }
}


visualization_msgs::msg::MarkerArray
make_markers(size_t num)
{
  visualization_msgs::msg::MarkerArray message;
  message.markers.resize(num);
  return message;
}

void set_increasing_ids(visualization_msgs::msg::MarkerArray * message)
{
  int id = 0;
  for (auto & marker : message->markers) {
    marker.id = id++;
  }
}

void set_decreasing_ids(visualization_msgs::msg::MarkerArray * message)
{
  int id = std::numeric_limits<int>::max();
  for (auto & marker : message->markers) {
    marker.id = id--;
  }
}


/// BENCHMARKS below

// No Check
static void no_check_10(benchmark::State & state)
{
  visualization_msgs::msg::MarkerArray message = make_markers(10);
  set_increasing_ids(&message);
  for (auto _ : state)
  {
    method_no_check(&message);
  }
}
BENCHMARK(no_check_10);

static void no_check_100(benchmark::State & state)
{
  visualization_msgs::msg::MarkerArray message = make_markers(100);
  for (auto _ : state)
  {
    method_no_check(&message);
  }
}
BENCHMARK(no_check_100);

static void no_check_1000(benchmark::State & state)
{
  visualization_msgs::msg::MarkerArray message = make_markers(1000);
  for (auto _ : state)
  {
    method_no_check(&message);
  }
}
BENCHMARK(no_check_1000);

static void no_check_10000(benchmark::State & state)
{
  visualization_msgs::msg::MarkerArray message = make_markers(10000);
  for (auto _ : state)
  {
    method_no_check(&message);
  }
}
BENCHMARK(no_check_10000);

// INCREASING IDS vector of pairs
static void increasing_ids__vector_of_pairs__10(benchmark::State & state)
{
  visualization_msgs::msg::MarkerArray message = make_markers(10);
  set_increasing_ids(&message);
  for (auto _ : state)
  {
    method_vector_of_pairs(&message);
  }
}
BENCHMARK(increasing_ids__vector_of_pairs__10);

static void increasing_ids__vector_of_pairs__100(benchmark::State & state)
{
  visualization_msgs::msg::MarkerArray message = make_markers(100);
  set_increasing_ids(&message);
  for (auto _ : state)
  {
    method_vector_of_pairs(&message);
  }
}
BENCHMARK(increasing_ids__vector_of_pairs__100);

static void increasing_ids__vector_of_pairs__1000(benchmark::State & state)
{
  visualization_msgs::msg::MarkerArray message = make_markers(1000);
  set_increasing_ids(&message);
  for (auto _ : state)
  {
    method_vector_of_pairs(&message);
  }
}
BENCHMARK(increasing_ids__vector_of_pairs__1000);

static void increasing_ids__vector_of_pairs__10000(benchmark::State & state)
{
  visualization_msgs::msg::MarkerArray message = make_markers(10000);
  set_increasing_ids(&message);
  for (auto _ : state)
  {
    method_vector_of_pairs(&message);
  }
}
BENCHMARK(increasing_ids__vector_of_pairs__10000);

// DECREASING IDS vector of pairs

static void decreasing_ids__vector_of_pairs__10(benchmark::State & state)
{
  visualization_msgs::msg::MarkerArray message = make_markers(10);
  set_decreasing_ids(&message);
  for (auto _ : state)
  {
    method_vector_of_pairs(&message);
  }
}
BENCHMARK(decreasing_ids__vector_of_pairs__10);

static void decreasing_ids__vector_of_pairs__100(benchmark::State & state)
{
  visualization_msgs::msg::MarkerArray message = make_markers(100);
  set_decreasing_ids(&message);
  for (auto _ : state)
  {
    method_vector_of_pairs(&message);
  }
}
BENCHMARK(decreasing_ids__vector_of_pairs__100);

static void decreasing_ids__vector_of_pairs__1000(benchmark::State & state)
{
  visualization_msgs::msg::MarkerArray message = make_markers(1000);
  set_decreasing_ids(&message);
  for (auto _ : state)
  {
    method_vector_of_pairs(&message);
  }
}
BENCHMARK(decreasing_ids__vector_of_pairs__1000);

static void decreasing_ids__vector_of_pairs__10000(benchmark::State & state)
{
  visualization_msgs::msg::MarkerArray message = make_markers(10000);
  set_decreasing_ids(&message);
  for (auto _ : state)
  {
    method_vector_of_pairs(&message);
  }
}
BENCHMARK(decreasing_ids__vector_of_pairs__10000);

// INCREASING IDS set insertion
static void increasing_ids__set_insertion__10(benchmark::State & state)
{
  visualization_msgs::msg::MarkerArray message = make_markers(10);
  set_increasing_ids(&message);
  for (auto _ : state)
  {
    method_set_insertion(&message);
  }
}
BENCHMARK(increasing_ids__set_insertion__10);

static void increasing_ids__set_insertion__100(benchmark::State & state)
{
  visualization_msgs::msg::MarkerArray message = make_markers(100);
  set_increasing_ids(&message);
  for (auto _ : state)
  {
    method_set_insertion(&message);
  }
}
BENCHMARK(increasing_ids__set_insertion__100);

static void increasing_ids__set_insertion__1000(benchmark::State & state)
{
  visualization_msgs::msg::MarkerArray message = make_markers(1000);
  set_increasing_ids(&message);
  for (auto _ : state)
  {
    method_set_insertion(&message);
  }
}
BENCHMARK(increasing_ids__set_insertion__1000);

static void increasing_ids__set_insertion__10000(benchmark::State & state)
{
  visualization_msgs::msg::MarkerArray message = make_markers(10000);
  set_increasing_ids(&message);
  for (auto _ : state)
  {
    method_set_insertion(&message);
  }
}
BENCHMARK(increasing_ids__set_insertion__10000);

// DECREASING IDS set insertion
static void decreasing_ids__set_insertion__10(benchmark::State & state)
{
  visualization_msgs::msg::MarkerArray message = make_markers(10);
  set_decreasing_ids(&message);
  for (auto _ : state)
  {
    method_set_insertion(&message);
  }
}
BENCHMARK(decreasing_ids__set_insertion__10);

static void decreasing_ids__set_insertion__100(benchmark::State & state)
{
  visualization_msgs::msg::MarkerArray message = make_markers(100);
  set_decreasing_ids(&message);
  for (auto _ : state)
  {
    method_set_insertion(&message);
  }
}
BENCHMARK(decreasing_ids__set_insertion__100);

static void decreasing_ids__set_insertion__1000(benchmark::State & state)
{
  visualization_msgs::msg::MarkerArray message = make_markers(1000);
  set_decreasing_ids(&message);
  for (auto _ : state)
  {
    method_set_insertion(&message);
  }
}
BENCHMARK(decreasing_ids__set_insertion__1000);

static void decreasing_ids__set_insertion__10000(benchmark::State & state)
{
  visualization_msgs::msg::MarkerArray message = make_markers(10000);
  set_decreasing_ids(&message);
  for (auto _ : state)
  {
    method_set_insertion(&message);
  }
}
BENCHMARK(decreasing_ids__set_insertion__10000);

// INCREASING IDS set insertion id first
static void increasing_ids__set_insertion_id_first__10(benchmark::State & state)
{
  visualization_msgs::msg::MarkerArray message = make_markers(10);
  set_increasing_ids(&message);
  for (auto _ : state)
  {
    method_set_insertion_id_first(&message);
  }
}
BENCHMARK(increasing_ids__set_insertion_id_first__10);

static void increasing_ids__set_insertion_id_first__100(benchmark::State & state)
{
  visualization_msgs::msg::MarkerArray message = make_markers(100);
  set_increasing_ids(&message);
  for (auto _ : state)
  {
    method_set_insertion_id_first(&message);
  }
}
BENCHMARK(increasing_ids__set_insertion_id_first__100);

static void increasing_ids__set_insertion_id_first__1000(benchmark::State & state)
{
  visualization_msgs::msg::MarkerArray message = make_markers(1000);
  set_increasing_ids(&message);
  for (auto _ : state)
  {
    method_set_insertion_id_first(&message);
  }
}
BENCHMARK(increasing_ids__set_insertion_id_first__1000);

static void increasing_ids__set_insertion_id_first__10000(benchmark::State & state)
{
  visualization_msgs::msg::MarkerArray message = make_markers(10000);
  set_increasing_ids(&message);
  for (auto _ : state)
  {
    method_set_insertion_id_first(&message);
  }
}
BENCHMARK(increasing_ids__set_insertion_id_first__10000);

// decreasing IDS set insertion id first
static void decreasing_ids__set_insertion_id_first__10(benchmark::State & state)
{
  visualization_msgs::msg::MarkerArray message = make_markers(10);
  set_decreasing_ids(&message);
  for (auto _ : state)
  {
    method_set_insertion_id_first(&message);
  }
}
BENCHMARK(decreasing_ids__set_insertion_id_first__10);

static void decreasing_ids__set_insertion_id_first__100(benchmark::State & state)
{
  visualization_msgs::msg::MarkerArray message = make_markers(100);
  set_decreasing_ids(&message);
  for (auto _ : state)
  {
    method_set_insertion_id_first(&message);
  }
}
BENCHMARK(decreasing_ids__set_insertion_id_first__100);

static void decreasing_ids__set_insertion_id_first__1000(benchmark::State & state)
{
  visualization_msgs::msg::MarkerArray message = make_markers(1000);
  set_decreasing_ids(&message);
  for (auto _ : state)
  {
    method_set_insertion_id_first(&message);
  }
}
BENCHMARK(decreasing_ids__set_insertion_id_first__1000);

static void decreasing_ids__set_insertion_id_first__10000(benchmark::State & state)
{
  visualization_msgs::msg::MarkerArray message = make_markers(10000);
  set_decreasing_ids(&message);
  for (auto _ : state)
  {
    method_set_insertion_id_first(&message);
  }
}
BENCHMARK(decreasing_ids__set_insertion_id_first__10000);

// INCREASING IDS set insertion id first
static void increasing_ids__set_insertion_id_first_always_lock__10(benchmark::State & state)
{
  visualization_msgs::msg::MarkerArray message = make_markers(10);
  set_increasing_ids(&message);
  for (auto _ : state)
  {
    method_set_insertion_id_first_always_lock(&message);
  }
}
BENCHMARK(increasing_ids__set_insertion_id_first_always_lock__10);

static void increasing_ids__set_insertion_id_first_always_lock__100(benchmark::State & state)
{
  visualization_msgs::msg::MarkerArray message = make_markers(100);
  set_increasing_ids(&message);
  for (auto _ : state)
  {
    method_set_insertion_id_first_always_lock(&message);
  }
}
BENCHMARK(increasing_ids__set_insertion_id_first_always_lock__100);

static void increasing_ids__set_insertion_id_first_always_lock__1000(benchmark::State & state)
{
  visualization_msgs::msg::MarkerArray message = make_markers(1000);
  set_increasing_ids(&message);
  for (auto _ : state)
  {
    method_set_insertion_id_first_always_lock(&message);
  }
}
BENCHMARK(increasing_ids__set_insertion_id_first_always_lock__1000);

static void increasing_ids__set_insertion_id_first_always_lock__10000(benchmark::State & state)
{
  visualization_msgs::msg::MarkerArray message = make_markers(10000);
  set_increasing_ids(&message);
  for (auto _ : state)
  {
    method_set_insertion_id_first_always_lock(&message);
  }
}
BENCHMARK(increasing_ids__set_insertion_id_first_always_lock__10000);

// decreasing IDS set insertion id first
static void decreasing_ids__set_insertion_id_first_always_lock__10(benchmark::State & state)
{
  visualization_msgs::msg::MarkerArray message = make_markers(10);
  set_decreasing_ids(&message);
  for (auto _ : state)
  {
    method_set_insertion_id_first_always_lock(&message);
  }
}
BENCHMARK(decreasing_ids__set_insertion_id_first_always_lock__10);

static void decreasing_ids__set_insertion_id_first_always_lock__100(benchmark::State & state)
{
  visualization_msgs::msg::MarkerArray message = make_markers(100);
  set_decreasing_ids(&message);
  for (auto _ : state)
  {
    method_set_insertion_id_first_always_lock(&message);
  }
}
BENCHMARK(decreasing_ids__set_insertion_id_first_always_lock__100);

static void decreasing_ids__set_insertion_id_first_always_lock__1000(benchmark::State & state)
{
  visualization_msgs::msg::MarkerArray message = make_markers(1000);
  set_decreasing_ids(&message);
  for (auto _ : state)
  {
    method_set_insertion_id_first_always_lock(&message);
  }
}
BENCHMARK(decreasing_ids__set_insertion_id_first_always_lock__1000);

static void decreasing_ids__set_insertion_id_first_always_lock__10000(benchmark::State & state)
{
  visualization_msgs::msg::MarkerArray message = make_markers(10000);
  set_decreasing_ids(&message);
  for (auto _ : state)
  {
    method_set_insertion_id_first_always_lock(&message);
  }
}
BENCHMARK(decreasing_ids__set_insertion_id_first_always_lock__10000);

// INCREASING IDS set insertion id first emplace
static void increasing_ids__set_insertion_id_first_always_lock_emplace__10(benchmark::State & state)
{
  visualization_msgs::msg::MarkerArray message = make_markers(10);
  set_increasing_ids(&message);
  for (auto _ : state)
  {
    method_set_insertion_id_first_always_lock_emplace(&message);
  }
}
BENCHMARK(increasing_ids__set_insertion_id_first_always_lock_emplace__10);

static void increasing_ids__set_insertion_id_first_always_lock_emplace__100(benchmark::State & state)
{
  visualization_msgs::msg::MarkerArray message = make_markers(100);
  set_increasing_ids(&message);
  for (auto _ : state)
  {
    method_set_insertion_id_first_always_lock_emplace(&message);
  }
}
BENCHMARK(increasing_ids__set_insertion_id_first_always_lock_emplace__100);

static void increasing_ids__set_insertion_id_first_always_lock_emplace__1000(benchmark::State & state)
{
  visualization_msgs::msg::MarkerArray message = make_markers(1000);
  set_increasing_ids(&message);
  for (auto _ : state)
  {
    method_set_insertion_id_first_always_lock_emplace(&message);
  }
}
BENCHMARK(increasing_ids__set_insertion_id_first_always_lock_emplace__1000);

static void increasing_ids__set_insertion_id_first_always_lock_emplace__10000(benchmark::State & state)
{
  visualization_msgs::msg::MarkerArray message = make_markers(10000);
  set_increasing_ids(&message);
  for (auto _ : state)
  {
    method_set_insertion_id_first_always_lock_emplace(&message);
  }
}
BENCHMARK(increasing_ids__set_insertion_id_first_always_lock_emplace__10000);

// decreasing IDS set insertion id first emplace
static void decreasing_ids__set_insertion_id_first_always_lock_emplace__10(benchmark::State & state)
{
  visualization_msgs::msg::MarkerArray message = make_markers(10);
  set_decreasing_ids(&message);
  for (auto _ : state)
  {
    method_set_insertion_id_first_always_lock_emplace(&message);
  }
}
BENCHMARK(decreasing_ids__set_insertion_id_first_always_lock_emplace__10);

static void decreasing_ids__set_insertion_id_first_always_lock_emplace__100(benchmark::State & state)
{
  visualization_msgs::msg::MarkerArray message = make_markers(100);
  set_decreasing_ids(&message);
  for (auto _ : state)
  {
    method_set_insertion_id_first_always_lock_emplace(&message);
  }
}
BENCHMARK(decreasing_ids__set_insertion_id_first_always_lock_emplace__100);

static void decreasing_ids__set_insertion_id_first_always_lock_emplace__1000(benchmark::State & state)
{
  visualization_msgs::msg::MarkerArray message = make_markers(1000);
  set_decreasing_ids(&message);
  for (auto _ : state)
  {
    method_set_insertion_id_first_always_lock_emplace(&message);
  }
}
BENCHMARK(decreasing_ids__set_insertion_id_first_always_lock_emplace__1000);

static void decreasing_ids__set_insertion_id_first_always_lock_emplace__10000(benchmark::State & state)
{
  visualization_msgs::msg::MarkerArray message = make_markers(10000);
  set_decreasing_ids(&message);
  for (auto _ : state)
  {
    method_set_insertion_id_first_always_lock_emplace(&message);
  }
}
BENCHMARK(decreasing_ids__set_insertion_id_first_always_lock_emplace__10000);
