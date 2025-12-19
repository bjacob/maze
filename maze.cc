// MIT License
//
// Copyright (c) 2025 Benoit Jacob
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

// This program generates random_func mazes using a variant of the
// Twist-and-Merge algorithm from [1], Algorithm 2. The output is a HTML
// document containing one maze per page, suitable for printing.
//
// [1]: V. Bellot, M. Cautrès, J-M. Favreau, M. Gonzalez-Thauvin, P. Lafourcade,
//      K. Le Cornec, B. Mosnier, S. Rivière-Wekstein,
//      "How to generate perfect mazes?,"
//      https://uca.hal.science/hal-03174952/document
//
// The algorithm can be summarized as follows:
// 0. In the initial state, all maze cells are surrounded by 4 walls. The maze
//    generation process consists in dropping select walls.
// 1. In the first phase, named Twist, one performs random_func walks across
// cells
//    that are still surrounded by 4 walls, opening passages between visited
//    cells. The random_func walks are "biased" in the sense that they never
//    traverse 3 consecutive aligned cells. This results in the twisty maze
//    appearance. This phase ends when all cells have been visited by one of the
//    random_func walks. At this point, the maze is not yet connected. it is the
//    disconnected union of twisty galleries.
// 2. In the second phase, named Merge, one removes one randomly-selected wall
//    at a time between cells belonging to different connected components. This
//    phase stops as soon as the maze is connected. Because of that, the result
//    is a maze that is not only connected, but simply connected: there is
//    exactly one path connecting any two cells. In other words, the maze is a
//    tree graph.
//
// This program slightly generalizes the Twist & Merge algorithm by introducing
// 3 parameters, corresponding to these command-line parameters. The original
// Twise & Merge algorithm can be recovered with specific non-default values:
//
//      --weight_straight_path=
//          Float between 0 and 1. Sets the likelihood of straight paths.
//              Default 0.25.
//              Set to 0 to recover the original Twist & Merge algorithm.
//      --weight_u_turn=
//          Float between 0 and 1. Sets the likelihood of U-turns.
//              Default 0.25.
//              Set to 1 to recover the original Twist & Merge algorithm.
//      --biased_merge=
//          Boolean: true or false. Sets the merge strategy.
//              Default true.
//              Set to false to recover the original Twist & Merge algorithm.
//
// This program is self-contained C++20 using only the standard library. It
// should simply compile with any compiler, only requiring a compiler flag to
// set C++20 mode and another to enable optimization. For example, for Clang
// and GCC compilers, this can be built by:
//
//   c++ --std=c++20 -O2 maze.cc -o maze
//
// And the resulting maze program can be run like:
//
//   ./maze --output=maze.html
//
// There are optional command-line flags to control the number of mazes
// generated, and their dimensions. For example:
//
//   ./maze --count=8 --columns=40 --output=maze.html
//
// The generated HTML renders the mazes as SVG elements. However, as SVG is
// not a compact representation, in order to avoid storing large files, SVG is
// only generated at runtime, in a script embedded in the generated HTML
// document, from a more compact serialization of the mazes.

#include <cassert>
#include <format>
#include <fstream>
#include <iostream>
#include <random>
#include <thread>
#include <vector>

namespace {

// A cell represents a single cell of a maze during generation. It contains a
// 'component' field, which is effectively used to component connected
// components, which is only useful during generation.
struct Cell {
  // Positive component values indicate connected components created during the
  // Twist phase. The zero value indicates a cell that has not yet been visited
  // during the Twist phase.
  int component = 0;
  // The cell has a wall on its East side.
  bool wall_east = true;
  // The cell has a wall on its South side.
  bool wall_south = true;
};

// The position of a cell in a maze.
struct Position {
  int row = 0;
  int column = 0;
};

enum Direction { North = 0, South = 1, West = 2, East = 3 };

Position get_neighbour(Position current, Direction direction) {
  switch (direction) {
  case Direction::North:
    return {current.row - 1, current.column};
  case Direction::South:
    return {current.row + 1, current.column};
  case Direction::West:
    return {current.row, current.column - 1};
  case Direction::East:
    return {current.row, current.column + 1};
  default:
    assert(false);
    return {};
  }
}

using RandomFunc = std::function<uint32_t()>;

// Represents a maze during generation by the Twist and Merge algorithm:
// See [1], Algorithm 2.
class TwistAndMergeMaze {
public:
  int rows() const { return _rows; }
  int columns() const { return _columns; }
  float weight_straight_path() const { return _weight_straight_path; }
  float weight_u_turn() const { return _weight_u_turn; }
  bool biased_merge() const { return _biased_merge; }

  TwistAndMergeMaze(int rows, int columns, float weight_straight_path,
                    float weight_u_turn, bool biased_merge,
                    RandomFunc random_func);

  // See [1], Procedure "biaised (sic) random_func walk".
  void biased_random_walk(Position start_position, int component,
                          RandomFunc random_func);

  bool in_bounds(Position position) const {
    return position.row >= 0 && position.row < rows() && position.column >= 0 &&
           position.column < columns();
  }

  Cell &cell(Position position) {
    assert(in_bounds(position));
    return _cells[position.column + position.row * columns()];
  }

  const Cell &cell(Position position) const {
    assert(in_bounds(position));
    return _cells[position.column + position.row * columns()];
  }

private:
  void twist_phase(RandomFunc random_func);
  void merge_phase(RandomFunc random_func);

  // Helper for biased_random_walk. Performs one step in the random_func walk.
  std::optional<Position> walk_next(std::optional<Position> previous2,
                                    std::optional<Position> previous1,
                                    Position current,
                                    RandomFunc random_func) const;

  void open_passage(Position current, Position neighbour);

  int _rows = 0;
  int _columns = 0;
  std::vector<Cell> _cells;

  // Parameters controlling the maze random_func generation. The parameter
  // values that allow matching the original Twist & Merge algorithm from the
  // paper [1] are:
  //   _weight_straight_path = 0.0
  //   _weight_u_turn = 1.0
  //   _biased_merge = false
  // but we prefer
  //   _weight_straight_path = 0.25
  //   _weight_u_turn = 0.25
  //   _biased_merge = true
  float _weight_straight_path;
  float _weight_u_turn;
  bool _biased_merge;
};

// Serializes a maze to a compact hexadecimal representation. Each cell has 2
// bits to store: cell.wall_east, cell.wall_south. Thus, one hex digit
// represents 2 cells.
std::string serialize_hex(const TwistAndMergeMaze &maze) {
  std::string out;
  for (int i = 0; i < maze.rows(); ++i) {
    int hex_bits = 0;
    uint8_t hex = 0;
    for (int j = 0; j < maze.columns(); ++j) {
      Cell cell = maze.cell({i, j});
      hex |= cell.wall_east << hex_bits++;
      hex |= cell.wall_south << hex_bits++;
      if (j == maze.columns() - 1 || hex_bits == 4) {
        out += std::format("{:x}", hex);
        hex = 0;
        hex_bits = 0;
      }
    }
    out += "\n";
  }
  return out;
}

template <typename T, size_t n> int amplitude(const std::array<T, n> &array) {
  T min = array[0];
  T max = array[0];
  for (T a : array) {
    min = std::min(min, a);
    max = std::max(max, a);
  }
  return max - min;
}

std::optional<Position>
TwistAndMergeMaze::walk_next(std::optional<Position> previous2,
                             std::optional<Position> previous1,
                             Position current, RandomFunc random_func) const {
  int num_candidates = 0;
  std::array<Position, 4> candidates;
  std::array<float, 4> weights;
  float weights_sum = 0.f;

  for (Direction direction :
       {Direction::North, Direction::South, Direction::West, Direction::East}) {
    Position candidate = get_neighbour(current, direction);
    float weight = 1.f;
    if (!in_bounds(candidate)) {
      continue;
    }
    if (cell(candidate).component) {
      continue;
    }
    if (previous1) {
      if (candidate.row == current.row && current.row == previous1->row) {
        weight = weight_straight_path();
      }
      if (candidate.column == current.column &&
          current.column == previous1->column) {
        weight = weight_straight_path();
      }
      if (amplitude(std::array{candidate.row, current.row, previous1->row,
                               previous2->row}) <= 1 &&
          amplitude(std::array{candidate.column, current.column,
                               previous1->column, previous2->column}) <= 1) {
        weight = weight_u_turn();
      }
    }
    candidates[num_candidates] = candidate;
    weights[num_candidates] = weight;
    num_candidates++;
    weights_sum += weight;
  }
  if (!num_candidates) {
    return {};
  }
  const int modulus = 1 << 16;
  float random_w = (random_func() % modulus) * (1.0f / modulus) * weights_sum;
  for (int i = 0; i < num_candidates - 1; ++i) {
    if (random_w < weights[i]) {
      return candidates[i];
    }
    random_w -= weights[i];
  }
  return candidates[num_candidates - 1];
}

void TwistAndMergeMaze::open_passage(Position current, Position neighbour) {
  if (neighbour.column == current.column + 1) {
    cell(current).wall_east = false;
  } else if (current.column == neighbour.column + 1) {
    cell(neighbour).wall_east = false;
  } else if (neighbour.row == current.row + 1) {
    cell(current).wall_south = false;
  } else if (current.row == neighbour.row + 1) {
    cell(neighbour).wall_south = false;
  } else {
    assert(false && "not actually neighbours");
  }
}

void TwistAndMergeMaze::biased_random_walk(Position start_position,
                                           int component,
                                           RandomFunc random_func) {
  Position current = start_position;
  std::optional<Position> previous1, previous2;
  while (true) {
    cell(current).component = component;
    std::optional<Position> next =
        walk_next(previous2, previous1, current, random_func);
    if (!next) {
      break;
    }
    open_passage(current, *next);
    previous2 = previous1;
    previous1 = current;
    current = *next;
  }
}

// A wall separating two connected components during the Merge phase of the
// Twist-and-Merge algorithm.
struct Wall {
  // Position of the cell that this wall is either an East or a South wall of.
  Position position;
  // May be either East or South. This restriction removes ambiguity as
  // otherwise a given wall could be equivalently seen as either the South wall
  // of a cell or the North wall of the southern neighbour cell.
  Direction direction;
};

bool has_wall(const TwistAndMergeMaze &maze, Position p, Direction d) {
  if (!maze.in_bounds(p)) {
    return true;
  }
  Cell cell = maze.cell(p);
  switch (d) {
  case Direction::South:
    return cell.wall_south;
  case Direction::East:
    return cell.wall_east;
  case Direction::North:
    return has_wall(maze, get_neighbour(p, Direction::North), Direction::South);
  case Direction::West:
    return has_wall(maze, get_neighbour(p, Direction::West), Direction::East);
  }
};

int wall_score(const TwistAndMergeMaze &maze, Wall wall) {
  Position current = wall.position;
  Position neighbour = get_neighbour(current, wall.direction);
  int score = 0;
  for (auto d :
       {Direction::East, Direction::West, Direction::North, Direction::South}) {
    score += has_wall(maze, current, d);
    score += has_wall(maze, neighbour, d);
  }
  return score;
}

std::vector<Wall>
get_walls_between_different_components(const TwistAndMergeMaze &maze) {
  std::vector<Wall> walls;
  for (int row = 0; row < maze.rows(); ++row) {
    for (int column = 0; column < maze.columns(); ++column) {
      Position current{row, column};
      const Cell &current_cell = maze.cell(current);
      for (Direction direction : {Direction::East, Direction::South}) {
        Position neighbour = get_neighbour(current, direction);
        if (maze.in_bounds(neighbour)) {
          if (current_cell.component != maze.cell(neighbour).component) {
            walls.emplace_back(current, direction);
          }
        }
      }
    }
  }
  return walls;
}

TwistAndMergeMaze::TwistAndMergeMaze(int rows, int columns,
                                     float weight_straight_path,
                                     float weight_u_turn, bool biased_merge,
                                     RandomFunc random_func)
    : _rows(rows), _columns(columns), _cells(rows * columns),
      _weight_straight_path(weight_straight_path),
      _weight_u_turn(weight_u_turn), _biased_merge(biased_merge) {
  twist_phase(random_func);
  merge_phase(random_func);
}

void TwistAndMergeMaze::twist_phase(RandomFunc random_func) {
  // Twist phase. Open galleries by performing "biased random_func walks".
  int component = 0;
  while (true) {
    // Gather all isolated cells: cells that haven't been visited by a
    // random_func walk yet.
    std::vector<Position> isolated;
    for (int row = 0; row < rows(); ++row) {
      for (int column = 0; column < columns(); ++column) {
        Position position{row, column};
        if (!cell(position).component) {
          isolated.push_back(position);
        }
      }
    }
    // No more isolated cells: the Twist phase has completed.
    if (isolated.empty()) {
      break;
    }
    // Perform a random_func walk from a randomly selected isolated cell.
    Position walk_start = isolated[random_func() % isolated.size()];
    biased_random_walk(walk_start, ++component, random_func);
  }
}

void TwistAndMergeMaze::merge_phase(RandomFunc random_func) {
  // Merge phase. Drop walls one at a time, stop when connected.
  std::vector<Wall> walls = get_walls_between_different_components(*this);
  while (true) {
    if (walls.empty()) {
      break;
    }
    int best_wall_score = INT_MIN;
    ssize_t starting_wall_index = random_func() % walls.size();
    int best_wall_index = starting_wall_index;
    if (biased_merge()) {
      ssize_t i = starting_wall_index;
      do {
        int score = wall_score(*this, walls[i]);
        if (score > best_wall_score) {
          best_wall_score = score;
          best_wall_index = i;
        }
        ++i;
        if (i == static_cast<ssize_t>(walls.size())) {
          i = 0;
        }
      } while (i != starting_wall_index);
    }
    Wall selected = walls[best_wall_index];
    Position position_a = selected.position;
    Position position_b = get_neighbour(position_a, selected.direction);
    open_passage(position_a, position_b);
    int component_a = cell(position_a).component;
    int component_b = cell(position_b).component;
    if (component_a > component_b) {
      std::swap(component_a, component_b);
    }
    for (Cell &cell : _cells) {
      if (cell.component == component_b) {
        cell.component = component_a;
      }
    }
    std::vector<Wall> updated_walls;
    updated_walls.reserve(walls.size());
    for (Wall &wall : walls) {
      uint32_t component_a = cell(wall.position).component;
      uint32_t component_b =
          cell(get_neighbour(wall.position, wall.direction)).component;
      if (component_a == component_b) {
        continue;
      }
      updated_walls.push_back(wall);
    }
    walls = updated_walls;
  }
}

std::string generate_random_maze_hex_str(int rows, int columns,
                                         float weight_straight_path,
                                         float weight_u_turn,
                                         bool biased_merge) {
  std::random_device device;
  std::mt19937 random_engine(device());
  TwistAndMergeMaze maze(rows, columns, weight_straight_path, weight_u_turn,
                         biased_merge,
                         [&random_engine]() { return random_engine(); });
  return serialize_hex(maze);
}

} // namespace

int main(int argc, const char *argv[]) {
  constexpr std::string_view count_arg_prefix = "--count=";
  constexpr std::string_view columns_arg_prefix = "--columns=";
  constexpr std::string_view aspect_arg_prefix = "--aspect=";
  constexpr std::string_view weight_straight_arg_prefix =
      "--weight_straight_path=";
  constexpr std::string_view weight_u_turn_arg_prefix = "--weight_u_turn=";
  constexpr std::string_view biased_merge_arg_prefix = "--biased_merge=";
  constexpr std::string_view output_arg_prefix = "--output=";

  int count = 4;
  int columns = 48;
  float aspect = 1.25f;
  const float weight_straight_path_default = 0.25f;
  const float weight_u_turn_default = 0.25f;
  const std::string biased_merge_str_default = "true";
  float weight_straight_path = weight_straight_path_default;
  float weight_u_turn = weight_u_turn_default;
  std::string biased_merge_str = biased_merge_str_default;

  std::string output_path;
  bool arg_error = false;
  for (int i = 1; i < argc; ++i) {
    std::string_view arg{argv[i]};
    if (arg.starts_with(count_arg_prefix)) {
      count = strtol(arg.data() + count_arg_prefix.size(), nullptr, 10);
    } else if (arg.starts_with(columns_arg_prefix)) {
      columns = strtol(arg.data() + columns_arg_prefix.size(), nullptr, 10);
    } else if (arg.starts_with(biased_merge_arg_prefix)) {
      biased_merge_str = arg.data() + biased_merge_arg_prefix.size();
    } else if (arg.starts_with(aspect_arg_prefix)) {
      aspect = strtof(arg.data() + aspect_arg_prefix.size(), nullptr);
    } else if (arg.starts_with(weight_straight_arg_prefix)) {
      weight_straight_path =
          strtof(arg.data() + weight_straight_arg_prefix.size(), nullptr);
    } else if (arg.starts_with(weight_u_turn_arg_prefix)) {
      weight_u_turn =
          strtof(arg.data() + weight_u_turn_arg_prefix.size(), nullptr);
    } else if (arg.starts_with(output_arg_prefix)) {
      output_path = arg.data() + output_arg_prefix.size();
    } else {
      arg_error = true;
      break;
    }
  }

  int rows = aspect * columns;

  if (arg_error ||
      !(count > 0 && rows > 1 && columns > 1 && aspect >= 0.01f &&
        aspect <= 100.0f && weight_straight_path >= 0.f &&
        weight_straight_path <= 1.0f && weight_u_turn >= 0.f &&
        weight_u_turn <= 1.0f &&
        (biased_merge_str == "true" || biased_merge_str == "false"))) {
    std::cerr << std::format(R"MSG(Flags:
    {}
        Sets the output HTML file path. If unspecified, output goes to stdout.
    {}
        Sets the number of mazes to generate. Each occupies one page.
    {}
        Sets the number of columns of each maze.
    {}
        Float. Sets the aspect ratio i.e. the ratio of number of rows to columns.
    {}
        Float between 0 and 1. Sets the likelihood of straight paths.
            Default {}.
            Set to 0 to recover the original Twist & Merge algorithm.
    {}
        Float between 0 and 1. Sets the likelihood of U-turns.
            Default {}.
            Set to 1 to recover the original Twist & Merge algorithm.
    {}
        Boolean: true or false. Sets the merge strategy.
            Default {}.
            Set to false to recover the original Twist & Merge algorithm.
)MSG",
                             output_arg_prefix, count_arg_prefix,
                             columns_arg_prefix, aspect_arg_prefix,
                             weight_straight_arg_prefix,
                             weight_straight_path_default,
                             weight_u_turn_arg_prefix, weight_u_turn_default,
                             biased_merge_arg_prefix, biased_merge_str_default);
    exit(EXIT_FAILURE);
  }
  bool biased_merge = (biased_merge_str == "true");

  const int num_threads = std::thread::hardware_concurrency();
  std::vector<std::string> maze_hex_strings(count);
  {
    std::vector<std::jthread> threads;
    for (int i = 0; i < num_threads; ++i) {
      threads.emplace_back([&, i]() {
        const int maze_start = i * count / num_threads;
        const int maze_end = (i + 1) * count / num_threads;
        for (int m = maze_start; m < maze_end; ++m) {
          maze_hex_strings[m] = generate_random_maze_hex_str(
              rows, columns, weight_straight_path, weight_u_turn, biased_merge);
        }
      });
    }
  }

  // Start HTML file. Generate the <style> element.
  std::string output_string = R"HTML(<html>
<style>
@media print {
  .page, .page-break { break-after: page; }
  svg {
    width: 100%;
    height: auto;
    display: block;
  }
}
</style>)HTML";

  // Generate the maze data as <script> elements with custom type.
  for (int i = 0; i < count; ++i) {
    output_string += std::format(R"HTML(
<script type="x-maze" id="maze{}" rows={} columns={}>
{}</script>
 )HTML",
                                 i, rows, columns, maze_hex_strings[i]);
  }

  // Generate the body onload callback to render the mazes.
  output_string += R"HTML(
<script src="https://bjacob.github.io/maze/maze_svg_render.js"></script>
<script>
  function maze_svg_render_all(count) {
    for (let i = 0; i < count; i++) {
      maze_element = document.getElementById('maze' + i.toString());
      dst_element = document.getElementById('render' + i.toString());
      // cell_size determines the on-screen svg size and the ratio of cell width
      // to stroke width. It does not affect printed size since we scale to fit
      // width for printing anyway, but it does affect the overall "darkness", the
      // percentage of paper filled with stroke ink. Higher values result in
      // lighter print.
      cell_size = 20;
      setTimeout(maze_svg_render, /*delay=*/0, maze_element, dst_element, cell_size);
    }  
  }
</script>
)HTML";

  // Generate the body with initially empty <svg> elements whole inner-HTML will
  // be generated by onload.
  output_string +=
      std::format("<body onload=\"maze_svg_render_all({})\">", count);
  for (int i = 0; i < count; ++i) {
    if (i > 0) {
      output_string += "<br/><div class=\"page-break\"></div><br/>\n";
    }
    output_string += std::format(R"HTML(
  <div id="render{}" viewBox="0 0 1000 1000"></div><br/>
  )HTML",
                                 i);
  }
  output_string += "</body>\n";

  if (output_path.empty()) {
    std::cout << output_string;
  } else {
    std::ofstream(output_path) << output_string;
  }
}
