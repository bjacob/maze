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
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

// This program generates random mazes using the Twist-and-Merge algorithm from
// [1], Algorithm 2. The output is a HTML document containing one maze per page,
// suitable for printing.
//
// [1]: V. Bellot, M. Cautrès, J-M. Favreau, M. Gonzalez-Thauvin, P. Lafourcade,
//      K. Le Cornec, B. Mosnier, S. Rivière-Wekstein,
//      "How to generate perfect mazes?,"
//      https://uca.hal.science/hal-03174952/document
//
// The algorithm can be summarized as follows:
// 0. In the initial state, all maze cells are surrounded by 4 walls. The maze
//    generation process consists in dropping select walls.
// 1. In the first phase, named Twist, one performs random walks across cells
//    that are still surrounded by 4 walls, opening passages between visited
//    cells. The random walks are "biased" in the sense that they never traverse
//    3 consecutive aligned cells. This results in the twisty maze appearance.
//    This phase ends when all cells have been visited by one of the random
//    walks. At this point, the maze is not yet connected. it is the
//    disconnected union of twisty galleries.
// 2. In the second phase, named Merge, one removes one randomly-selected wall
//    at a time between cells belonging to different connected components. This
//    phase stops as soon as the maze is connected. Because of that, the result
//    is a maze that is not only connected, but simply connected: there is
//    exactly one path connecting any two cells. In other words, the maze is a
//    tree graph.
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
#include <vector>

namespace {

// A cell represents a single cell of a maze during generation. It contains a
// 'component' field, which is effectively used to component connected
// components, which is only useful during generation.
struct Cell {
  // Positive component values indicate connected components created during the
  // Twist phase. The zero value indicates a cell that has not yet been visited
  // during the Twist phase.
  uint32_t component : 30 = 0;
  // The cell has a wall on its East side.
  bool wall_east : 1 = true;
  // The cell has a wall on its South side.
  bool wall_south : 1 = true;
};

// The position of a cell in a maze.
struct Position {
  int row = 0;
  int column = 0;
};

bool operator==(Position a, Position b) {
  return a.row == b.row && a.column == b.column;
}

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

Position next_aligned(Position previous, Position current) {
  return {2 * current.row - previous.row, 2 * current.column - previous.column};
}

using Random = std::function<uint32_t()>;

// Represents a maze during generation by the Twist and Merge algorithm:
// See [1], Algorithm 2.
struct TwistAndMergeMaze {
  int rows = 0;
  int columns = 0;
  std::vector<Cell> cells;

  TwistAndMergeMaze(int rows, int columns, Random random);

  // See [1], Procedure "biaised (sic) random walk".
  void biased_random_walk(Position start_position, int component,
                          Random random);

  bool in_bounds(Position position) const {
    return position.row >= 0 && position.row < rows && position.column >= 0 &&
           position.column < columns;
  }

  Cell &cell_at(Position position) {
    assert(in_bounds(position));
    return cells[position.column + position.row * columns];
  }

  const Cell &cell_at(Position position) const {
    assert(in_bounds(position));
    return cells[position.column + position.row * columns];
  }

  // Helper for biased_random_walk. Performs one step in the random walk.
  std::optional<Position> walk_next(Position previous, Position current,
                                    Random random) const;

  void open_passage(Position current, Position neighbour) {
    if (neighbour.column == current.column + 1) {
      cell_at(current).wall_east = false;
    } else if (current.column == neighbour.column + 1) {
      cell_at(neighbour).wall_east = false;
    } else if (neighbour.row == current.row + 1) {
      cell_at(current).wall_south = false;
    } else if (current.row == neighbour.row + 1) {
      cell_at(neighbour).wall_south = false;
    } else {
      assert(false && "not actually neighbours");
    }
  }
};

// Serializes a maze to a compact hexadecimal representation. Each cell has 2
// bits to store: cell.wall_east, cell.wall_south. Thus, one hex digit
// represents 2 cells.
std::string serialize_hex(const TwistAndMergeMaze &maze) {
  std::string out;
  for (int i = 0; i < maze.rows; ++i) {
    int hex_bits = 0;
    uint8_t hex = 0;
    for (int j = 0; j < maze.columns; ++j) {
      Cell cell = maze.cell_at({i, j});
      hex |= cell.wall_east << hex_bits++;
      hex |= cell.wall_south << hex_bits++;
      if (j == maze.columns - 1 || hex_bits == 4) {
        out += std::format("{:x}", hex);
        hex = 0;
        hex_bits = 0;
      }
    }
    out += "\n";
  }
  return out;
}

std::optional<Position> TwistAndMergeMaze::walk_next(Position previous,
                                                     Position current,
                                                     Random random) const {
  int num_candidates = 0;
  std::array<Position, 4> candidates;
  for (Direction direction :
       {Direction::North, Direction::South, Direction::West, Direction::East}) {
    Position candidate = get_neighbour(current, direction);
    if (candidate == next_aligned(previous, current)) {
      continue;
    }
    if (!in_bounds(candidate)) {
      continue;
    }
    if (cell_at(candidate).component) {
      continue;
    }
    candidates[num_candidates++] = candidate;
  }
  if (!num_candidates) {
    return {};
  }
  return candidates[random() % num_candidates];
}

void TwistAndMergeMaze::biased_random_walk(Position start_position,
                                           int component, Random random) {
  Position current = start_position;
  Position previous = current;
  while (true) {
    cell_at(current).component = component;
    std::optional<Position> next = walk_next(previous, current, random);
    if (!next) {
      break;
    }
    open_passage(current, *next);
    previous = current;
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
  Cell cell = maze.cell_at(p);
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
  for (int row = 0; row < maze.rows; ++row) {
    for (int column = 0; column < maze.columns; ++column) {
      Position current{row, column};
      const Cell &current_cell = maze.cell_at(current);
      for (Direction direction : {Direction::East, Direction::South}) {
        Position neighbour = get_neighbour(current, direction);
        if (maze.in_bounds(neighbour)) {
          if (current_cell.component != maze.cell_at(neighbour).component) {
            walls.emplace_back(current, direction);
          }
        }
      }
    }
  }
  return walls;
}

TwistAndMergeMaze::TwistAndMergeMaze(int rows, int columns, Random random)
    : rows(rows), columns(columns), cells(rows * columns) {
  // Twist phase. Open galleries by performing "biased random walks".
  int component = 0;
  while (true) {
    // Gather all isolated cells: cells that haven't been visited by a random
    // walk yet.
    std::vector<Position> isolated;
    for (int row = 0; row < rows; ++row) {
      for (int column = 0; column < columns; ++column) {
        Position position{row, column};
        if (!cell_at(position).component) {
          isolated.push_back(position);
        }
      }
    }
    // No more isolated cells: the Twist phase has completed.
    if (isolated.empty()) {
      break;
    }
    // Perform a random walk from a randomly selected isolated cell.
    Position walk_start = isolated[random() % isolated.size()];
    biased_random_walk(walk_start, ++component, random);
  }
  // Merge phase. Drop walls one at a time, stop when connected.
  std::vector<Wall> walls = get_walls_between_different_components(*this);
  while (true) {
    if (walls.empty()) {
      break;
    }
    int best_wall_score = INT_MIN;
    int best_wall_index = 0;
    for (ssize_t i = 0, e = walls.size(); i < e; ++i) {
      int score = wall_score(*this, walls[i]);
      if (score > best_wall_score) {
        best_wall_score = score;
        best_wall_index = i;
      }
    }
    Wall selected = walls[best_wall_index];
    Position position_a = selected.position;
    Position position_b = get_neighbour(position_a, selected.direction);
    open_passage(position_a, position_b);
    int component_a = cell_at(position_a).component;
    int component_b = cell_at(position_b).component;
    if (component_a > component_b) {
      std::swap(component_a, component_b);
    }
    for (Cell &cell : cells) {
      if (cell.component == component_b) {
        cell.component = component_a;
      }
    }
    std::vector<Wall> updated_walls;
    updated_walls.reserve(walls.size());
    for (Wall &wall : walls) {
      uint32_t component_a = cell_at(wall.position).component;
      uint32_t component_b =
          cell_at(get_neighbour(wall.position, wall.direction)).component;
      if (component_a == component_b) {
        continue;
      }
      updated_walls.push_back(wall);
    }
    walls = updated_walls;
  }
}

std::string generate_random_maze_hex_str(int rows, int columns) {
  std::random_device device;
  std::mt19937 random_engine(device());
  TwistAndMergeMaze maze(rows, columns,
                         [&random_engine]() { return random_engine(); });
  return serialize_hex(maze);
}

} // namespace

int main(int argc, const char *argv[]) {
  constexpr std::string_view count_arg_prefix = "--count=";
  constexpr std::string_view columns_arg_prefix = "--columns=";
  constexpr std::string_view aspect_arg_prefix = "--aspect=";
  constexpr std::string_view output_arg_prefix = "--output=";

  int count = 4;
  int columns = 48;
  float aspect = 1.25f;
  std::string output_path;
  bool arg_error = false;
  for (int i = 1; i < argc; ++i) {
    std::string_view arg{argv[i]};
    if (arg.starts_with(count_arg_prefix)) {
      count = strtol(arg.data() + count_arg_prefix.size(), nullptr, 10);
    } else if (arg.starts_with(columns_arg_prefix)) {
      columns = strtol(arg.data() + columns_arg_prefix.size(), nullptr, 10);
    } else if (arg.starts_with(aspect_arg_prefix)) {
      aspect = strtof(arg.data() + aspect_arg_prefix.size(), nullptr);
    } else if (arg.starts_with(output_arg_prefix)) {
      output_path = arg.data() + output_arg_prefix.size();
    } else {
      arg_error = true;
      break;
    }
  }

  int rows = aspect * columns;

  if (arg_error || count <= 0 || rows <= 1 || columns <= 1 ||
      !(aspect >= 0.01f && aspect <= 100.0f)) {
    std::cerr << std::format(R"MSG(
Flags:
    {}
        Sets the number of mazes to generate. Each occupies one page.
    {}
        Sets the number of columns of each maze.
    {}
        Sets the aspect ration i.e. the ratio of number of rows to columns.
    {}
        Sets the output HTML file path. If unspecified, output goes to stdout.
)MSG",
                             count_arg_prefix, columns_arg_prefix,
                             aspect_arg_prefix, output_arg_prefix);
    exit(EXIT_FAILURE);
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
    std::string maze_hex = generate_random_maze_hex_str(rows, columns);
    output_string += std::format(R"HTML(
<script type="x-maze" id="maze{}" rows={} columns={}>
{}
</script>
 )HTML",
                                 i, rows, columns, maze_hex);
  }

  // Generate the JS script that will convert maze data into SVG inner-HTML.
  output_string += R"JS(
<script>
  function generate_one_svg(i, rows, columns) {
    maze_id = "maze" + i.toString();
    maze_hex = document.getElementById(maze_id).innerText.split("\n").filter(line => line);
    svg_id = "svg" + i.toString();
    svg_elem = document.getElementById(svg_id);
    cell_size = 20;
    svg_width = columns * cell_size + 1;
    svg_height = rows * cell_size + 1;
    svg_elem.width = svg_width;
    svg_elem.height = svg_height;
    svg_elem.viewBox = "0 0 width=" + svg_width + " height=" + svg_height;
    const line = (x, y, length, horizontal) => {
      var rect = document.createElementNS("http://www.w3.org/2000/svg", 'rect');
      rect.setAttributeNS(null, "x", x * cell_size);
      rect.setAttributeNS(null, "y", y * cell_size);
      rect.setAttributeNS(null, "width", horizontal ? length * cell_size : 1);
      rect.setAttributeNS(null, "height", horizontal ? 1 : length * cell_size);
      rect.setAttributeNS(null, "stroke", "black");
      rect.setAttributeNS(null, "fill", "black");
      rect.setAttributeNS(null, "fill-opacity", 1);
      rect.setAttributeNS(null, "stroke-opacity", 1);
      rect.setAttributeNS(null, "stroke-width", 1); 
      svg_elem.appendChild(rect);
    }
    line(1, 0, columns - 1, true);
    line(0, 1, rows - 1, false);
    for (let r = 0; r < rows; r++) {
      maze_hex_line = maze_hex[r];
      char_pos = 0;
      bit_pos = 0;
      for (let c = 0; c < columns; c++) {
        if (r == rows - 1 && c == columns - 1) {
          continue;
        }
        maze_hex_digit = parseInt(maze_hex_line.charAt(char_pos), 16);
        wall_east = (maze_hex_digit >> bit_pos++) & 1;
        wall_south = (maze_hex_digit >> bit_pos++) & 1;
        if (bit_pos == 4) {
          char_pos++;
          bit_pos = 0;
        }
        if (wall_east) {
          line(c + 1, r, 1, false);
        }
        if (wall_south) {
          line(c, r + 1, 1, true);
        }
      }
    }
  }
  function generate_all_svg(count, rows, columns) {
    for (let i = 0; i < count; i++) {
      generate_one_svg(i, rows, columns);
    }  
  }
</script>
)JS";

  // Generate the body with initially empty <svg> elements whole inner-HTML will
  // be generated by onload.
  output_string += std::format("<body onload=\"generate_all_svg({}, {}, {})\">",
                               count, rows, columns);
  for (int i = 0; i < count; ++i) {
    std::cerr << std::format("Generating maze {}/{} ...\n", i + 1, count);
    if (i > 0) {
      output_string += "<br/><div class=\"page-break\"></div><br/>\n";
    }
    // cell_size determines the on-screen svg size and the ratio of cell width
    // to stroke width. It does not affect printed size since we scale to fit
    // width for printing anyway, but it does affect the overall "darkness", the
    // percentage of paper filled with stroke ink. Higher values result in
    // lighter print.
    const int cell_size = 20;
    std::string svg;
    int svg_width = columns * cell_size + 1;
    int svg_height = rows * cell_size + 1;
    output_string += std::format(R"HTML(
  <svg id="svg{}" viewBox="0 0 {} {}" width="{}" height="{}">
  <div style="font-size: 10px">{}&times;{} &mdash; https://github.com/bjacob/maze </div>
  )HTML",
                                 i, svg_width, svg_height, svg_width,
                                 svg_height, columns, rows);
  }
  output_string += "</body>\n";

  if (output_path.empty()) {
    std::cout << output_string;
  } else {
    std::ofstream(output_path) << output_string;
  }
}
