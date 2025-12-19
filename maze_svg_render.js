function maze_svg_render(maze_element, dst_element, cell_size) {
  rows = maze_element.getAttribute('rows')
  columns = maze_element.getAttribute('columns')
  svg_width = columns * cell_size + 1;
  svg_height = rows * cell_size + 1;
  svg = '<svg' +
      ' width="' + svg_width + '"' +
      ' height="' + svg_height + '"' +
      ' viewBox="0 0 ' + svg_width + ' ' + svg_height + '"' +
      ' xmlns="http://www.w3.org/2000/svg">\n';
  // Helper to render one horizontal or vertical line.
  const line = (x, y, length, horizontal) => {
    svg_rect = '<rect x="' + (x * cell_size) + '"' +
        ' y="' + (y * cell_size) + '"' +
        ' width="' + (horizontal ? length * cell_size + 1 : 1) + '"' +
        ' height="' + (horizontal ? 1 : length * cell_size + 1) + '"' +
        ' stroke = "black"' +
        ' fill="black" />\n';
    svg += svg_rect;
  };
  // Top edge. The 1 here are to leave an entrance at the top-left corner.
  line(1, 0, columns - 1, true);
  // Left edge.
  line(0, 1, rows - 1, false);
  // Iterate over rows.
  maze_hex = maze_element.innerText.split('\n').filter(line => line);
  for (let r = 0; r < rows; r++) {
    maze_hex_line = maze_hex[r];
    char_pos = 0;
    bit_pos = 0;
    // Iterate over columns.
    for (let c = 0; c < columns; c++) {
      // Leave an exit at the bottom-right corner.
      if (r == rows - 1 && c == columns - 1) {
        continue;
      }
      // Each hex digit, 4 bits, encodes 2 cells (2 bits each). 1 bit == 1 wall.
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
  svg += '</svg>\n'
  const parser = new DOMParser();
  const doc = parser.parseFromString(svg, 'image/svg+xml');
  const svgImg = doc.querySelector('svg')
  dst_element.appendChild(svgImg);
}
