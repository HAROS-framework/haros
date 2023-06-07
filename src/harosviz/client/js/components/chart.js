// SPDX-License-Identifier: MIT
// Copyright © 2023 André Santos

// -----------------------------------------------------------------------------
//  Constants
// -----------------------------------------------------------------------------

const alphabet = [
  { letter: "A", frequency: 0.08167 },
  { letter: "B", frequency: 0.01492 },
  { letter: "C", frequency: 0.02782 },
  { letter: "D", frequency: 0.04253 },
  { letter: "E", frequency: 0.12702 },
  { letter: "F", frequency: 0.02268 },
];

// -----------------------------------------------------------------------------
//  Setup
// -----------------------------------------------------------------------------

function newChart() {
  // Declare the chart dimensions and margins.
  const width = 400;
  const height = 300;
  const marginTop = 30;
  const marginRight = 0;
  const marginBottom = 30;
  const marginLeft = 40;

  // Declare the x (horizontal position) scale.
  const x = d3.scaleBand()
      .domain(d3.groupSort(alphabet, ([d]) => -d.frequency, (d) => d.letter)) // descending frequency
      .range([marginLeft, width - marginRight])
      .padding(0.1);

  // Declare the y (vertical position) scale.
  const y = d3.scaleLinear()
      .domain([0, d3.max(alphabet, (d) => d.frequency)])
      .range([height - marginBottom, marginTop]);

  // Create the SVG container.
  const svg = d3.create("svg")
      .attr("width", width)
      .attr("height", height)
      .attr("viewBox", [0, 0, width, height])
      .attr("style", "max-width: 100%; height: auto;");

  // Add a rect for each bar.
  svg.append("g")
      .attr("fill", "steelblue")
    .selectAll()
    .data(alphabet)
    .join("rect")
      .attr("x", (d) => x(d.letter))
      .attr("y", (d) => y(d.frequency))
      .attr("height", (d) => y(0) - y(d.frequency))
      .attr("width", x.bandwidth());

  // Add the x-axis and label.
  svg.append("g")
      .attr("transform", `translate(0,${height - marginBottom})`)
      .call(d3.axisBottom(x).tickSizeOuter(0));

  // Add the y-axis and label, and remove the domain line.
  svg.append("g")
      .attr("transform", `translate(${marginLeft},0)`)
      .call(d3.axisLeft(y).tickFormat((y) => (y * 100).toFixed()))
      .call(g => g.select(".domain").remove())
      .call(g => g.append("text")
          .attr("x", -marginLeft)
          .attr("y", 10)
          .attr("fill", "currentColor")
          .attr("text-anchor", "start")
          .text("↑ Frequency (%)"));

  // Return the SVG element.
  return svg.node();
}
